#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/PubServoInfo_.hpp"
#include "msg/ArmString_.hpp"
#include "msg/SetServoDumping_.hpp"

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"

#include "d1_control/joint_interval.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using namespace std::chrono_literals;
using fsuservo::FSUS_Protocol;
using fsuservo::FSUS_Servo;
using unitree::robot::ChannelFactory;
using DampingCommand = unitree_arm::msg::dds_::SetServoDumping_;
using ServoFeedback = unitree_arm::msg::dds_::PubServoInfo_;
using ArmString = unitree_arm::msg::dds_::ArmString_;

constexpr std::size_t kJointCount = 7U;
constexpr char kServoPort[] = "/dev/ttyS4";
constexpr char kDampingTopic[] = "set_servo_dumping";
constexpr char kFeedbackTopic[] = "current_servo_angle";
constexpr char kJointSegmentTopic[] = "d1_native_joint_segment";
constexpr double kSerialCommandSkewMs = 1.4;
constexpr std::uint16_t kDefaultMinimumJointIntervalMs = 10U;
constexpr std::uint16_t kMaximumConfigurableMinimumJointIntervalMs = 1000U;

volatile sig_atomic_t g_stop_requested = 0;

void request_stop(int)
{
  g_stop_requested = 1;
}

std::uint16_t minimum_joint_interval_ms_from_environment()
{
  const char * value = std::getenv("D1_MIN_JOINT_INTERVAL_MS");
  if (value == nullptr || *value == '\0')
  {
    return kDefaultMinimumJointIntervalMs;
  }
  char * end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed < 1L ||
      parsed > static_cast<long>(kMaximumConfigurableMinimumJointIntervalMs))
  {
    throw std::invalid_argument(
      "D1_MIN_JOINT_INTERVAL_MS must be an integer in [1, 1000]");
  }
  return static_cast<std::uint16_t>(parsed);
}

struct NativeSegment
{
  std::uint64_t sequence{0U};
  std::uint16_t motion_profile{2U};
  std::uint16_t duration_ms{0U};
  std::uint16_t acceleration_ms{0U};
  std::uint16_t deceleration_ms{0U};
  std::array<float, kJointCount> angle_deg{};
};

bool extract_unsigned(
  const std::string & payload, const char * key, std::uint64_t & output)
{
  const std::string token = std::string("\"") + key + "\"";
  const auto key_at = payload.find(token);
  const auto colon = key_at == std::string::npos ? std::string::npos :
    payload.find(':', key_at + token.size());
  if (colon == std::string::npos) return false;
  char * end = nullptr;
  const auto parsed = std::strtoull(payload.c_str() + colon + 1, &end, 10);
  if (end == payload.c_str() + colon + 1) return false;
  output = parsed;
  return true;
}

bool parse_native_segment(const std::string & payload, NativeSegment & segment)
{
  std::uint64_t motion_profile = 2U;
  std::uint64_t duration = 0U;
  std::uint64_t acceleration = 0U;
  std::uint64_t deceleration = 0U;
  // Missing motion_profile keeps compatibility with older host gateways,
  // whose native segments always meant common arrival.
  extract_unsigned(payload, "motion_profile", motion_profile);
  if (!extract_unsigned(payload, "seq", segment.sequence) ||
    !extract_unsigned(payload, "duration_ms", duration) ||
    !extract_unsigned(payload, "acceleration_ms", acceleration) ||
    !extract_unsigned(payload, "deceleration_ms", deceleration) ||
    (motion_profile != 2U && motion_profile != 3U) ||
    duration == 0U || duration > 65535U || acceleration > 65535U ||
    deceleration > 65535U || acceleration + deceleration > duration)
  {
    return false;
  }
  const auto key = payload.find("\"angles_deg\"");
  const auto open = key == std::string::npos ? std::string::npos : payload.find('[', key);
  if (open == std::string::npos) return false;
  const char * cursor = payload.c_str() + open + 1;
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    char * end = nullptr;
    const double value = std::strtod(cursor, &end);
    if (end == cursor || !std::isfinite(value)) return false;
    segment.angle_deg[joint] = static_cast<float>(value);
    cursor = end;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (joint + 1U < kJointCount)
    {
      if (*cursor != ',') return false;
      ++cursor;
    }
  }
  while (*cursor == ' ' || *cursor == '\t') ++cursor;
  if (*cursor != ']') return false;
  segment.motion_profile = static_cast<std::uint16_t>(motion_profile);
  segment.duration_ms = static_cast<std::uint16_t>(duration);
  segment.acceleration_ms = static_cast<std::uint16_t>(acceleration);
  segment.deceleration_ms = static_cast<std::uint16_t>(deceleration);
  return true;
}

class D1ControlNode
{
public:
  D1ControlNode()
  : protocol_(kServoPort, FSUS_DEFAULT_BAUDRATE),
    servos_{
      FSUS_Servo(0, &protocol_), FSUS_Servo(1, &protocol_),
      FSUS_Servo(2, &protocol_), FSUS_Servo(3, &protocol_),
      FSUS_Servo(4, &protocol_), FSUS_Servo(5, &protocol_),
      FSUS_Servo(6, &protocol_)},
    minimum_joint_interval_ms_(minimum_joint_interval_ms_from_environment())
  {
  }

  bool start()
  {
    feedback_publisher_ = std::make_unique<
      unitree::robot::ChannelPublisher<ServoFeedback>>(kFeedbackTopic);
    feedback_publisher_->InitChannel();

    segment_subscriber_ = std::make_unique<
      unitree::robot::ChannelSubscriber<ArmString>>(kJointSegmentTopic);
    segment_subscriber_->InitChannel(
      [this](const void * raw) {receive_native_segment(raw);}, 8);

    damping_subscriber_ = std::make_unique<
      unitree::robot::ChannelSubscriber<DampingCommand>>(kDampingTopic);
    damping_subscriber_->InitChannel(
      [this](const void * raw) {receive_damping_command(raw);}, 16);

    io_thread_ = std::thread([this]() {io_loop();});
    std::cout
      << "D1 control node ready: timed seven-joint segments; "
      << "serial I/O has one owner; uniform_min_joint_interval_ms="
      << minimum_joint_interval_ms_ << std::endl;
    return true;
  }

  void stop()
  {
    if (stopping_.exchange(true))
    {
      return;
    }
    state_changed_.notify_all();
    if (io_thread_.joinable())
    {
      io_thread_.join();
    }
    if (segment_subscriber_)
    {
      segment_subscriber_->CloseChannel();
      segment_subscriber_.reset();
    }
    if (damping_subscriber_)
    {
      damping_subscriber_->CloseChannel();
      damping_subscriber_.reset();
    }
    if (feedback_publisher_)
    {
      feedback_publisher_->CloseChannel();
      feedback_publisher_.reset();
    }
  }

private:
  void receive_native_segment(const void * raw)
  {
    const auto * message = static_cast<const ArmString *>(raw);
    NativeSegment segment;
    if (!parse_native_segment(message->data_(), segment))
    {
      ++invalid_segments_;
      std::cerr << "Rejected malformed native joint segment" << std::endl;
      return;
    }
    const auto skew_budget = static_cast<std::uint16_t>(
      std::lround((kJointCount - 1U) * kSerialCommandSkewMs));
    if (segment.duration_ms <= segment.acceleration_ms +
      segment.deceleration_ms + skew_budget)
    {
      ++invalid_segments_;
      std::cerr << "Rejected native segment too short for serial skew compensation"
                << std::endl;
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_segment_ && segment.sequence <= latest_segment_->sequence)
    {
      ++stale_segments_;
      return;
    }
    if (latest_segment_) ++superseded_segments_;
    latest_segment_ = segment;
    state_changed_.notify_one();
  }

  void receive_damping_command(const void * raw)
  {
    const auto * message = static_cast<const DampingCommand *>(raw);
    if (message->id_() >= kJointCount)
    {
      std::cerr << "Rejected invalid damping command id="
                << static_cast<int>(message->id_()) << std::endl;
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    damping_commands_[message->id_()] = *message;
    state_changed_.notify_one();
  }

  void io_loop()
  {
    auto next_feedback_joint = std::chrono::steady_clock::now();
    while (!stopping_.load() && !g_stop_requested)
    {
      std::optional<NativeSegment> segment;
      std::array<std::optional<DampingCommand>, kJointCount> damping;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_changed_.wait_until(
          lock, next_feedback_joint,
          [this]()
          {
            return stopping_.load() || g_stop_requested ||
                   std::any_of(
                     damping_commands_.begin(), damping_commands_.end(),
                     [](const auto & command) {return command.has_value();}) ||
                   latest_segment_.has_value();
          });
        if (latest_segment_)
        {
          segment = *latest_segment_;
          latest_segment_.reset();
        }
        damping.swap(damping_commands_);
      }

      if (segment)
      {
        dispatch_native_segment(*segment);
      }
      for (const auto & command : damping)
      {
        if (command)
        {
          servos_[command->id_()].setDamping(command->power_());
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (now >= next_feedback_joint)
      {
        query_next_feedback_joint();
        next_feedback_joint = std::chrono::steady_clock::now() + 14ms;
      }
    }
  }

  void dispatch_native_segment(const NativeSegment & segment)
  {
    if (segment.motion_profile == 3U)
    {
      dispatch_uniform_joint_speed_segment(segment);
      return;
    }
    const auto started = std::chrono::steady_clock::now();
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      const auto skew_ms = static_cast<std::uint16_t>(
        std::lround(joint * kSerialCommandSkewMs));
      servos_[joint].setRawAngle(
        segment.angle_deg[joint],
        static_cast<std::uint16_t>(segment.duration_ms - skew_ms));
    }
    ++dispatched_segments_;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    std::cout << "Dispatched native joint segment: seq=" << segment.sequence
              << " vendor_setRawAngle common_arrival_ms=" << segment.duration_ms
              << " serial_burst_ms=" << std::fixed << std::setprecision(3)
              << elapsed_ms << std::endl;
  }

  void dispatch_uniform_joint_speed_segment(const NativeSegment & segment)
  {
    if (completed_feedback_cycles_ == 0U)
    {
      ++invalid_segments_;
      std::cerr << "Rejected uniform-joint-speed segment before first complete feedback cycle"
                << std::endl;
      return;
    }

    std::array<double, kJointCount> delta_deg{};
    double maximum_delta_deg = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      delta_deg[joint] = std::abs(
        static_cast<double>(segment.angle_deg[joint] - feedback_angles_[joint]));
      maximum_delta_deg = std::max(maximum_delta_deg, delta_deg[joint]);
    }
    if (maximum_delta_deg < 0.05)
    {
      std::cout << "Uniform-joint-speed segment already at target: seq="
                << segment.sequence << std::endl;
      return;
    }

    const auto started = std::chrono::steady_clock::now();
    const double speed_deg_s = maximum_delta_deg * 1000.0 / segment.duration_ms;
    std::array<std::uint16_t, kJointCount> interval_ms{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      const double scale = delta_deg[joint] / maximum_delta_deg;
      // Match Unitree's funcode=2, mode=1 implementation exactly: derive an
      // independent delay from displacement, then call setRawAngle().  This is
      // intentionally not setRawAngleByInterval(); D1095 showed severe Joint3
      // overshoot when that different low-level primitive was given a short
      // interval, while the vendor path accurately executed a 0.5 degree move.
      const auto requested_interval_ms = static_cast<std::uint16_t>(std::clamp<long long>(
        static_cast<long long>(static_cast<double>(segment.duration_ms) * scale),
        1LL, 65535LL));
      interval_ms[joint] = d1_control::enforceMinimumJointInterval(
        requested_interval_ms, minimum_joint_interval_ms_);
      servos_[joint].setRawAngle(segment.angle_deg[joint], interval_ms[joint]);
    }
    ++dispatched_segments_;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    std::cout << "Dispatched uniform-joint-speed segment: seq=" << segment.sequence
              << " vendor_mode1_speed_deg_s=" << std::fixed << std::setprecision(3)
              << speed_deg_s << " max_duration_ms=" << segment.duration_ms
              << " interval_ms=[" << interval_ms[0] << ", " << interval_ms[1]
              << ", " << interval_ms[2] << ", " << interval_ms[3] << ", "
              << interval_ms[4] << ", " << interval_ms[5] << ", "
              << interval_ms[6] << "]"
              << " serial_burst_ms=" << elapsed_ms << std::endl;
  }

  void query_next_feedback_joint()
  {
    feedback_angles_[feedback_joint_] = servos_[feedback_joint_].queryRawAngle();
    ++feedback_joint_;
    if (feedback_joint_ != kJointCount)
    {
      return;
    }
    feedback_joint_ = 0U;
    ++completed_feedback_cycles_;
    ServoFeedback message;
    message.servo0_data_() = feedback_angles_[0];
    message.servo1_data_() = feedback_angles_[1];
    message.servo2_data_() = feedback_angles_[2];
    message.servo3_data_() = feedback_angles_[3];
    message.servo4_data_() = feedback_angles_[4];
    message.servo5_data_() = feedback_angles_[5];
    message.servo6_data_() = feedback_angles_[6];
    feedback_publisher_->Write(message, 0);
  }

  FSUS_Protocol protocol_;
  std::array<FSUS_Servo, kJointCount> servos_;
  std::unique_ptr<unitree::robot::ChannelPublisher<ServoFeedback>>
    feedback_publisher_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<ArmString>>
    segment_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<DampingCommand>>
    damping_subscriber_;

  std::mutex state_mutex_;
  std::condition_variable state_changed_;
  std::optional<NativeSegment> latest_segment_;
  std::array<std::optional<DampingCommand>, kJointCount> damping_commands_{};
  std::array<float, kJointCount> feedback_angles_{};
  std::size_t feedback_joint_{0U};
  std::uint64_t completed_feedback_cycles_{0U};
  std::thread io_thread_;
  std::atomic<bool> stopping_{false};
  std::uint64_t dispatched_segments_{0U};
  std::uint64_t superseded_segments_{0U};
  std::uint64_t invalid_segments_{0U};
  std::uint64_t stale_segments_{0U};
  const std::uint16_t minimum_joint_interval_ms_;
};

}  // namespace

int main()
{
  ::signal(SIGINT, request_stop);
  ::signal(SIGTERM, request_stop);
  try
  {
    ChannelFactory::Instance()->Init(0);
    D1ControlNode controller;
    controller.start();
    while (!g_stop_requested)
    {
      std::this_thread::sleep_for(100ms);
    }
    controller.stop();
    ChannelFactory::Instance()->Release();
    return 0;
  }
  catch (const std::exception & error)
  {
    std::cerr << "D1 control node failed: " << error.what() << std::endl;
    ChannelFactory::Instance()->Release();
    return 1;
  }
}
