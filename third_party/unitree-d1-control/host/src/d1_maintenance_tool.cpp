#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"
#include "SetServoDumping_.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using namespace std::chrono_literals;
using ArmString = unitree_arm::msg::dds_::ArmString_;
using ServoFeedback = unitree_arm::msg::dds_::PubServoInfo_;
using DampingCommand = unitree_arm::msg::dds_::SetServoDumping_;

constexpr std::size_t kJointCount = 7U;
constexpr char kFeedbackTopic[] = "current_servo_angle";
constexpr char kNativeSegmentTopic[] = "d1_native_joint_segment";
constexpr char kDampingTopic[] = "set_servo_dumping";
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr double kNominalSpeedDegPerSecond = 15.0;
constexpr std::uint32_t kMinimumDurationMs = 1500U;
constexpr std::uint32_t kMaximumDurationMs = 30000U;
constexpr std::uint32_t kMaximumRampMs = 800U;
constexpr double kEndpointToleranceDeg = 2.0;

enum class Operation {unload, stowed, zero};

struct Config
{
  std::string interface;
  int domain_id{0};
  std::uint32_t duration_ms{0U};
  bool confirmed{false};
};

constexpr Operation compiled_operation()
{
#if defined(D1_MAINTENANCE_UNLOAD)
  return Operation::unload;
#elif defined(D1_MAINTENANCE_STOWED)
  return Operation::stowed;
#elif defined(D1_MAINTENANCE_ZERO)
  return Operation::zero;
#else
#error "A D1 maintenance operation compile definition is required"
#endif
}

const char * operation_name(const Operation operation)
{
  switch (operation)
  {
    case Operation::unload: return "all-joint unload";
    case Operation::stowed: return "STOWED move";
    case Operation::zero: return "zero move";
  }
  return "unknown";
}

const char * confirmation_token(const Operation operation)
{
  switch (operation)
  {
    case Operation::unload: return "UNLOAD_ALL";
    case Operation::stowed: return "STOWED_MOVE";
    case Operation::zero: return "ZERO_MOVE";
  }
  return "INVALID";
}

void print_usage(const char * program, const Operation operation)
{
  std::cout << "Usage: " << program
            << " --interface IFACE [--domain-id 0]";
  if (operation != Operation::unload) std::cout << " [--duration-ms N]";
  std::cout << " --confirm " << confirmation_token(operation) << '\n';
  if (operation == Operation::unload)
  {
    std::cout << "Legacy: " << program << " UNLOAD_ALL IFACE\n";
    std::cout << "WARNING: every joint immediately loses holding torque. "
                 "Support the arm before use.\n";
  }
  else
  {
    std::cout << "WARNING: this commands the physical D1. Clear its workspace "
                 "and do not run another command publisher.\n";
  }
}

int parse_integer(const std::string & value, const std::string & option)
{
  std::size_t consumed = 0U;
  const int parsed = std::stoi(value, &consumed);
  if (consumed != value.size())
  {
    throw std::invalid_argument("invalid integer for " + option + ": " + value);
  }
  return parsed;
}

Config parse_arguments(int argc, char ** argv, const Operation operation)
{
  Config config;
  // Preserve the established emergency-tool spelling:
  //   all_joints_unload UNLOAD_ALL enp3s0
  if (operation == Operation::unload && argc >= 2 && argc <= 3 &&
    std::string(argv[1]) == confirmation_token(operation))
  {
    if (argc == 2)
      throw std::invalid_argument("the legacy form also requires an interface");
    config.interface = argv[2];
    config.confirmed = true;
    return config;
  }
  for (int index = 1; index < argc; ++index)
  {
    const std::string option = argv[index];
    if (option == "-h" || option == "--help")
    {
      print_usage(argv[0], operation);
      std::exit(EXIT_SUCCESS);
    }
    if (index + 1 >= argc) throw std::invalid_argument("missing value for " + option);
    const std::string value = argv[++index];
    if (option == "--interface")
    {
      config.interface = value;
    }
    else if (option == "--domain-id")
    {
      config.domain_id = parse_integer(value, option);
      if (config.domain_id < 0 || config.domain_id > 232)
        throw std::invalid_argument("domain id must be in [0, 232]");
    }
    else if (option == "--duration-ms" && operation != Operation::unload)
    {
      const int parsed = parse_integer(value, option);
      if (parsed < static_cast<int>(kMinimumDurationMs) ||
        parsed > static_cast<int>(kMaximumDurationMs))
        throw std::invalid_argument("duration must be in [1500, 30000] ms");
      config.duration_ms = static_cast<std::uint32_t>(parsed);
    }
    else if (option == "--confirm")
    {
      config.confirmed = value == confirmation_token(operation);
    }
    else
    {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (config.interface.empty()) throw std::invalid_argument("--interface is required");
  if (!config.confirmed)
  {
    throw std::invalid_argument(
            std::string("physical operation requires --confirm ") +
            confirmation_token(operation));
  }
  return config;
}

std::array<double, kJointCount> feedback_to_array(const ServoFeedback & message)
{
  return {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(),
    message.servo3_data_(), message.servo4_data_(), message.servo5_data_(),
    message.servo6_data_()};
}

void print_angles(const char * label, const std::array<double, kJointCount> & angles)
{
  std::cout << label << " [";
  for (std::size_t joint = 0; joint < angles.size(); ++joint)
  {
    if (joint != 0U) std::cout << ", ";
    std::cout << std::fixed << std::setprecision(1) << angles[joint];
  }
  std::cout << "] deg\n";
}

class FeedbackMonitor
{
public:
  void receive(const void * raw)
  {
    const auto * message = static_cast<const ServoFeedback *>(raw);
    std::lock_guard<std::mutex> lock(mutex_);
    angles_ = feedback_to_array(*message);
    received_at_ = std::chrono::steady_clock::now();
    valid_ = std::all_of(angles_.begin(), angles_.end(), [](const double value)
      {return std::isfinite(value) && std::abs(value) <= 360.0;});
    changed_.notify_all();
  }

  bool wait_for_fresh(
    std::array<double, kJointCount> & output,
    const std::chrono::milliseconds timeout)
  {
    const auto started = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = changed_.wait_for(lock, timeout, [this, started]()
      {return valid_ && received_at_ >= started;});
    if (ready) output = angles_;
    return ready;
  }

  bool wait_until_target(
    const std::array<double, kJointCount> & target,
    const std::chrono::milliseconds timeout,
    std::array<double, kJointCount> & final)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(mutex_);
    while (std::chrono::steady_clock::now() < deadline)
    {
      changed_.wait_until(lock, deadline);
      if (!valid_) continue;
      final = angles_;
      double maximum_error = 0.0;
      for (std::size_t joint = 0; joint < kJointCount; ++joint)
        maximum_error = std::max(maximum_error, std::abs(target[joint] - angles_[joint]));
      if (maximum_error <= kEndpointToleranceDeg) return true;
    }
    if (valid_) final = angles_;
    return false;
  }

private:
  std::mutex mutex_;
  std::condition_variable changed_;
  std::array<double, kJointCount> angles_{};
  std::chrono::steady_clock::time_point received_at_{};
  bool valid_{false};
};

std::array<double, kJointCount> pose_target(const Operation operation)
{
  // Both maintenance poses deliberately leave the gripper fully open.
  if (operation == Operation::stowed)
  {
    return {0.0, -1.54 * kRadiansToDegrees, 1.55 * kRadiansToDegrees,
      0.0, 0.0, 0.0, 60.0};
  }
  return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0};
}

std::uint32_t automatic_duration(
  const std::array<double, kJointCount> & current,
  const std::array<double, kJointCount> & target)
{
  double maximum_delta = 0.0;
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
    maximum_delta = std::max(maximum_delta, std::abs(target[joint] - current[joint]));
  const auto requested = static_cast<std::uint32_t>(
    std::ceil(maximum_delta / kNominalSpeedDegPerSecond * 1000.0));
  return std::clamp(requested, kMinimumDurationMs, kMaximumDurationMs);
}

std::string segment_payload(
  const std::array<double, kJointCount> & target,
  const std::uint32_t duration_ms)
{
  const auto ramp_ms = std::min(
    kMaximumRampMs, static_cast<std::uint32_t>(std::lround(duration_ms * 0.15)));
  const auto sequence = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  std::ostringstream payload;
  payload.setf(std::ios::fixed);
  payload.precision(7);
  payload << "{\"seq\":" << sequence << ",\"duration_ms\":" << duration_ms
          << ",\"acceleration_ms\":" << ramp_ms
          << ",\"deceleration_ms\":" << ramp_ms << ",\"angles_deg\":[";
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    if (joint != 0U) payload << ',';
    payload << target[joint];
  }
  payload << "]}";
  return payload.str();
}

int run_unload()
{
  unitree::robot::ChannelPublisher<DampingCommand> publisher(kDampingTopic);
  publisher.InitChannel();
  std::this_thread::sleep_for(700ms);
  bool accepted = true;
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    DampingCommand command;
    command.seq_() = static_cast<std::int32_t>(joint + 1U);
    command.id_() = static_cast<std::uint8_t>(joint);
    command.power_() = 0U;
    accepted = publisher.Write(command) && accepted;
  }
  std::this_thread::sleep_for(300ms);
  publisher.CloseChannel();
  if (!accepted) throw std::runtime_error("DDS rejected at least one unload command");
  std::cout << "Sent damping power=0 to Joint0..6. Support the arm and verify "
               "that every joint is unloaded.\n";
  return EXIT_SUCCESS;
}

int run_pose(const Operation operation, const Config & config)
{
  FeedbackMonitor monitor;
  unitree::robot::ChannelSubscriber<ServoFeedback> subscriber(kFeedbackTopic);
  subscriber.InitChannel([&monitor](const void * raw) {monitor.receive(raw);}, 8);
  unitree::robot::ChannelPublisher<ArmString> publisher(kNativeSegmentTopic);
  publisher.InitChannel();

  std::array<double, kJointCount> current{};
  if (!monitor.wait_for_fresh(current, 3s))
    throw std::runtime_error(
            "no fresh current_servo_angle feedback; is the onboard executor running?");
  const auto final_target = pose_target(operation);
  print_angles("Current:", current);
  print_angles("Target: ", final_target);

  // Receiving feedback proves that this DDS participant can hear the board,
  // but it does not prove that the newly-created command writer has already
  // matched the board-side segment reader. Keep the writer alive for a short
  // discovery window so the first maintenance command is not lost.
  std::this_thread::sleep_for(700ms);

  const auto duration_ms = config.duration_ms == 0U ?
    automatic_duration(current, final_target) : config.duration_ms;
  std::cout << "Native common-arrival duration=" << duration_ms << " ms\n";
  ArmString command;
  command.data_() = segment_payload(final_target, duration_ms);
  if (!publisher.Write(command))
    throw std::runtime_error("DDS rejected native segment command");

  std::array<double, kJointCount> final = current;
  const bool reached = monitor.wait_until_target(
    final_target, std::chrono::milliseconds(duration_ms + 5000U), final);
  print_angles("Final:  ", final);
  publisher.CloseChannel();
  subscriber.CloseChannel();
  if (!reached)
    throw std::runtime_error("pose verification timed out (2 degree tolerance)");
  std::cout << operation_name(operation) << " reached within 2 degrees.\n";
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto operation = compiled_operation();
  try
  {
    const auto config = parse_arguments(argc, argv, operation);
    std::cout << "WARNING: physical D1 " << operation_name(operation)
              << " on interface '" << config.interface << "'.\n";
    unitree::robot::ChannelFactory::Instance()->Init(config.domain_id, config.interface);
    const int result = operation == Operation::unload ?
      run_unload() : run_pose(operation, config);
    unitree::robot::ChannelFactory::Instance()->Release();
    return result;
  }
  catch (const std::exception & error)
  {
    std::cerr << "ERROR: " << error.what() << "\n\n";
    print_usage(argv[0], operation);
    return EXIT_FAILURE;
  }
}
