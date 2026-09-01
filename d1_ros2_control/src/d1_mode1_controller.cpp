#include "d1_streaming_control/local_protocol.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "d1_ros2_control/action/execute_joint_segment.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace d1_ros2_control
{
using d1_streaming_control::JointPacket;
using d1_streaming_control::PacketKind;

namespace
{
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

double targetDirectedProgress(
  const std::array<double, 7> & initial,
  const std::array<double, 7> & current,
  const std::array<double, 6> & target)
{
  double target_norm_squared = 0.0;
  double projected_motion = 0.0;
  for (std::size_t joint = 0; joint < target.size(); ++joint) {
    const double target_delta = target[joint] - initial[joint];
    target_norm_squared += target_delta * target_delta;
    projected_motion += (current[joint] - initial[joint]) * target_delta;
  }
  if (target_norm_squared <= 1e-12) {
    return 0.0;
  }
  return projected_motion / std::sqrt(target_norm_squared);
}

template<typename T>
bool finiteVector(const std::vector<T> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const auto value) {return std::isfinite(value);});
}

template<std::size_t Size>
std::string degreesString(const std::array<double, Size> & positions)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << '[';
  for (std::size_t index = 0; index < Size; ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << positions[index] * kRadiansToDegrees;
  }
  stream << ']';
  return stream.str();
}

std::array<double, 6> armPositions(const std::array<double, 7> & positions)
{
  std::array<double, 6> arm{};
  std::copy_n(positions.begin(), arm.size(), arm.begin());
  return arm;
}
}  // namespace

class D1NativeSegmentController final : public rclcpp::Node
{
public:
  using Arm = control_msgs::action::FollowJointTrajectory;
  using ArmHandle = rclcpp_action::ServerGoalHandle<Arm>;
  using Segment = d1_ros2_control::action::ExecuteJointSegment;
  using SegmentHandle = rclcpp_action::ServerGoalHandle<Segment>;
  using Gripper = control_msgs::action::GripperCommand;
  using GripperHandle = rclcpp_action::ServerGoalHandle<Gripper>;

  D1NativeSegmentController()
  : Node("d1_native_segment_controller")
  {
    gateway_host_ = declare_parameter<std::string>("gateway_host", "127.0.0.1");
    command_port_ = declare_parameter<int>("command_port", 15000);
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 1.5);
    position_tolerance_rad_ = declare_parameter<double>(
      "arm_position_tolerance_rad", 0.034906585);
    stable_samples_ = declare_parameter<int>("arm_stable_samples", 3);
    timeout_padding_s_ = declare_parameter<double>("timeout_padding_s", 3.0);
    minimum_timeout_s_ = declare_parameter<double>("minimum_timeout_s", 5.0);
    command_start_timeout_s_ = declare_parameter<double>(
      "command_start_timeout_s", 2.0);
    arm_motion_start_progress_deg_ = declare_parameter<double>(
      "arm_motion_start_progress_deg", 1.0);
    no_motion_max_retries_ = declare_parameter<int>(
      "no_motion_max_retries", 4);
    gripper_closed_angle_deg_ = declare_parameter<double>(
      "gripper_closed_angle_deg", -30.0);
    gripper_open_angle_deg_ = declare_parameter<double>(
      "gripper_open_angle_deg", 60.0);
    gripper_travel_m_ = declare_parameter<double>("gripper_travel_m", 0.03);
    gripper_goal_tolerance_deg_ = declare_parameter<double>(
      "gripper_goal_tolerance_deg", 2.0);
    gripper_stable_range_deg_ = declare_parameter<double>(
      "gripper_stable_range_deg", 0.3);
    gripper_stable_duration_s_ = declare_parameter<double>(
      "gripper_stable_duration_s", 0.55);
    gripper_motion_start_deg_ = declare_parameter<double>(
      "gripper_motion_start_deg", 1.0);
    gripper_timeout_s_ = declare_parameter<double>("gripper_timeout_s", 10.0);
    native_joint_speed_deg_s_ = declare_parameter<double>(
      "native_joint_speed_deg_s", 15.0);
    gripper_joint_speed_deg_s_ = declare_parameter<double>(
      "gripper_joint_speed_deg_s", 30.0);
    native_acceleration_fraction_ = declare_parameter<double>(
      "native_acceleration_fraction", 0.15);
    native_profile_max_ramp_ms_ = declare_parameter<int>(
      "native_profile_max_ramp_ms", 800);
    native_minimum_duration_ms_ = declare_parameter<int>(
      "native_minimum_duration_ms", 1500);
    native_maximum_duration_ms_ = declare_parameter<int>(
      "native_maximum_duration_ms", 30000);
    lower_limits_ = vectorToArray(declare_parameter<std::vector<double>>(
      "lower_limits", {-6.3, -6.3, -6.3, -6.3, -6.3, -6.3, 0.0}),
      "lower_limits");
    upper_limits_ = vectorToArray(declare_parameter<std::vector<double>>(
      "upper_limits", {6.3, 6.3, 6.3, 6.3, 6.3, 6.3, 0.03}),
      "upper_limits");

    if (command_port_ <= 0 || command_port_ > 65535 || feedback_timeout_s_ <= 0.0 ||
      position_tolerance_rad_ <= 0.0 || stable_samples_ <= 0 ||
      timeout_padding_s_ < 0.0 ||
      minimum_timeout_s_ <= 0.0 || command_start_timeout_s_ <= 0.0 ||
      arm_motion_start_progress_deg_ <= 0.0 || no_motion_max_retries_ < 0 ||
      gripper_travel_m_ <= 0.0 ||
      gripper_open_angle_deg_ <= gripper_closed_angle_deg_ ||
      gripper_goal_tolerance_deg_ <= 0.0 || gripper_stable_range_deg_ <= 0.0 ||
      gripper_stable_duration_s_ <= 0.0 || gripper_motion_start_deg_ <= 0.0 ||
      gripper_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid D1 native-segment controller parameters");
    }
    if (native_joint_speed_deg_s_ <= 0.0 || gripper_joint_speed_deg_s_ <= 0.0 ||
      native_acceleration_fraction_ < 0.0 ||
      native_acceleration_fraction_ > 0.5 || native_profile_max_ramp_ms_ < 0 ||
      native_minimum_duration_ms_ <= 0 ||
      native_maximum_duration_ms_ < native_minimum_duration_ms_ ||
      native_maximum_duration_ms_ > 65535)
    {
      throw std::invalid_argument("invalid native segment profile parameters");
    }
    for (std::size_t index = 0; index < lower_limits_.size(); ++index) {
      if (!std::isfinite(lower_limits_[index]) || !std::isfinite(upper_limits_[index]) ||
        lower_limits_[index] >= upper_limits_[index])
      {
        throw std::invalid_argument("invalid joint limits");
      }
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(
              std::string("failed to create D1 command socket: ") + std::strerror(errno));
    }
    gateway_address_.sin_family = AF_INET;
    gateway_address_.sin_port = htons(static_cast<std::uint16_t>(command_port_));
    if (::inet_pton(AF_INET, gateway_host_.c_str(), &gateway_address_.sin_addr) != 1) {
      throw std::invalid_argument("gateway_host must be an IPv4 address");
    }

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        handleJointState(std::move(message));
      });
    arm_server_ = rclcpp_action::create_server<Arm>(
      this, "/arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Arm::Goal> goal) {
        return acceptArmGoal(*goal);
      },
      [](const std::shared_ptr<ArmHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ArmHandle> handle) {
        std::thread([this, handle]() {executeArm(handle);}).detach();
      });
    segment_server_ = rclcpp_action::create_server<Segment>(
      this, "/arm_controller/execute_joint_segment",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Segment::Goal> goal) {
        return acceptSegmentGoal(*goal);
      },
      [](const std::shared_ptr<SegmentHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<SegmentHandle> handle) {
        std::thread([this, handle]() {executeSegment(handle);}).detach();
      });
    gripper_server_ = rclcpp_action::create_server<Gripper>(
      this, "/gripper_controller/gripper_cmd",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Gripper::Goal> goal) {
        return acceptGripperGoal(*goal);
      },
      [](const std::shared_ptr<GripperHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GripperHandle> handle) {
        std::thread([this, handle]() {executeGripper(handle);}).detach();
      });

    RCLCPP_INFO(
      get_logger(),
      "D1 native-segment controller ready: default_profile=uniform_joint_speed "
      "default_speed=%.1f deg/s gripper_speed=%.1f deg/s "
      "explicit_action=/arm_controller/execute_joint_segment",
      native_joint_speed_deg_s_, gripper_joint_speed_deg_s_);
  }

  ~D1NativeSegmentController() override
  {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
    }
  }

private:
  enum class MotionProfile : std::uint8_t
  {
    uniform_joint_speed = Segment::Goal::UNIFORM_JOINT_SPEED,
    common_arrival = Segment::Goal::COMMON_ARRIVAL,
  };

  struct MotionSettings
  {
    MotionProfile profile{MotionProfile::uniform_joint_speed};
    double speed_deg_s{15.0};
  };

  struct CompletionSettings
  {
    bool require_settled_feedback{false};
    double maximum_deviation_rad{0.0};
    double maximum_stable_range_rad{0.0};
    std::size_t stable_samples{0U};
    double stable_sample_period_s{0.0};
  };

  static const char * profileName(const MotionProfile profile)
  {
    return profile == MotionProfile::common_arrival ?
      "common_arrival" : "uniform_joint_speed";
  }

  std::uint32_t segmentDurationMs(
    const double maximum_delta_deg, const MotionSettings & settings) const
  {
    const double required_duration_s = maximum_delta_deg / settings.speed_deg_s;
    const int minimum_duration_ms = settings.profile == MotionProfile::uniform_joint_speed ?
      1 : native_minimum_duration_ms_;
    return static_cast<std::uint32_t>(std::clamp<long long>(
      std::llround(1000.0 * required_duration_s),
      minimum_duration_ms, native_maximum_duration_ms_));
  }

  static std::array<double, 7> vectorToArray(
    const std::vector<double> & values, const char * name)
  {
    if (values.size() != 7 || !finiteVector(values)) {
      throw std::invalid_argument(std::string(name) + " must contain seven finite values");
    }
    std::array<double, 7> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  void handleJointState(sensor_msgs::msg::JointState::ConstSharedPtr message)
  {
    std::array<double, 7> positions{};
    std::array<bool, 7> found{};
    for (std::size_t source = 0;
      source < message->name.size() && source < message->position.size(); ++source)
    {
      for (std::size_t joint = 0; joint < found.size(); ++joint) {
        if (message->name[source] == "Joint" + std::to_string(joint)) {
          positions[joint] = message->position[source];
          found[joint] = std::isfinite(positions[joint]);
          break;
        }
      }
    }
    if (!std::all_of(found.begin(), found.end(), [](const bool value) {return value;})) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (feedback_sequence_ > 0U) {
      const double dt = std::chrono::duration<double>(now - latest_feedback_time_).count();
      if (dt > 1e-6) {
        latest_gripper_velocity_ = (positions[6] - latest_positions_[6]) / dt;
      }
    }
    latest_positions_ = positions;
    latest_feedback_time_ = now;
    ++feedback_sequence_;
    feedback_changed_.notify_all();
  }

  bool latestFeedback(
    std::array<double, 7> & positions, double & gripper_velocity,
    std::uint64_t * sequence = nullptr)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (feedback_sequence_ == 0U ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_feedback_time_).count() >
      feedback_timeout_s_)
    {
      return false;
    }
    positions = latest_positions_;
    gripper_velocity = latest_gripper_velocity_;
    if (sequence != nullptr) {
      *sequence = feedback_sequence_;
    }
    return true;
  }

  bool initializeDesiredFromFeedback()
  {
    std::array<double, 7> current{};
    double velocity = 0.0;
    if (!latestFeedback(current, velocity)) {
      return false;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!desired_valid_) {
      desired_ = current;
      desired_valid_ = true;
      RCLCPP_INFO(get_logger(), "Initialized seven-joint target from live feedback");
    }
    return true;
  }

  bool sendSnapshot(
    const std::array<double, 7> & target, const std::uint32_t duration_ms,
    const MotionProfile profile)
  {
    JointPacket packet;
    packet.kind = PacketKind::command;
    packet.sequence = sequence_.fetch_add(1U);
    packet.smoothing_mode = profile == MotionProfile::uniform_joint_speed ? 3U : 2U;
    packet.duration_ms = duration_ms;
    const auto ramp_ms = static_cast<std::uint32_t>(std::min(
      native_profile_max_ramp_ms_,
      static_cast<int>(std::lround(duration_ms * native_acceleration_fraction_))));
    packet.acceleration_ms = ramp_ms;
    packet.deceleration_ms = ramp_ms;
    for (std::size_t joint = 0; joint < 6; ++joint) {
      packet.angle_deg[joint] = target[joint] * kRadiansToDegrees;
    }
    const double ratio = std::clamp(target[6] / gripper_travel_m_, 0.0, 1.0);
    packet.angle_deg[6] = gripper_closed_angle_deg_ + ratio *
      (gripper_open_angle_deg_ - gripper_closed_angle_deg_);

    std::lock_guard<std::mutex> send_lock(send_mutex_);
    const bool sent = ::sendto(
      socket_fd_, &packet, sizeof(packet), 0,
      reinterpret_cast<const sockaddr *>(&gateway_address_), sizeof(gateway_address_)) ==
      static_cast<ssize_t>(sizeof(packet));
    RCLCPP_INFO(
      get_logger(),
      "Local D1 native segment %s: seq=%lu profile=%s duration=%u accel=%u decel=%u target_deg="
      "[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      sent ? "sent" : "failed", static_cast<unsigned long>(packet.sequence),
      profileName(profile),
      packet.duration_ms, packet.acceleration_ms, packet.deceleration_ms,
      packet.angle_deg[0], packet.angle_deg[1],
      packet.angle_deg[2], packet.angle_deg[3], packet.angle_deg[4],
      packet.angle_deg[5], packet.angle_deg[6]);
    return sent;
  }

  rclcpp_action::GoalResponse acceptArmGoal(const Arm::Goal & goal)
  {
    const auto & trajectory = goal.trajectory;
    if (trajectory.points.empty() || trajectory.joint_names.size() != 6 ||
      trajectory.points.back().positions.size() != trajectory.joint_names.size())
    {
      RCLCPP_ERROR(get_logger(), "Rejecting malformed arm trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::array<bool, 6> found{};
    for (const auto & name : trajectory.joint_names) {
      for (std::size_t joint = 0; joint < found.size(); ++joint) {
        if (name == "Joint" + std::to_string(joint)) {
          if (found[joint]) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          found[joint] = true;
        }
      }
    }
    if (!std::all_of(found.begin(), found.end(), [](const bool value) {return value;})) {
      RCLCPP_ERROR(get_logger(), "Rejecting arm trajectory with unexpected joints");
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting arm goal: physical command owner is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse acceptGripperGoal(const Gripper::Goal & goal)
  {
    const double target = goal.command.position;
    if (!std::isfinite(target) || target < lower_limits_[6] || target > upper_limits_[6]) {
      RCLCPP_ERROR(get_logger(), "Rejecting out-of-range gripper target %.6f", target);
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting gripper goal: physical command owner is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse acceptSegmentGoal(const Segment::Goal & goal)
  {
    const bool invalid_settled_policy = goal.require_settled_feedback &&
      (!std::isfinite(goal.maximum_deviation_rad) ||
      !std::isfinite(goal.maximum_stable_range_rad) ||
      !std::isfinite(goal.stable_sample_period_s) ||
      goal.maximum_deviation_rad <= 0.0 ||
      goal.maximum_stable_range_rad <= 0.0 ||
      goal.stable_samples < 2U || goal.stable_sample_period_s <= 0.0);
    if (goal.joint_names.size() != 6 || goal.positions.size() != 6 ||
      !finiteVector(goal.positions) || !std::isfinite(goal.speed_deg_s) ||
      goal.speed_deg_s <= 0.0 ||
      invalid_settled_policy ||
      (goal.motion_profile != Segment::Goal::UNIFORM_JOINT_SPEED &&
      goal.motion_profile != Segment::Goal::COMMON_ARRIVAL))
    {
      RCLCPP_ERROR(get_logger(), "Rejecting malformed explicit joint segment");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::array<bool, 6> found{};
    for (const auto & name : goal.joint_names) {
      for (std::size_t joint = 0; joint < found.size(); ++joint) {
        if (name == "Joint" + std::to_string(joint)) {
          if (found[joint]) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          found[joint] = true;
        }
      }
    }
    if (!std::all_of(found.begin(), found.end(), [](const bool value) {return value;})) {
      RCLCPP_ERROR(get_logger(), "Rejecting explicit segment with unexpected joints");
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting explicit segment: physical command owner is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::array<double, 6> armTarget(const Arm::Goal & goal) const
  {
    std::array<double, 6> target{};
    const auto & names = goal.trajectory.joint_names;
    const auto & positions = goal.trajectory.points.back().positions;
    for (std::size_t source = 0; source < names.size(); ++source) {
      for (std::size_t joint = 0; joint < target.size(); ++joint) {
        if (names[source] == "Joint" + std::to_string(joint)) {
          target[joint] = positions[source];
          break;
        }
      }
    }
    return target;
  }

  std::array<double, 6> segmentTarget(const Segment::Goal & goal) const
  {
    std::array<double, 6> target{};
    for (std::size_t source = 0; source < goal.joint_names.size(); ++source) {
      for (std::size_t joint = 0; joint < target.size(); ++joint) {
        if (goal.joint_names[source] == "Joint" + std::to_string(joint)) {
          target[joint] = goal.positions[source];
          break;
        }
      }
    }
    return target;
  }

  enum class SegmentStatus {success, canceled, invalid, failed};

  struct SegmentOutcome
  {
    SegmentStatus status{SegmentStatus::failed};
    std::string detail;
    double maximum_error_rad{0.0};
  };

  template<typename CancelFunction, typename FeedbackFunction>
  SegmentOutcome runArmSegment(
    const std::array<double, 6> & target, const MotionSettings & settings,
    const double planned_duration_s, const CompletionSettings & completion,
    CancelFunction canceled, FeedbackFunction feedback)
  {
    try {
      if (!initializeDesiredFromFeedback()) {
        return {SegmentStatus::failed, "fresh D1 feedback unavailable", 0.0};
      }
      std::array<double, 7> initial{};
      double gripper_velocity = 0.0;
      if (!latestFeedback(initial, gripper_velocity)) {
        return {SegmentStatus::failed, "fresh D1 feedback unavailable", 0.0};
      }
      double maximum_delta_deg = 0.0;
      for (std::size_t joint = 0; joint < target.size(); ++joint) {
        if (!std::isfinite(target[joint]) || target[joint] < lower_limits_[joint] ||
          target[joint] > upper_limits_[joint])
        {
          return {
            SegmentStatus::invalid, "arm endpoint violates configured joint limits", 0.0};
        }
        maximum_delta_deg = std::max(
          maximum_delta_deg, std::abs(target[joint] - initial[joint]) * kRadiansToDegrees);
      }

      std::array<double, 7> snapshot{};
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        snapshot = desired_;
        std::copy(target.begin(), target.end(), snapshot.begin());
      }
      RCLCPP_INFO(
        get_logger(), "Arm command initial_deg=%s target_deg=%s preserved_joint6=%.2f deg",
        degreesString(armPositions(initial)).c_str(), degreesString(target).c_str(),
        gripper_closed_angle_deg_ + snapshot[6] / gripper_travel_m_ *
        (gripper_open_angle_deg_ - gripper_closed_angle_deg_));
      const double required_duration_s = maximum_delta_deg / settings.speed_deg_s;
      const auto duration_ms = segmentDurationMs(maximum_delta_deg, settings);
      if (!sendSnapshot(snapshot, duration_ms, settings.profile)) {
        return {SegmentStatus::failed, "failed to send D1 native arm segment", 0.0};
      }
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        desired_ = snapshot;
      }
      const double timeout_s = std::max(
        minimum_timeout_s_, duration_ms / 1000.0 + timeout_padding_s_);
      RCLCPP_INFO(
        get_logger(),
        "Arm native segment sent once: max_delta=%.1f deg planned=%.3f s "
        "requested_speed=%.1f deg/s native_required=%.3f s segment_duration=%.3f s "
        "timeout=%.1f s",
        maximum_delta_deg, planned_duration_s, settings.speed_deg_s,
        required_duration_s, duration_ms / 1000.0, timeout_s);

      const auto started = std::chrono::steady_clock::now();
      const auto native_completion_time = started +
        std::chrono::milliseconds(duration_ms);
      const auto deadline = started + std::chrono::duration<double>(
        timeout_s + (completion.require_settled_feedback ? 0.0 :
        command_start_timeout_s_ * no_motion_max_retries_));
      auto next_start_deadline = started +
        std::chrono::duration<double>(command_start_timeout_s_);
      int no_motion_retries = 0;
      int stable = 0;
      std::uint64_t observed = 0U;
      std::deque<std::array<double, 6>> settled_window;
      auto last_settled_sample_time = started -
        std::chrono::duration<double>(completion.stable_sample_period_s);
      std::array<double, 6> latest_stable_range{};
      bool motion_started = false;
      double directed_progress_rad = 0.0;
      std::array<double, 7> last = initial;
      while (std::chrono::steady_clock::now() < deadline) {
        if (canceled()) {
          return {
            SegmentStatus::canceled, "canceled; no replacement command sent", 0.0};
        }
        std::array<double, 7> current{};
        double velocity = 0.0;
        if (!latestFeedback(current, velocity)) {
          return {SegmentStatus::failed, "D1 feedback became stale", 0.0};
        }
        last = current;
        double maximum_error_rad = 0.0;
        for (std::size_t joint = 0; joint < target.size(); ++joint) {
          maximum_error_rad = std::max(
            maximum_error_rad, std::abs(current[joint] - target[joint]));
        }
        feedback(maximum_error_rad);
        if (!motion_started) {
          directed_progress_rad = targetDirectedProgress(initial, current, target);
          motion_started = directed_progress_rad * kRadiansToDegrees >=
            arm_motion_start_progress_deg_;
          if (motion_started) {
            RCLCPP_INFO(
              get_logger(),
              "Arm feedback made %.2f deg target-directed progress after %.3f s: "
              "current_deg=%s",
              directed_progress_rad * kRadiansToDegrees,
              std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count(),
              degreesString(armPositions(current)).c_str());
          }
        }
        const auto now = std::chrono::steady_clock::now();
        if (completion.require_settled_feedback) {
          const double since_last_sample = std::chrono::duration<double>(
            now - last_settled_sample_time).count();
          if (now >= native_completion_time &&
            since_last_sample + 1e-9 >= completion.stable_sample_period_s)
          {
            std::array<double, 6> sample{};
            std::copy_n(current.begin(), sample.size(), sample.begin());
            settled_window.push_back(sample);
            last_settled_sample_time = now;
            while (settled_window.size() > completion.stable_samples) {
              settled_window.pop_front();
            }
            if (settled_window.size() == completion.stable_samples) {
              bool stopped = true;
              latest_stable_range.fill(0.0);
              for (std::size_t joint = 0; joint < target.size(); ++joint) {
                double minimum = settled_window.front()[joint];
                double maximum = minimum;
                for (const auto & value : settled_window) {
                  minimum = std::min(minimum, value[joint]);
                  maximum = std::max(maximum, value[joint]);
                }
                latest_stable_range[joint] = maximum - minimum;
                stopped = stopped && latest_stable_range[joint] <=
                  completion.maximum_stable_range_rad;
              }
              if (stopped) {
                const auto error = [&target, &current]() {
                    std::array<double, 6> value{};
                    for (std::size_t joint = 0; joint < value.size(); ++joint) {
                      value[joint] = target[joint] - current[joint];
                    }
                    return value;
                  }();
                const double elapsed = std::chrono::duration<double>(now - started).count();
                if (maximum_error_rad > completion.maximum_deviation_rad) {
                  std::ostringstream detail;
                  detail << std::fixed << std::setprecision(2)
                         << "visual-correction segment settled outside the "
                         << "plan-deviation guard: max_error="
                         << maximum_error_rad * kRadiansToDegrees
                         << " deg limit="
                         << completion.maximum_deviation_rad * kRadiansToDegrees
                         << " deg error_deg=" << degreesString(error)
                         << " stable_range_deg=" << degreesString(latest_stable_range);
                  RCLCPP_ERROR(
                    get_logger(),
                    "Visual-correction segment stopped too far from plan after %.3f s: "
                    "error_deg=%s stable_range_deg=%s max_error=%.2f deg limit=%.2f deg",
                    elapsed, degreesString(error).c_str(),
                    degreesString(latest_stable_range).c_str(),
                    maximum_error_rad * kRadiansToDegrees,
                    completion.maximum_deviation_rad * kRadiansToDegrees);
                  return {
                    SegmentStatus::failed,
                    detail.str(),
                    maximum_error_rad};
                }
                RCLCPP_INFO(
                  get_logger(),
                  "Visual-correction segment settled after native duration: elapsed=%.3f s "
                  "native=%.3f s final_deg=%s error_deg=%s stable_range_deg=%s",
                  elapsed, duration_ms / 1000.0,
                  degreesString(armPositions(current)).c_str(),
                  degreesString(error).c_str(),
                  degreesString(latest_stable_range).c_str());
                return {
                  SegmentStatus::success,
                  "D1 visual-correction segment settled within the plan-deviation guard",
                  maximum_error_rad};
              }
            }
          }
        } else {
          bool reached = true;
          for (std::size_t joint = 0; joint < target.size(); ++joint) {
            reached = reached &&
              std::abs(current[joint] - target[joint]) <= position_tolerance_rad_;
          }
          stable = reached ? stable + 1 : 0;
          if (stable >= stable_samples_) {
            RCLCPP_INFO(
              get_logger(), "Arm endpoint reached after %.3f s: final_deg=%s",
              std::chrono::duration<double>(now - started).count(),
              degreesString(armPositions(current)).c_str());
            return {
              SegmentStatus::success, "D1 native segment endpoint reached", maximum_error_rad};
          }
        }
        if (!completion.require_settled_feedback &&
          !motion_started && now >= next_start_deadline)
        {
          if (no_motion_retries >= no_motion_max_retries_) {
            RCLCPP_ERROR(
              get_logger(),
              "Arm did not start after %d send attempts over %.3f s; final_deg=%s",
              no_motion_retries + 1,
              std::chrono::duration<double>(now - started).count(),
              degreesString(armPositions(current)).c_str());
            return {
              SegmentStatus::failed, "D1 arm target produced no observed motion",
              maximum_error_rad};
          }
          ++no_motion_retries;
          RCLCPP_WARN(
            get_logger(),
            "Arm has only %.2f deg target-directed progress %.3f s after send; "
            "retrying absolute target (%d/%d)",
            directed_progress_rad * kRadiansToDegrees, command_start_timeout_s_,
            no_motion_retries, no_motion_max_retries_);
          if (!sendSnapshot(snapshot, duration_ms, settings.profile)) {
            return {
              SegmentStatus::failed, "failed to retry D1 native arm segment",
              maximum_error_rad};
          }
          next_start_deadline = now +
            std::chrono::duration<double>(command_start_timeout_s_);
        }
        std::unique_lock<std::mutex> lock(feedback_mutex_);
        observed = feedback_sequence_;
        feedback_changed_.wait_for(
          lock, 100ms, [this, observed]() {return feedback_sequence_ != observed;});
      }
      std::array<double, 6> final_error{};
      for (std::size_t joint = 0; joint < final_error.size(); ++joint) {
        final_error[joint] = target[joint] - last[joint];
      }
      RCLCPP_ERROR(
        get_logger(),
        "Arm endpoint timeout: motion_started=%s final_deg=%s error_deg=%s "
        "stable_range_deg=%s settled_policy=%s",
        motion_started ? "true" : "false", degreesString(armPositions(last)).c_str(),
        degreesString(final_error).c_str(), degreesString(latest_stable_range).c_str(),
        completion.require_settled_feedback ? "true" : "false");
      double maximum_error_rad = 0.0;
      for (std::size_t joint = 0; joint < final_error.size(); ++joint) {
        maximum_error_rad = std::max(maximum_error_rad, std::abs(final_error[joint]));
      }
      return {SegmentStatus::failed, "D1 native arm segment timed out", maximum_error_rad};
    } catch (const std::exception & error) {
      return {SegmentStatus::invalid, error.what(), 0.0};
    }
  }

  void executeArm(const std::shared_ptr<ArmHandle> & handle)
  {
    const auto finish = [this]() {busy_.store(false);};
    const auto planned_duration = handle->get_goal()->trajectory.points.back().time_from_start;
    const double planned_duration_s = static_cast<double>(planned_duration.sec) +
      static_cast<double>(planned_duration.nanosec) * 1e-9;
    const auto outcome = runArmSegment(
      armTarget(*handle->get_goal()),
      MotionSettings{MotionProfile::uniform_joint_speed, native_joint_speed_deg_s_},
      planned_duration_s, CompletionSettings{},
      [handle]() {return handle->is_canceling();}, [](double) {});
    auto result = std::make_shared<Arm::Result>();
    result->error_string = outcome.detail;
    if (outcome.status == SegmentStatus::success) {
      result->error_code = Arm::Result::SUCCESSFUL;
      handle->succeed(result);
    } else if (outcome.status == SegmentStatus::canceled) {
      result->error_code = Arm::Result::SUCCESSFUL;
      handle->canceled(result);
    } else {
      result->error_code = outcome.status == SegmentStatus::invalid ?
        Arm::Result::INVALID_GOAL : Arm::Result::GOAL_TOLERANCE_VIOLATED;
      handle->abort(result);
    }
    finish();
  }

  void executeSegment(const std::shared_ptr<SegmentHandle> & handle)
  {
    const auto finish = [this]() {busy_.store(false);};
    const auto goal = handle->get_goal();
    const MotionSettings settings{
      goal->motion_profile == Segment::Goal::COMMON_ARRIVAL ?
      MotionProfile::common_arrival : MotionProfile::uniform_joint_speed,
      goal->speed_deg_s};
    const CompletionSettings completion{
      goal->require_settled_feedback,
      goal->maximum_deviation_rad,
      goal->maximum_stable_range_rad,
      static_cast<std::size_t>(goal->stable_samples),
      goal->stable_sample_period_s};
    const auto outcome = runArmSegment(
      segmentTarget(*goal), settings, 0.0, completion,
      [handle]() {return handle->is_canceling();},
      [handle](const double error) {
        auto value = std::make_shared<Segment::Feedback>();
        value->maximum_error_rad = error;
        handle->publish_feedback(value);
      });
    auto result = std::make_shared<Segment::Result>();
    result->success = outcome.status == SegmentStatus::success;
    result->detail = outcome.detail;
    result->maximum_error_rad = outcome.maximum_error_rad;
    if (outcome.status == SegmentStatus::success) {
      handle->succeed(result);
    } else if (outcome.status == SegmentStatus::canceled) {
      handle->canceled(result);
    } else {
      handle->abort(result);
    }
    finish();
  }

  void executeGripper(const std::shared_ptr<GripperHandle> & handle)
  {
    const auto finish = [this]() {busy_.store(false);};
    auto result = std::make_shared<Gripper::Result>();
    if (!initializeDesiredFromFeedback()) {
      handle->abort(result); finish(); return;
    }
    const double target = handle->get_goal()->command.position;
    std::array<double, 7> initial{};
    double ignored_velocity = 0.0;
    if (!latestFeedback(initial, ignored_velocity)) {
      handle->abort(result); finish(); return;
    }
    // A gripper command must still be transmitted as a complete seven-joint D1
    // target. Preserve the live Joint0..5 positions rather than the historical
    // desired_ targets so closing the gripper cannot make the arm chase an old
    // endpoint that it never reached.
    std::array<double, 7> snapshot = initial;
    snapshot[6] = target;
    const double angle_span_deg = gripper_open_angle_deg_ - gripper_closed_angle_deg_;
    const double maximum_delta_deg =
      std::abs(target - initial[6]) / gripper_travel_m_ * angle_span_deg;
    const MotionSettings settings{
      MotionProfile::uniform_joint_speed, gripper_joint_speed_deg_s_};
    const auto duration_ms = segmentDurationMs(maximum_delta_deg, settings);
    if (!sendSnapshot(snapshot, duration_ms, settings.profile))
    {
      handle->abort(result); finish(); return;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      desired_ = snapshot;
    }
    RCLCPP_INFO(
      get_logger(),
      "Gripper target sent as full native segment: target=%.1f deg "
      "delta=%.1f deg speed=%.1f deg/s duration=%.3f s; Joint0..5 preserve live feedback",
      gripper_closed_angle_deg_ + target / gripper_travel_m_ *
      angle_span_deg, maximum_delta_deg, settings.speed_deg_s, duration_ms / 1000.0);

    const double goal_tolerance = gripper_goal_tolerance_deg_ / angle_span_deg *
      gripper_travel_m_;
    const double stable_range = gripper_stable_range_deg_ / angle_span_deg *
      gripper_travel_m_;
    const double motion_start = gripper_motion_start_deg_ / angle_span_deg *
      gripper_travel_m_;
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::duration<double>(
      gripper_timeout_s_ + command_start_timeout_s_ * no_motion_max_retries_);
    auto next_start_deadline = started +
      std::chrono::duration<double>(command_start_timeout_s_);
    int no_motion_retries = 0;
    using TimedPosition = std::pair<std::chrono::steady_clock::time_point, double>;
    std::deque<TimedPosition> stable_window;
    bool motion_started = false;
    while (std::chrono::steady_clock::now() < deadline) {
      std::array<double, 7> current{};
      double velocity = 0.0;
      if (!latestFeedback(current, velocity)) {
        handle->abort(result); finish(); return;
      }
      result->position = current[6];
      result->effort = 0.0;
      auto feedback = std::make_shared<Gripper::Feedback>();
      feedback->position = current[6];
      feedback->effort = 0.0;
      feedback->stalled = false;
      feedback->reached_goal = false;
      handle->publish_feedback(feedback);

      if (handle->is_canceling()) {
        handle->canceled(result); finish(); return;
      }
      if (std::abs(current[6] - target) <= goal_tolerance) {
        result->reached_goal = true;
        result->stalled = false;
        handle->succeed(result); finish(); return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (!motion_started && std::abs(current[6] - initial[6]) >= motion_start) {
        motion_started = true;
        stable_window.clear();
        RCLCPP_INFO(
          get_logger(), "Gripper feedback first moved after %.3f s",
          std::chrono::duration<double>(now - started).count());
      }
      if (motion_started) {
        stable_window.emplace_back(now, current[6]);
        const double window_age = std::chrono::duration<double>(
          now - stable_window.front().first).count();
        if (window_age >= gripper_stable_duration_s_) {
          const auto bounds = std::minmax_element(
            stable_window.begin(), stable_window.end(),
            [](const TimedPosition & left, const TimedPosition & right) {
              return left.second < right.second;
            });
          const double range = bounds.second->second - bounds.first->second;
          if (range <= stable_range) {
            result->reached_goal = false;
            result->stalled = true;
            RCLCPP_INFO(
              get_logger(),
              "Gripper stopped after %.3f s: position=%.2f deg "
              "window=%.3f s range=%.2f deg",
              std::chrono::duration<double>(now - started).count(),
              gripper_closed_angle_deg_ + current[6] / gripper_travel_m_ * angle_span_deg,
              window_age, range / gripper_travel_m_ * angle_span_deg);
            handle->succeed(result); finish(); return;
          }
        }
        while (stable_window.size() > 1U && std::chrono::duration<double>(
            now - stable_window[1].first).count() >= gripper_stable_duration_s_)
        {
          stable_window.pop_front();
        }
      }
      if (!motion_started && now >= next_start_deadline) {
        if (no_motion_retries >= no_motion_max_retries_) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper did not start after %d send attempts over %.3f s; position=%.2f deg",
            no_motion_retries + 1,
            std::chrono::duration<double>(now - started).count(),
            gripper_closed_angle_deg_ + current[6] / gripper_travel_m_ * angle_span_deg);
          handle->abort(result); finish(); return;
        }
        ++no_motion_retries;
        RCLCPP_WARN(
          get_logger(),
          "Gripper has not started %.3f s after send; retrying absolute target (%d/%d)",
          command_start_timeout_s_, no_motion_retries, no_motion_max_retries_);
        if (!sendSnapshot(snapshot, duration_ms, settings.profile))
        {
          handle->abort(result); finish(); return;
        }
        next_start_deadline = now +
          std::chrono::duration<double>(command_start_timeout_s_);
      }
      std::unique_lock<std::mutex> lock(feedback_mutex_);
      const auto observed = feedback_sequence_;
      feedback_changed_.wait_for(
        lock, 100ms, [this, observed]() {return feedback_sequence_ != observed;});
    }
    handle->abort(result); finish();
  }

  std::string gateway_host_;
  int command_port_{15000};
  double feedback_timeout_s_{1.5};
  double position_tolerance_rad_{0.034906585};
  int stable_samples_{3};
  double timeout_padding_s_{3.0};
  double minimum_timeout_s_{5.0};
  double command_start_timeout_s_{2.0};
  double arm_motion_start_progress_deg_{1.0};
  int no_motion_max_retries_{4};
  double gripper_closed_angle_deg_{-30.0};
  double gripper_open_angle_deg_{60.0};
  double gripper_travel_m_{0.03};
  double gripper_goal_tolerance_deg_{2.0};
  double gripper_stable_range_deg_{0.3};
  double gripper_stable_duration_s_{0.55};
  double gripper_motion_start_deg_{1.0};
  double gripper_timeout_s_{10.0};
  double native_joint_speed_deg_s_{15.0};
  double gripper_joint_speed_deg_s_{30.0};
  double native_acceleration_fraction_{0.15};
  int native_profile_max_ramp_ms_{800};
  int native_minimum_duration_ms_{1500};
  int native_maximum_duration_ms_{30000};
  std::array<double, 7> lower_limits_{};
  std::array<double, 7> upper_limits_{};

  int socket_fd_{-1};
  sockaddr_in gateway_address_{};
  std::atomic<std::uint64_t> sequence_{1U};
  std::mutex send_mutex_;

  std::mutex feedback_mutex_;
  std::condition_variable feedback_changed_;
  std::array<double, 7> latest_positions_{};
  double latest_gripper_velocity_{0.0};
  std::chrono::steady_clock::time_point latest_feedback_time_{};
  std::uint64_t feedback_sequence_{0U};

  std::mutex command_mutex_;
  std::array<double, 7> desired_{};
  bool desired_valid_{false};
  std::atomic<bool> busy_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp_action::Server<Arm>::SharedPtr arm_server_;
  rclcpp_action::Server<Segment>::SharedPtr segment_server_;
  rclcpp_action::Server<Gripper>::SharedPtr gripper_server_;
};
}  // namespace d1_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<d1_ros2_control::D1NativeSegmentController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
