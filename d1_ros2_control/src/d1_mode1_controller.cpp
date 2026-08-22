#include "d1_ros2_control/local_protocol.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

class D1Mode1Controller final : public rclcpp::Node
{
public:
  using Arm = control_msgs::action::FollowJointTrajectory;
  using ArmHandle = rclcpp_action::ServerGoalHandle<Arm>;
  using Gripper = control_msgs::action::GripperCommand;
  using GripperHandle = rclcpp_action::ServerGoalHandle<Gripper>;

  D1Mode1Controller()
  : Node("d1_mode1_controller")
  {
    gateway_host_ = declare_parameter<std::string>("gateway_host", "127.0.0.1");
    command_port_ = declare_parameter<int>("command_port", 15000);
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 1.5);
    position_tolerance_rad_ = declare_parameter<double>(
      "arm_position_tolerance_rad", 0.034906585);
    stable_samples_ = declare_parameter<int>("arm_stable_samples", 3);
    assumed_speed_deg_s_ = declare_parameter<double>("assumed_speed_deg_s", 10.0);
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
    lower_limits_ = vectorToArray(declare_parameter<std::vector<double>>(
      "lower_limits", {-6.3, -6.3, -6.3, -6.3, -6.3, -6.3, 0.0}),
      "lower_limits");
    upper_limits_ = vectorToArray(declare_parameter<std::vector<double>>(
      "upper_limits", {6.3, 6.3, 6.3, 6.3, 6.3, 6.3, 0.03}),
      "upper_limits");

    if (command_port_ <= 0 || command_port_ > 65535 || feedback_timeout_s_ <= 0.0 ||
      position_tolerance_rad_ <= 0.0 || stable_samples_ <= 0 ||
      assumed_speed_deg_s_ <= 0.0 || timeout_padding_s_ < 0.0 ||
      minimum_timeout_s_ <= 0.0 || command_start_timeout_s_ <= 0.0 ||
      arm_motion_start_progress_deg_ <= 0.0 || no_motion_max_retries_ < 0 ||
      gripper_travel_m_ <= 0.0 ||
      gripper_open_angle_deg_ <= gripper_closed_angle_deg_ ||
      gripper_goal_tolerance_deg_ <= 0.0 || gripper_stable_range_deg_ <= 0.0 ||
      gripper_stable_duration_s_ <= 0.0 || gripper_motion_start_deg_ <= 0.0 ||
      gripper_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid D1 mode=1 controller parameters");
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
      "D1 real mode=1 controller ready: one owner for arm and gripper commands");
  }

  ~D1Mode1Controller() override
  {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
    }
  }

private:
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

  bool sendSnapshot(const std::array<double, 7> & target)
  {
    JointPacket packet;
    packet.kind = PacketKind::command;
    packet.sequence = sequence_.fetch_add(1U);
    packet.smoothing_mode = 1U;
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
      "Local D1 command %s: seq=%lu mode=%u target_deg="
      "[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      sent ? "sent" : "failed", static_cast<unsigned long>(packet.sequence),
      packet.smoothing_mode, packet.angle_deg[0], packet.angle_deg[1],
      packet.angle_deg[2], packet.angle_deg[3], packet.angle_deg[4],
      packet.angle_deg[5], packet.angle_deg[6]);
    return sent;
  }

  bool holdArmAtMeasuredPosition()
  {
    std::array<double, 7> current{};
    double velocity = 0.0;
    if (!latestFeedback(current, velocity)) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!desired_valid_) {
        desired_ = current;
        desired_valid_ = true;
      } else {
        std::copy_n(current.begin(), 6, desired_.begin());
      }
      current = desired_;
    }
    return sendSnapshot(current);
  }

  bool holdGripperAtMeasuredPosition()
  {
    std::array<double, 7> current{};
    double velocity = 0.0;
    if (!latestFeedback(current, velocity)) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!desired_valid_) {
        desired_ = current;
        desired_valid_ = true;
      } else {
        desired_[6] = current[6];
      }
      current = desired_;
    }
    return sendSnapshot(current);
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

  void executeArm(const std::shared_ptr<ArmHandle> & handle)
  {
    const auto finish = [this]() {busy_.store(false);};
    auto result = std::make_shared<Arm::Result>();
    try {
      if (!initializeDesiredFromFeedback()) {
        result->error_code = Arm::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "fresh D1 feedback unavailable";
        handle->abort(result); finish(); return;
      }
      const auto target = armTarget(*handle->get_goal());
      std::array<double, 7> initial{};
      double gripper_velocity = 0.0;
      if (!latestFeedback(initial, gripper_velocity)) {
        result->error_code = Arm::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "fresh D1 feedback unavailable";
        handle->abort(result); finish(); return;
      }
      double maximum_delta_deg = 0.0;
      for (std::size_t joint = 0; joint < target.size(); ++joint) {
        if (!std::isfinite(target[joint]) || target[joint] < lower_limits_[joint] ||
          target[joint] > upper_limits_[joint])
        {
          result->error_code = Arm::Result::INVALID_GOAL;
          result->error_string = "arm endpoint violates configured joint limits";
          handle->abort(result); finish(); return;
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
      if (!sendSnapshot(snapshot)) {
        result->error_code = Arm::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "failed to send D1 mode=1 arm target";
        handle->abort(result); finish(); return;
      }
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        desired_ = snapshot;
      }
      const double timeout_s = std::max(
        minimum_timeout_s_, maximum_delta_deg / assumed_speed_deg_s_ + timeout_padding_s_);
      RCLCPP_INFO(
        get_logger(), "Arm endpoint sent once with mode=1: max_delta=%.1f deg timeout=%.1f s",
        maximum_delta_deg, timeout_s);

      const auto started = std::chrono::steady_clock::now();
      const auto deadline = started + std::chrono::duration<double>(
        timeout_s + command_start_timeout_s_ * no_motion_max_retries_);
      auto next_start_deadline = started +
        std::chrono::duration<double>(command_start_timeout_s_);
      int no_motion_retries = 0;
      int stable = 0;
      std::uint64_t observed = 0U;
      bool motion_started = false;
      double directed_progress_rad = 0.0;
      std::array<double, 7> last = initial;
      while (std::chrono::steady_clock::now() < deadline) {
        if (handle->is_canceling()) {
          holdArmAtMeasuredPosition();
          result->error_code = Arm::Result::SUCCESSFUL;
          result->error_string = "canceled; measured position held";
          handle->canceled(result); finish(); return;
        }
        std::array<double, 7> current{};
        double velocity = 0.0;
        if (!latestFeedback(current, velocity)) {
          result->error_code = Arm::Result::PATH_TOLERANCE_VIOLATED;
          result->error_string = "D1 feedback became stale";
          handle->abort(result); finish(); return;
        }
        last = current;
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
        bool reached = true;
        for (std::size_t joint = 0; joint < target.size(); ++joint) {
          reached = reached &&
            std::abs(current[joint] - target[joint]) <= position_tolerance_rad_;
        }
        stable = reached ? stable + 1 : 0;
        if (stable >= stable_samples_) {
          RCLCPP_INFO(
            get_logger(), "Arm endpoint reached after %.3f s: final_deg=%s",
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - started).count(),
            degreesString(armPositions(current)).c_str());
          result->error_code = Arm::Result::SUCCESSFUL;
          result->error_string = "D1 mode=1 endpoint reached";
          handle->succeed(result); finish(); return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!motion_started && now >= next_start_deadline) {
          if (no_motion_retries >= no_motion_max_retries_) {
            RCLCPP_ERROR(
              get_logger(),
              "Arm did not start after %d send attempts over %.3f s; final_deg=%s",
              no_motion_retries + 1,
              std::chrono::duration<double>(now - started).count(),
              degreesString(armPositions(current)).c_str());
            result->error_code = Arm::Result::GOAL_TOLERANCE_VIOLATED;
            result->error_string = "D1 arm target produced no observed motion";
            handle->abort(result); finish(); return;
          }
          ++no_motion_retries;
          RCLCPP_WARN(
            get_logger(),
            "Arm has only %.2f deg target-directed progress %.3f s after send; "
            "retrying absolute target (%d/%d)",
            directed_progress_rad * kRadiansToDegrees, command_start_timeout_s_,
            no_motion_retries, no_motion_max_retries_);
          if (!sendSnapshot(snapshot)) {
            result->error_code = Arm::Result::PATH_TOLERANCE_VIOLATED;
            result->error_string = "failed to retry D1 mode=1 arm target";
            handle->abort(result); finish(); return;
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
        "Arm endpoint timeout: motion_started=%s final_deg=%s error_deg=%s",
        motion_started ? "true" : "false", degreesString(armPositions(last)).c_str(),
        degreesString(final_error).c_str());
      result->error_code = Arm::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "D1 mode=1 arm endpoint timed out";
      handle->abort(result); finish();
    } catch (const std::exception & error) {
      result->error_code = Arm::Result::INVALID_GOAL;
      result->error_string = error.what();
      handle->abort(result); finish();
    }
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
    std::array<double, 7> snapshot{};
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      snapshot = desired_;
      snapshot[6] = target;
    }
    if (!sendSnapshot(snapshot)) {
      handle->abort(result); finish(); return;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      desired_ = snapshot;
    }
    RCLCPP_INFO(
      get_logger(), "Gripper target sent with full mode=1 snapshot: %.1f deg",
      gripper_closed_angle_deg_ + target / gripper_travel_m_ *
      (gripper_open_angle_deg_ - gripper_closed_angle_deg_));

    const double angle_span_deg = gripper_open_angle_deg_ - gripper_closed_angle_deg_;
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
        holdGripperAtMeasuredPosition();
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
          holdGripperAtMeasuredPosition();
          handle->abort(result); finish(); return;
        }
        ++no_motion_retries;
        RCLCPP_WARN(
          get_logger(),
          "Gripper has not started %.3f s after send; retrying absolute target (%d/%d)",
          command_start_timeout_s_, no_motion_retries, no_motion_max_retries_);
        if (!sendSnapshot(snapshot)) {
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
    holdGripperAtMeasuredPosition();
    handle->abort(result); finish();
  }

  std::string gateway_host_;
  int command_port_{15000};
  double feedback_timeout_s_{1.5};
  double position_tolerance_rad_{0.034906585};
  int stable_samples_{3};
  double assumed_speed_deg_s_{10.0};
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
  rclcpp_action::Server<Gripper>::SharedPtr gripper_server_;
};
}  // namespace d1_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<d1_ros2_control::D1Mode1Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
