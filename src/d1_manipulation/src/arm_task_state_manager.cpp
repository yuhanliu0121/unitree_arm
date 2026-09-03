#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "d1_manipulation/arm_task_state_machine.hpp"
#include "d1_interfaces/msg/arm_task_status.hpp"
#include "d1_manipulation/srv/apply_task_event.hpp"

namespace d1_manipulation
{
using namespace std::chrono_literals;

class ArmTaskStateManager
{
public:
  using Arm = control_msgs::action::FollowJointTrajectory;

  explicit ArmTaskStateManager(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    stowed_ = node_->declare_parameter<std::vector<double>>(
      "stowed_joint_positions", {0.0, -1.54, 1.55, 0.0, 0.0, 0.0});
    stowed_tolerance_ = node_->declare_parameter<double>("stowed_tolerance_rad", 0.034906585);
    initialization_timeout_s_ = node_->declare_parameter<double>("initialization_timeout_s", 60.0);
    feedback_timeout_s_ = node_->declare_parameter<double>("task_state_feedback_timeout_s", 1.0);
    startup_recovery_max_delta_ = node_->declare_parameter<double>(
      "startup_recovery_max_delta_rad", 0.785398163);
    startup_recovery_duration_s_ = node_->declare_parameter<double>(
      "startup_recovery_duration_s", 8.0);
    if (stowed_.size() != 6 || stowed_tolerance_ <= 0.0 ||
      initialization_timeout_s_ <= 0.0 || feedback_timeout_s_ <= 0.0 ||
      startup_recovery_max_delta_ <= stowed_tolerance_ || startup_recovery_duration_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid arm task state manager parameters");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_publisher_ = node_->create_publisher<d1_interfaces::msg::ArmTaskStatus>(
      "/arm/task_status", qos);
    event_service_ = node_->create_service<srv::ApplyTaskEvent>(
      "/arm/tasks/apply_state_event",
      [this](const std::shared_ptr<srv::ApplyTaskEvent::Request> request,
        std::shared_ptr<srv::ApplyTaskEvent::Response> response)
      {
        applyEvent(*request, *response);
      });
    arm_client_ = rclcpp_action::create_client<Arm>(
      node_, "/arm_controller/follow_joint_trajectory");
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr message) {onJointState(*message);});
    base_scene_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/arm/planning_scene_ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        base_scene_ready_ = message->data;
        completeInitializationIfReadyLocked();
      });
    health_timer_ = node_->create_wall_timer(100ms, [this]() {checkHealth();});
    initialization_started_ = std::chrono::steady_clock::now();
    publishStatus("Waiting for verified STOWED joint feedback");
    RCLCPP_INFO(node_->get_logger(), "Arm task state manager started in INITIALIZING");
  }

private:
  static ArmTaskEvent eventFromMessage(std::uint8_t value)
  {
    if (value > srv::ApplyTaskEvent::Request::FAULT) {
      throw std::out_of_range("unknown arm task event");
    }
    return static_cast<ArmTaskEvent>(value);
  }

  void onJointState(const sensor_msgs::msg::JointState& message)
  {
    std::vector<double> arm(6, 0.0);
    for (std::size_t joint = 0; joint < arm.size(); ++joint) {
      const std::string name = "Joint" + std::to_string(joint);
      const auto found = std::find(message.name.begin(), message.name.end(), name);
      if (found == message.name.end()) return;
      const auto index = static_cast<std::size_t>(std::distance(message.name.begin(), found));
      if (index >= message.position.size() || !std::isfinite(message.position[index])) return;
      arm[joint] = message.position[index];
    }

    std::lock_guard<std::mutex> lock(mutex_);
    last_feedback_ = std::chrono::steady_clock::now();
    feedback_received_ = true;
    if (machine_.snapshot().state != ArmTaskState::INITIALIZING) return;
    bool stowed = true;
    double maximum_delta = 0.0;
    for (std::size_t joint = 0; joint < arm.size(); ++joint) {
      const double delta = std::abs(arm[joint] - stowed_[joint]);
      maximum_delta = std::max(maximum_delta, delta);
      stowed = stowed && delta <= stowed_tolerance_;
    }
    latest_maximum_delta_ = maximum_delta;
    stowed_verified_ = stowed;
    completeInitializationIfReadyLocked();
  }

  void completeInitializationIfReadyLocked()
  {
    if (machine_.snapshot().state != ArmTaskState::INITIALIZING ||
      !stowed_verified_ || !base_scene_ready_)
    {
      return;
    }
    if (machine_.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED)) {
      failure_code_.clear();
      active_operation_.clear();
      active_phase_.clear();
      detail_ =
        "Verified canonical STOWED feedback and base planning scene; "
        "startup assumes an empty gripper";
      publishStatusLocked();
    }
  }

  void checkHealth()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    const auto state = machine_.snapshot().state;
    if (state == ArmTaskState::FAULTED) return;
    if (state == ArmTaskState::INITIALIZING) {
      if (feedback_received_ && !stowed_verified_ && !startup_recovery_requested_) {
        if (latest_maximum_delta_ > startup_recovery_max_delta_) {
          machine_.process(ArmTaskEvent::INITIALIZATION_FAILED);
          failure_code_ = "STARTUP_POSE_OUT_OF_RECOVERY_RANGE";
          detail_ = "Startup STOWED recovery was refused because the arm is outside the "
            "configured near-STOWED envelope";
          publishStatusLocked();
          return;
        }
        requestStartupRecoveryLocked();
      }
      if (now - initialization_started_ >
        std::chrono::duration<double>(initialization_timeout_s_))
      {
        machine_.process(ArmTaskEvent::INITIALIZATION_FAILED);
        failure_code_ = "INITIALIZATION_TIMEOUT";
        if (!feedback_received_) {
          detail_ = "Joint feedback was unavailable during initialization";
        } else if (!stowed_verified_) {
          detail_ = "Joint feedback arrived but canonical STOWED was not verified";
        } else {
          detail_ = "Canonical STOWED was verified but the base planning scene was not ready";
        }
        publishStatusLocked();
      }
      return;
    }
    if (!feedback_received_ ||
      now - last_feedback_ > std::chrono::duration<double>(feedback_timeout_s_))
    {
      machine_.process(ArmTaskEvent::FAULT);
      failure_code_ = "JOINT_FEEDBACK_TIMEOUT";
      detail_ = "Critical joint feedback became stale; no further task may be accepted";
      publishStatusLocked();
    }
  }

  void requestStartupRecoveryLocked()
  {
    if (!arm_client_->action_server_is_ready()) {
      detail_ = "Waiting for arm controller before bounded startup recovery to STOWED";
      return;
    }
    startup_recovery_requested_ = true;
    Arm::Goal goal;
    goal.trajectory.joint_names = {
      "Joint0", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = stowed_;
    const auto nanoseconds = static_cast<std::int64_t>(
      std::llround(startup_recovery_duration_s_ * 1.0e9));
    point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    goal.trajectory.points.push_back(std::move(point));

    rclcpp_action::Client<Arm>::SendGoalOptions options;
    options.goal_response_callback = [this](const auto& goal_handle) {
        if (goal_handle) return;
        std::lock_guard<std::mutex> lock(mutex_);
        faultInitializationLocked(
          "STARTUP_RECOVERY_REJECTED", "Arm controller rejected startup STOWED recovery");
      };
    options.result_callback = [this](const auto& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (machine_.snapshot().state != ArmTaskState::INITIALIZING) return;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
          result.result->error_code != Arm::Result::SUCCESSFUL)
        {
          faultInitializationLocked(
            "STARTUP_RECOVERY_FAILED", "Arm controller failed startup STOWED recovery");
          return;
        }
        detail_ = "Startup recovery command completed; verifying canonical STOWED feedback";
        publishStatusLocked();
      };
    arm_client_->async_send_goal(goal, options);
    detail_ = "Commanding bounded startup recovery to canonical STOWED";
    publishStatusLocked();
  }

  void faultInitializationLocked(const std::string& code, const std::string& detail)
  {
    if (machine_.snapshot().state != ArmTaskState::INITIALIZING) return;
    machine_.process(ArmTaskEvent::INITIALIZATION_FAILED);
    failure_code_ = code;
    detail_ = detail;
    publishStatusLocked();
  }

  void applyEvent(
    const srv::ApplyTaskEvent::Request& request,
    srv::ApplyTaskEvent::Response& response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      const auto event = eventFromMessage(request.event);
      const auto before = machine_.snapshot().state;
      response.accepted = machine_.process(event);
      const auto after = machine_.snapshot();
      if (response.accepted) {
        if (!request.active_phase.empty()) active_phase_ = request.active_phase;
        if (!request.failure_code.empty()) failure_code_ = request.failure_code;
        if (!request.detail.empty()) detail_ = request.detail;
        updateOperation(event, after.state);
        publishStatusLocked();
        RCLCPP_INFO(
          node_->get_logger(), "Arm task transition: %s --event=%u--> %s phase=%s",
          ArmTaskStateMachine::name(before), static_cast<unsigned>(request.event),
          ArmTaskStateMachine::name(after.state), active_phase_.c_str());
      } else {
        RCLCPP_WARN(
          node_->get_logger(), "Rejected arm task event %u in state %s",
          static_cast<unsigned>(request.event), ArmTaskStateMachine::name(before));
      }
      response.state = static_cast<std::uint8_t>(after.state);
      response.payload_state = static_cast<std::uint8_t>(after.payload);
      response.canonical_pose = static_cast<std::uint8_t>(after.pose);
      response.detail = response.accepted ? "transition accepted" :
        std::string("event is invalid in state ") + ArmTaskStateMachine::name(before);
    } catch (const std::exception& error) {
      response.accepted = false;
      const auto snapshot = machine_.snapshot();
      response.state = static_cast<std::uint8_t>(snapshot.state);
      response.payload_state = static_cast<std::uint8_t>(snapshot.payload);
      response.canonical_pose = static_cast<std::uint8_t>(snapshot.pose);
      response.detail = error.what();
    }
  }

  void updateOperation(ArmTaskEvent event, ArmTaskState state)
  {
    if (event == ArmTaskEvent::START_PICK) active_operation_ = "pick_object";
    if (event == ArmTaskEvent::START_DROP) active_operation_ = "drop_object";
    if (state == ArmTaskState::READY_STOWED || state == ArmTaskState::READY_CARRY) {
      active_operation_.clear();
      active_phase_.clear();
      failure_code_.clear();
    }
  }

  void publishStatus(const std::string& detail)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    detail_ = detail;
    publishStatusLocked();
  }

  void publishStatusLocked()
  {
    const auto snapshot = machine_.snapshot();
    if (snapshot.state == ArmTaskState::FAULTED && !fault_logged_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "***** ARM ENTERED FAULTED ***** failure_code=%s detail=%s "
        "active_operation=%s active_phase=%s payload_state=%u canonical_pose=%u",
        failure_code_.empty() ? "UNSPECIFIED" : failure_code_.c_str(),
        detail_.empty() ? "No diagnostic detail was provided" : detail_.c_str(),
        active_operation_.empty() ? "none" : active_operation_.c_str(),
        active_phase_.empty() ? "none" : active_phase_.c_str(),
        static_cast<unsigned int>(snapshot.payload),
        static_cast<unsigned int>(snapshot.pose));
      fault_logged_ = true;
    }
    d1_interfaces::msg::ArmTaskStatus message;
    message.stamp = node_->now();
    message.state = static_cast<std::uint8_t>(snapshot.state);
    message.payload_state = static_cast<std::uint8_t>(snapshot.payload);
    message.canonical_pose = static_cast<std::uint8_t>(snapshot.pose);
    message.active_operation = active_operation_;
    message.active_phase = active_phase_;
    message.failure_code = failure_code_;
    message.detail = detail_;
    status_publisher_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  ArmTaskStateMachine machine_;
  std::mutex mutex_;
  std::vector<double> stowed_;
  double stowed_tolerance_{};
  double initialization_timeout_s_{};
  double feedback_timeout_s_{};
  double startup_recovery_max_delta_{};
  double startup_recovery_duration_s_{};
  bool feedback_received_{false};
  bool stowed_verified_{false};
  bool base_scene_ready_{false};
  bool startup_recovery_requested_{false};
  bool fault_logged_{false};
  double latest_maximum_delta_{0.0};
  std::chrono::steady_clock::time_point initialization_started_;
  std::chrono::steady_clock::time_point last_feedback_;
  std::string active_operation_;
  std::string active_phase_;
  std::string failure_code_;
  std::string detail_;
  rclcpp::Publisher<d1_interfaces::msg::ArmTaskStatus>::SharedPtr status_publisher_;
  rclcpp::Service<srv::ApplyTaskEvent>::SharedPtr event_service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_scene_subscription_;
  rclcpp_action::Client<Arm>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};
}  // namespace d1_manipulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("d1_arm_task_state_manager");
  auto manager = std::make_shared<d1_manipulation::ArmTaskStateManager>(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  manager.reset();
  rclcpp::shutdown();
  return 0;
}
