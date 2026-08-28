#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "d1_manipulation/action/drop_object.hpp"
#include "d1_ros2_control/action/execute_joint_segment.hpp"

using namespace std::chrono_literals;

namespace d1_manipulation
{
namespace
{
template<typename T>
T parameterOrDeclare(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& fallback)
{
  if (node->has_parameter(name)) {
    T value;
    if (node->get_parameter(name, value)) return value;
  }
  return node->declare_parameter<T>(name, fallback);
}

geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d& value)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = value.translation().x();
  pose.position.y = value.translation().y();
  pose.position.z = value.translation().z();
  const Eigen::Quaterniond q(value.rotation());
  pose.orientation.x = q.x(); pose.orientation.y = q.y();
  pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}
}  // namespace

class DropObjectServer
{
public:
  using Drop = action::DropObject;
  using Handle = rclcpp_action::ServerGoalHandle<Drop>;
  using Gripper = control_msgs::action::GripperCommand;
  using Segment = d1_ros2_control::action::ExecuteJointSegment;

  explicit DropObjectServer(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node, parameterOrDeclare(node, "arm_group", std::string("arm"))),
    tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    backend_ = parameterOrDeclare(node_, "backend", std::string{});
    if (backend_ != "simulation" && backend_ != "real") {
      throw std::invalid_argument("backend must be explicitly set to 'simulation' or 'real'");
    }
    action_name_ = parameterOrDeclare(node_, "action_name", std::string("/arm/tasks/drop_object"));
    planning_frame_ = parameterOrDeclare(node_, "planning_frame", std::string("base_link"));
    gravity_frame_ = parameterOrDeclare(node_, "gravity_frame", std::string("world"));
    tcp_frame_ = parameterOrDeclare(node_, "tcp_frame", std::string("tcp_link"));
    stowed_ = parameterOrDeclare(node_, "stowed_joint_positions", std::vector<double>{0, -1.54, 1.55, 0, 0, 0});
    stowed_tolerance_ = parameterOrDeclare(node_, "stowed_tolerance_rad", 0.034906585);
    height_offsets_ = parameterOrDeclare(
      node_, "height_offsets_m",
      std::vector<double>{
        0.0, -0.01, 0.01, -0.02, 0.02, -0.03, 0.03, -0.04, 0.04,
        -0.05, 0.05, -0.06, 0.06, -0.08, 0.08});
    y_offsets_ = parameterOrDeclare(
      node_, "y_offsets_m",
      std::vector<double>{
        0.0, 0.003, -0.003, 0.006, -0.006, 0.009, -0.009,
        0.012, -0.012, 0.015, -0.015});
    yaw_offsets_ = parameterOrDeclare(node_, "yaw_offsets_degrees", std::vector<double>{0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90});
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_open_hold_ = parameterOrDeclare(node_, "gripper_open_hold_s", 0.5);
    segment_action_name_ = parameterOrDeclare(
      node_, "joint_segment_action_name", std::string("/arm_controller/execute_joint_segment"));
    real_motion_speed_deg_s_ = parameterOrDeclare(node_, "real_motion_speed_deg_s", 15.0);

    move_group_.setEndEffectorLink(tcp_frame_);
    move_group_.setPoseReferenceFrame(planning_frame_);
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPlanningTime(parameterOrDeclare(node_, "planning_time_s", 5.0));
    move_group_.setNumPlanningAttempts(parameterOrDeclare(node_, "planning_attempts", 6));
    move_group_.setMaxVelocityScalingFactor(parameterOrDeclare(node_, "velocity_scaling", 0.10));
    move_group_.setMaxAccelerationScalingFactor(parameterOrDeclare(node_, "acceleration_scaling", 0.10));
    move_group_.setGoalPositionTolerance(parameterOrDeclare(node_, "position_tolerance_m", 0.005));
    move_group_.setGoalOrientationTolerance(parameterOrDeclare(node_, "orientation_tolerance_rad", 0.03));
    gripper_client_ = rclcpp_action::create_client<Gripper>(node_, "/gripper_controller/gripper_cmd");
    segment_client_ = rclcpp_action::create_client<Segment>(node_, segment_action_name_);

    server_ = rclcpp_action::create_server<Drop>(
      node_, action_name_,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Drop::Goal> goal) {
        if (goal->target.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;
        cancel_.store(false); return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<Handle>) {
        requestCancel(); return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<Handle> handle) {
        std::thread([this, handle]() { execute(handle); }).detach();
      });
    RCLCPP_INFO(
      node_->get_logger(), "DropObject action server ready: %s backend=%s",
      action_name_.c_str(), backend_.c_str());
  }

private:
  bool nearPose(const std::vector<double>& target, double tolerance)
  {
    const auto current = move_group_.getCurrentJointValues();
    if (current.size() != target.size()) return false;
    for (std::size_t i = 0; i < current.size(); ++i) {
      if (std::abs(current[i] - target[i]) > tolerance) return false;
    }
    return true;
  }

  bool moveTo(const std::vector<double>& target)
  {
    move_group_.setStartStateToCurrentState();
    if (!move_group_.setJointValueTarget(target)) return false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      executePlan(plan);
  }

  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    if (backend_ == "simulation") {
      return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    const auto& trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.joint_names.size() != 6 || trajectory.points.empty() ||
      trajectory.points.back().positions.size() != trajectory.joint_names.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Cannot execute malformed real-arm drop endpoint");
      return false;
    }
    if (!segment_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Explicit D1 joint-segment action is unavailable: %s",
        segment_action_name_.c_str());
      return false;
    }

    Segment::Goal goal;
    goal.joint_names = trajectory.joint_names;
    goal.positions = trajectory.points.back().positions;
    goal.motion_profile = Segment::Goal::UNIFORM_JOINT_SPEED;
    goal.speed_deg_s = real_motion_speed_deg_s_;
    RCLCPP_INFO(
      node_->get_logger(),
      "Executing real-arm drop endpoint: profile=uniform_joint_speed speed=%.1f deg/s",
      real_motion_speed_deg_s_);

    auto sent = segment_client_->async_send_goal(goal);
    if (sent.wait_for(5s) != std::future_status::ready || !sent.get()) {
      RCLCPP_ERROR(node_->get_logger(), "Explicit D1 drop joint segment was rejected");
      return false;
    }
    const auto segment_goal = sent.get();
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_segment_goal_ = segment_goal;
    }
    auto result = segment_client_->async_get_result(segment_goal);
    while (result.wait_for(50ms) != std::future_status::ready) {
      if (cancel_.load()) {
        segment_client_->async_cancel_goal(segment_goal);
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_segment_goal_.reset();
        return false;
      }
    }
    const auto wrapped = result.get();
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_segment_goal_.reset();
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped.result || !wrapped.result->success)
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Explicit D1 drop joint segment failed: %s",
        wrapped.result ? wrapped.result->detail.c_str() : "no result");
      return false;
    }
    return true;
  }

  bool moveToStowed()
  {
    auto bounded_stowed = stowed_;
    const auto robot_model = move_group_.getRobotModel();
    const auto* joint_group = robot_model->getJointModelGroup(move_group_.getName());
    const auto& variable_names = joint_group->getVariableNames();
    for (std::size_t i = 0; i < bounded_stowed.size(); ++i) {
      const auto& bounds = robot_model->getVariableBounds(variable_names.at(i));
      bounded_stowed[i] = std::clamp(bounded_stowed[i], bounds.min_position_, bounds.max_position_);
    }
    return moveTo(bounded_stowed);
  }

  void feedback(const std::shared_ptr<Handle>& handle, const std::string& state,
    float progress, const std::string& detail)
  {
    auto message = std::make_shared<Drop::Feedback>();
    message->current_state = state; message->progress = progress; message->detail = detail;
    handle->publish_feedback(message);
  }

  void requestCancel()
  {
    cancel_.store(true);
    move_group_.stop();
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_gripper_goal_) gripper_client_->async_cancel_goal(active_gripper_goal_);
    if (active_segment_goal_) segment_client_->async_cancel_goal(active_segment_goal_);
  }

  void fail(const std::shared_ptr<Handle>& handle, uint8_t category,
    const std::string& state, const std::string& detail, bool canceled = false)
  {
    move_group_.stop();
    const bool returned = nearPose(stowed_, stowed_tolerance_);
    auto result = std::make_shared<Drop::Result>();
    result->success = false; result->failure_category = category;
    result->failed_state = state;
    result->detail = detail + "; motion stopped and current position held";
    result->returned_to_stowed = returned;
    const bool was_canceled = canceled || cancel_.load() || handle->is_canceling();
    RCLCPP_ERROR(
      node_->get_logger(),
      "DROP FAILED: category=%u failed_state=%s detail=%s returned_to_stowed=%s action_status=%s",
      static_cast<unsigned int>(category), state.c_str(), result->detail.c_str(),
      returned ? "true" : "false", was_canceled ? "CANCELED" : "ABORTED");
    if (was_canceled) handle->canceled(result); else handle->abort(result);
  }

  Eigen::Vector3d pointInPlanningFrame(const geometry_msgs::msg::PointStamped& target)
  {
    Eigen::Vector3d point(target.point.x, target.point.y, target.point.z);
    if (!point.allFinite()) throw std::runtime_error("trash-bin coordinates are not finite");
    if (target.header.frame_id == planning_frame_) return point;
    const auto transform = tf_buffer_.lookupTransform(
      planning_frame_, target.header.frame_id, tf2::TimePointZero, 3s);
    return tf2::transformToEigen(transform) * point;
  }

  Eigen::Vector3d gravityUp()
  {
    const auto transform = tf_buffer_.lookupTransform(
      planning_frame_, gravity_frame_, tf2::TimePointZero, 3s);
    return (tf2::transformToEigen(transform).rotation() * Eigen::Vector3d::UnitZ()).normalized();
  }

  bool commandGripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(5s)) return false;
    Gripper::Goal goal; goal.command.position = position;
    auto sent = gripper_client_->async_send_goal(goal);
    if (sent.wait_for(5s) != std::future_status::ready || !sent.get()) return false;
    const auto gripper_goal = sent.get();
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_gripper_goal_ = gripper_goal;
    }
    auto result = gripper_client_->async_get_result(gripper_goal);
    const auto deadline = std::chrono::steady_clock::now() + 8s;
    while (result.wait_for(50ms) != std::future_status::ready) {
      if (cancel_.load() || std::chrono::steady_clock::now() >= deadline) {
        gripper_client_->async_cancel_goal(gripper_goal);
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_gripper_goal_.reset();
        return false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_gripper_goal_.reset();
    }
    const auto wrapped = result.get();
    return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->reached_goal;
  }

  std::vector<std::string> heldObjectIds()
  {
    std::vector<std::string> ids;
    for (const auto& [id, object] : planning_scene_.getAttachedObjects()) {
      (void)object;
      if (id.rfind("held/", 0) == 0) ids.push_back(id);
    }
    return ids;
  }

  bool detachAndForgetHeldObject(const std::string& id)
  {
    auto objects = planning_scene_.getAttachedObjects({id});
    if (objects.size() != 1) return false;
    auto attached = objects.begin()->second;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!planning_scene_.applyAttachedCollisionObject(attached)) return false;

    // Detaching normally returns the collision object to MoveIt's world at
    // the release pose. The real object is subsequently simulated by MuJoCo
    // (and will fall into the bin), so that frozen world copy would be a ghost
    // obstacle. Remove it from the planning scene once it is detached.
    std::this_thread::sleep_for(100ms);
    planning_scene_.removeCollisionObjects({id});
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      const bool still_attached = !planning_scene_.getAttachedObjects({id}).empty();
      const auto known = planning_scene_.getKnownObjectNames();
      const bool still_in_world = std::find(known.begin(), known.end(), id) != known.end();
      if (!still_attached && !still_in_world) return true;
      if (!still_attached && still_in_world) planning_scene_.removeCollisionObjects({id});
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  void execute(const std::shared_ptr<Handle>& handle)
  {
    try {
      feedback(handle, "CHECK_PRECONDITIONS", 0.05F, "Resolving bin target, gravity and held object state");
      const auto held_ids = heldObjectIds();
      if (held_ids.size() > 1) {
        fail(handle, Drop::Result::FAILURE_INCOMPLETE_INFORMATION,
          "CHECK_PRECONDITIONS", "multiple held objects are attached in MoveIt"); return;
      }
      const Eigen::Vector3d bottom = pointInPlanningFrame(handle->get_goal()->target);
      const Eigen::Vector3d up = gravityUp();
      const Eigen::Vector3d horizontal = bottom - bottom.dot(up) * up;
      const Eigen::Isometry3d current_tcp = tf2::transformToEigen(
        tf_buffer_.lookupTransform(planning_frame_, tcp_frame_, tf2::TimePointZero, 3s));
      Eigen::Vector3d reference = current_tcp.rotation().col(0) -
        current_tcp.rotation().col(0).dot(up) * up;
      if (reference.norm() < 1e-6) reference = horizontal;
      if (reference.norm() < 1e-6) reference = up.unitOrthogonal();
      reference.normalize();

      moveit::planning_interface::MoveGroupInterface::Plan release_plan;
      geometry_msgs::msg::Pose selected_pose;
      double selected_height = 0.0, selected_y = 0.0, selected_yaw = 0.0;
      bool found = false;
      std::size_t candidate_index = 0;
      std::size_t ik_failures = 0;
      std::size_t planning_failures = 0;
      feedback(handle, "PLAN_RELEASE", 0.20F, "Searching gravity-aligned release candidates");
      for (const double height : height_offsets_) {
        for (const double y_offset : y_offsets_) {
          for (const double yaw_deg : yaw_offsets_) {
            ++candidate_index;
            const Eigen::Vector3d x =
              Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, up) * reference;
            const Eigen::Vector3d z = -up;
            const Eigen::Vector3d y = z.cross(x).normalized();
            const Eigen::Vector3d candidate_bottom =
              bottom + y_offset * Eigen::Vector3d::UnitY();
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear().col(0) = x; pose.linear().col(1) = y; pose.linear().col(2) = z;
            pose.translation() =
              candidate_bottom + (-candidate_bottom.dot(up) + height) * up;
            move_group_.setStartStateToCurrentState();
            if (!move_group_.setJointValueTarget(poseMessage(pose), tcp_frame_)) {
              ++ik_failures;
              RCLCPP_INFO(
                node_->get_logger(),
                "Release candidate %zu: height=%+.0f mm y=%+.0f mm yaw=%+.1f deg -> IK_FAILED",
                candidate_index, 1000.0 * height, 1000.0 * y_offset, yaw_deg);
              continue;
            }
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
              ++planning_failures;
              RCLCPP_INFO(
                node_->get_logger(),
                "Release candidate %zu: height=%+.0f mm y=%+.0f mm yaw=%+.1f deg "
                "-> PLANNING_FAILED",
                candidate_index, 1000.0 * height, 1000.0 * y_offset, yaw_deg);
              continue;
            }
            RCLCPP_INFO(
              node_->get_logger(),
              "Release candidate %zu: height=%+.0f mm y=%+.0f mm yaw=%+.1f deg "
              "-> PLAN_SUCCEEDED",
              candidate_index, 1000.0 * height, 1000.0 * y_offset, yaw_deg);
            release_plan = std::move(plan);
            selected_pose = poseMessage(pose);
            selected_height = height;
            selected_y = y_offset;
            selected_yaw = yaw_deg;
            found = true;
            break;
          }
          if (found) break;
        }
        if (found) break;
      }
      if (!found) {
        const std::string detail =
          "no gravity-aligned release plan after " + std::to_string(candidate_index) +
          " candidates (IK failures=" + std::to_string(ik_failures) +
          ", planning failures=" + std::to_string(planning_failures) +
          "); reposition Go2";
        fail(handle, Drop::Result::FAILURE_REPOSITION_REQUIRED,
          "PLAN_RELEASE", detail); return;
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "Selected release: base height offset=%+.0f mm y offset=%+.0f mm yaw=%+.1f deg",
        1000.0 * selected_height, 1000.0 * selected_y, selected_yaw);
      if (cancel_.load()) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR, "PLAN_RELEASE", "canceled", true); return;
      }
      feedback(handle, "MOVE_RELEASE", 0.55F, "Moving held object above trash bin");
      if (!executePlan(release_plan)) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "MOVE_RELEASE", "release trajectory execution failed"); return;
      }
      if (cancel_.load() || handle->is_canceling()) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR, "MOVE_RELEASE", "canceled", true); return;
      }
      feedback(handle, "RELEASE", 0.75F, "Opening gripper fully");
      if (!commandGripper(gripper_open_)) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "RELEASE", "gripper did not reach its fully open target"); return;
      }
      if (!held_ids.empty() && !detachAndForgetHeldObject(held_ids.front())) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "RELEASE", "failed to detach and remove held object from MoveIt"); return;
      }
      if (cancel_.load() || handle->is_canceling()) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR, "RELEASE", "canceled", true); return;
      }
      feedback(handle, "RELEASE_HOLD", 0.80F, "Holding gripper fully open");
      const auto hold_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(gripper_open_hold_);
      while (std::chrono::steady_clock::now() < hold_deadline) {
        if (cancel_.load() || handle->is_canceling()) {
          fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
            "RELEASE_HOLD", "canceled", true); return;
        }
        std::this_thread::sleep_for(20ms);
      }
      feedback(handle, "STOWED", 0.85F, "Planning directly from release pose to STOWED");
      if (!moveToStowed()) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "STOWED", "failed to return to STOWED"); return;
      }
      auto result = std::make_shared<Drop::Result>();
      result->success = true; result->failure_category = Drop::Result::FAILURE_NONE;
      result->detail = held_ids.empty() ?
        "empty-gripper release motion completed and arm returned to STOWED" :
        "held object released above trash bin and arm returned to STOWED";
      result->returned_to_stowed = true;
      result->release_pose.header.frame_id = planning_frame_;
      result->release_pose.header.stamp = node_->now(); result->release_pose.pose = selected_pose;
      result->height_offset_m = selected_height; result->release_yaw_degrees = selected_yaw;
      handle->succeed(result);
      RCLCPP_INFO(node_->get_logger(), "DROP SUCCEEDED");
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "DropObject error: %s", error.what());
      fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR, "INTERNAL", error.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<Gripper>::SharedPtr gripper_client_;
  rclcpp_action::Client<Segment>::SharedPtr segment_client_;
  rclcpp_action::Server<Drop>::SharedPtr server_;
  std::atomic<bool> cancel_{false};
  std::mutex active_goal_mutex_;
  rclcpp_action::ClientGoalHandle<Gripper>::SharedPtr active_gripper_goal_;
  rclcpp_action::ClientGoalHandle<Segment>::SharedPtr active_segment_goal_;
  std::string backend_, action_name_, planning_frame_, gravity_frame_, tcp_frame_;
  std::string segment_action_name_;
  std::vector<double> stowed_, height_offsets_, y_offsets_, yaw_offsets_;
  double stowed_tolerance_{};
  double gripper_open_{}, gripper_open_hold_{}, real_motion_speed_deg_s_{};
};
}  // namespace d1_manipulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "d1_drop_object", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<d1_manipulation::DropObjectServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node); executor.spin();
  server.reset(); rclcpp::shutdown(); return 0;
}
