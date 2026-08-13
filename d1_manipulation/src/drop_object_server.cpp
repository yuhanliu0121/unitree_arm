#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
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
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "d1_manipulation/action/drop_object.hpp"

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

  explicit DropObjectServer(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node, parameterOrDeclare(node, "arm_group", std::string("arm"))),
    tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    action_name_ = parameterOrDeclare(node_, "action_name", std::string("/arm/tasks/drop_object"));
    planning_frame_ = parameterOrDeclare(node_, "planning_frame", std::string("base_link"));
    gravity_frame_ = parameterOrDeclare(node_, "gravity_frame", std::string("world"));
    tcp_frame_ = parameterOrDeclare(node_, "tcp_frame", std::string("tcp_link"));
    carry_ = parameterOrDeclare(node_, "carry_joint_positions", std::vector<double>{0, -1.5, 1.5, 0, -0.6, 0});
    stowed_ = parameterOrDeclare(node_, "stowed_joint_positions", std::vector<double>{0, -1.5, 1.5, 0, 0, 0});
    carry_tolerance_ = parameterOrDeclare(node_, "carry_tolerance_rad", 0.08);
    stowed_tolerance_ = parameterOrDeclare(node_, "stowed_tolerance_rad", 0.08);
    min_distance_ = parameterOrDeclare(node_, "min_bin_distance_m", 0.35);
    max_distance_ = parameterOrDeclare(node_, "max_bin_distance_m", 0.45);
    height_offsets_ = parameterOrDeclare(node_, "height_offsets_m", std::vector<double>{0, -0.025, -0.05, 0.025, 0.05});
    yaw_offsets_ = parameterOrDeclare(node_, "yaw_offsets_degrees", std::vector<double>{0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90});
    bin_radius_ = parameterOrDeclare(node_, "trash_bin_radius_m", 0.15);
    bin_height_ = parameterOrDeclare(node_, "trash_bin_height_m", 0.10);
    bin_wall_ = parameterOrDeclare(node_, "trash_bin_wall_thickness_m", 0.01);
    bin_segments_ = parameterOrDeclare(node_, "trash_bin_wall_segments", 16);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_open_hold_ = parameterOrDeclare(node_, "gripper_open_hold_s", 0.5);

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

    server_ = rclcpp_action::create_server<Drop>(
      node_, action_name_,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Drop::Goal> goal) {
        if (goal->target.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_.store(false); return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<Handle>) {
        cancel_.store(true); move_group_.stop(); return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<Handle> handle) {
        std::thread([this, handle]() { execute(handle); }).detach();
      });
    RCLCPP_INFO(node_->get_logger(), "DropObject action server ready: %s", action_name_.c_str());
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
      move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void feedback(const std::shared_ptr<Handle>& handle, const std::string& state,
    float progress, const std::string& detail)
  {
    auto message = std::make_shared<Drop::Feedback>();
    message->current_state = state; message->progress = progress; message->detail = detail;
    handle->publish_feedback(message);
  }

  void fail(const std::shared_ptr<Handle>& handle, uint8_t category,
    const std::string& state, const std::string& detail, bool canceled = false)
  {
    const bool returned = nearPose(stowed_, stowed_tolerance_) || moveTo(stowed_);
    planning_scene_.removeCollisionObjects({"drop_trash_bin"});
    auto result = std::make_shared<Drop::Result>();
    result->success = false; result->failure_category = category;
    result->failed_state = state; result->detail = detail; result->returned_to_stowed = returned;
    if (canceled) handle->canceled(result); else handle->abort(result);
    busy_.store(false);
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
    auto result = gripper_client_->async_get_result(sent.get());
    if (result.wait_for(8s) != std::future_status::ready) return false;
    const auto wrapped = result.get();
    return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->reached_goal;
  }

  bool applyTrashBin(const Eigen::Vector3d& bottom, const Eigen::Vector3d& up)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = planning_frame_; object.id = "drop_trash_bin";
    Eigen::Vector3d radial_reference = Eigen::Vector3d::UnitX() -
      Eigen::Vector3d::UnitX().dot(up) * up;
    if (radial_reference.norm() < 1e-6) radial_reference = Eigen::Vector3d::UnitY();
    radial_reference.normalize();
    const Eigen::Vector3d tangent_reference = up.cross(radial_reference).normalized();
    const double wall_radius = bin_radius_ - 0.5 * bin_wall_;
    const double tangent_half = wall_radius * std::tan(M_PI / bin_segments_) + 0.001;
    for (int i = 0; i < bin_segments_; ++i) {
      const double theta = 2.0 * M_PI * i / bin_segments_;
      const Eigen::Vector3d radial =
        std::cos(theta) * radial_reference + std::sin(theta) * tangent_reference;
      const Eigen::Vector3d tangent = up.cross(radial).normalized();
      Eigen::Matrix3d rotation;
      rotation.col(0) = tangent; rotation.col(1) = -radial; rotation.col(2) = up;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = rotation;
      pose.translation() = bottom + wall_radius * radial + 0.5 * bin_height_ * up;
      shape_msgs::msg::SolidPrimitive wall;
      wall.type = shape_msgs::msg::SolidPrimitive::BOX;
      wall.dimensions = {2.0 * tangent_half, bin_wall_, bin_height_};
      object.primitives.push_back(wall); object.primitive_poses.push_back(poseMessage(pose));
    }
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return planning_scene_.applyCollisionObject(object);
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

  bool detachHeldObject(const std::string& id)
  {
    auto objects = planning_scene_.getAttachedObjects({id});
    if (objects.size() != 1) return false;
    auto attached = objects.begin()->second;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    return planning_scene_.applyAttachedCollisionObject(attached);
  }

  void execute(const std::shared_ptr<Handle>& handle)
  {
    try {
      feedback(handle, "CHECK_PRECONDITIONS", 0.05F, "Resolving bin target, gravity and CARRY state");
      if (!nearPose(carry_, carry_tolerance_)) {
        fail(handle, Drop::Result::FAILURE_INCOMPLETE_INFORMATION,
          "CHECK_PRECONDITIONS", "arm is not in CARRY pose"); return;
      }
      const auto held_ids = heldObjectIds();
      if (held_ids.size() != 1) {
        fail(handle, Drop::Result::FAILURE_INCOMPLETE_INFORMATION,
          "CHECK_PRECONDITIONS",
          held_ids.empty() ? "no held object is attached in MoveIt" :
          "multiple held objects are attached in MoveIt"); return;
      }
      const Eigen::Vector3d bottom = pointInPlanningFrame(handle->get_goal()->target);
      const Eigen::Vector3d up = gravityUp();
      const Eigen::Vector3d horizontal = bottom - bottom.dot(up) * up;
      const double distance = horizontal.norm();
      if (distance < min_distance_ || distance > max_distance_) {
        fail(handle, Drop::Result::FAILURE_REPOSITION_REQUIRED,
          "CHECK_PRECONDITIONS", "trash-bin horizontal distance is outside 0.35..0.45 m"); return;
      }
      if (!applyTrashBin(bottom, up)) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "PLAN_RELEASE", "failed to add trash-bin walls to MoveIt"); return;
      }

      const Eigen::Isometry3d current_tcp = tf2::transformToEigen(
        tf_buffer_.lookupTransform(planning_frame_, tcp_frame_, tf2::TimePointZero, 3s));
      Eigen::Vector3d reference = current_tcp.rotation().col(0) -
        current_tcp.rotation().col(0).dot(up) * up;
      if (reference.norm() < 1e-6) reference = horizontal.normalized();
      reference.normalize();

      moveit::planning_interface::MoveGroupInterface::Plan release_plan;
      geometry_msgs::msg::Pose selected_pose;
      double selected_height = 0.0, selected_yaw = 0.0;
      bool found = false;
      feedback(handle, "PLAN_RELEASE", 0.20F, "Searching gravity-aligned release candidates");
      for (const double height : height_offsets_) {
        for (const double yaw_deg : yaw_offsets_) {
          const Eigen::Vector3d x = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, up) * reference;
          const Eigen::Vector3d z = -up;
          const Eigen::Vector3d y = z.cross(x).normalized();
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.linear().col(0) = x; pose.linear().col(1) = y; pose.linear().col(2) = z;
          pose.translation() = bottom + (-bottom.dot(up) + height) * up;
          move_group_.setStartStateToCurrentState();
          if (!move_group_.setJointValueTarget(poseMessage(pose), tcp_frame_)) continue;
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) continue;
          release_plan = std::move(plan);
          selected_pose = poseMessage(pose); selected_height = height; selected_yaw = yaw_deg;
          found = true; break;
        }
        if (found) break;
      }
      if (!found) {
        fail(handle, Drop::Result::FAILURE_REPOSITION_REQUIRED,
          "PLAN_RELEASE", "no gravity-aligned release plan; reposition Go2"); return;
      }
      RCLCPP_INFO(node_->get_logger(), "Selected release: base height offset=%+.0f mm yaw=%+.1f deg",
        1000.0 * selected_height, selected_yaw);
      if (cancel_.load()) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR, "PLAN_RELEASE", "canceled", true); return;
      }
      feedback(handle, "MOVE_RELEASE", 0.55F, "Moving held object above trash bin");
      if (move_group_.execute(release_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "MOVE_RELEASE", "release trajectory execution failed"); return;
      }
      feedback(handle, "RELEASE", 0.75F, "Opening gripper fully");
      if (!commandGripper(gripper_open_)) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "RELEASE", "gripper did not reach its fully open target"); return;
      }
      if (!detachHeldObject(held_ids.front())) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "RELEASE", "failed to detach held object from MoveIt"); return;
      }
      feedback(handle, "RELEASE_HOLD", 0.80F, "Holding gripper fully open");
      std::this_thread::sleep_for(std::chrono::duration<double>(gripper_open_hold_));
      feedback(handle, "STOWED", 0.85F, "Planning directly from release pose to STOWED");
      if (!moveTo(stowed_)) {
        fail(handle, Drop::Result::FAILURE_EXECUTION_ERROR,
          "STOWED", "failed to return to STOWED"); return;
      }
      planning_scene_.removeCollisionObjects({"drop_trash_bin"});
      auto result = std::make_shared<Drop::Result>();
      result->success = true; result->failure_category = Drop::Result::FAILURE_NONE;
      result->detail = "held object released above trash bin and arm returned to STOWED";
      result->returned_to_stowed = true;
      result->release_pose.header.frame_id = planning_frame_;
      result->release_pose.header.stamp = node_->now(); result->release_pose.pose = selected_pose;
      result->height_offset_m = selected_height; result->release_yaw_degrees = selected_yaw;
      handle->succeed(result); busy_.store(false);
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
  rclcpp_action::Server<Drop>::SharedPtr server_;
  std::atomic<bool> busy_{false}, cancel_{false};
  std::string action_name_, planning_frame_, gravity_frame_, tcp_frame_;
  std::vector<double> carry_, stowed_, height_offsets_, yaw_offsets_;
  double carry_tolerance_{}, stowed_tolerance_{}, min_distance_{}, max_distance_;
  double bin_radius_{}, bin_height_{}, bin_wall_{};
  int bin_segments_{}; double gripper_open_{}, gripper_open_hold_{};
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
