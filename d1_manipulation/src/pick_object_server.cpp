#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
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
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "d1_manipulation/action/observe_target.hpp"
#include "d1_manipulation/action/pick_object.hpp"
#include "d1_manipulation/srv/estimate_cube.hpp"

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

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA value;
  value.r = r; value.g = g; value.b = b; value.a = a;
  return value;
}
}  // namespace

class PickObjectServer
{
public:
  using Pick = action::PickObject;
  using PickHandle = rclcpp_action::ServerGoalHandle<Pick>;
  using Observe = action::ObserveTarget;
  using Gripper = control_msgs::action::GripperCommand;

  explicit PickObjectServer(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node, parameterOrDeclare(node, "arm_group", std::string("arm"))),
    tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    action_name_ = parameterOrDeclare(node_, "action_name", std::string("/arm/tasks/pick_object"));
    observe_name_ = parameterOrDeclare(node_, "observe_action_name", std::string("/arm/debug/observe_target"));
    estimate_name_ = parameterOrDeclare(node_, "cube_estimation_service_name", std::string("/arm/perception/estimate_cube"));
    planning_frame_ = parameterOrDeclare(node_, "planning_frame", std::string("base_link"));
    link6_frame_ = parameterOrDeclare(node_, "link6_frame", std::string("Link6"));
    tcp_frame_ = parameterOrDeclare(node_, "tcp_frame", std::string("tcp_link"));
    camera_frame_ = parameterOrDeclare(node_, "color_optical_frame", std::string("wrist_camera_color_optical_frame"));
    cube_size_ = parameterOrDeclare(node_, "cube_size_m", 0.05);
    pregrasp_distance_max_ = parameterOrDeclare(node_, "pregrasp_distance_max_m", 0.080);
    pregrasp_distance_min_ = parameterOrDeclare(node_, "pregrasp_distance_min_m", 0.015);
    pregrasp_distance_step_ = parameterOrDeclare(node_, "pregrasp_distance_step_m", 0.005);
    grasp_distance_min_ = parameterOrDeclare(node_, "grasp_distance_min_m", -0.040);
    grasp_distance_max_ = parameterOrDeclare(node_, "grasp_distance_max_m", -0.024);
    grasp_distance_step_ = parameterOrDeclare(node_, "grasp_distance_step_m", 0.002);
    if (cube_size_ <= 0.0 || pregrasp_distance_min_ < 0.0 ||
      pregrasp_distance_max_ < pregrasp_distance_min_ || pregrasp_distance_step_ <= 0.0 ||
      grasp_distance_min_ > grasp_distance_max_ || grasp_distance_max_ >= 0.0 ||
      grasp_distance_step_ <= 0.0)
    {
      throw std::invalid_argument("invalid signed cube pregrasp/grasp search parameters");
    }
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    top_distances_ = parameterOrDeclare(node_, "top_observation_distances_m", std::vector<double>{0.35, 0.40, 0.45, 0.50, 0.55, 0.60});
    top_rolls_ = parameterOrDeclare(node_, "top_observation_roll_degrees", std::vector<double>{0, 90, -90, 180, 45, -45, 135, -135});
    camera_settle_ = parameterOrDeclare(node_, "camera_settle_s", 0.5);
    cartesian_step_ = parameterOrDeclare(node_, "cartesian_step_m", 0.005);
    minimum_fraction_ = parameterOrDeclare(node_, "minimum_cartesian_fraction", 0.95);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_closed_ = parameterOrDeclare(node_, "gripper_closed_m", 0.0);
    grasp_settle_ = parameterOrDeclare(node_, "grasp_settle_s", 0.5);
    stowed_tolerance_ = parameterOrDeclare(node_, "stowed_tolerance_rad", 0.08);
    stowed_ = parameterOrDeclare(node_, "stowed_joint_positions", std::vector<double>{0, -1.5, 1.5, 0, 0, 0});

    move_group_.setEndEffectorLink(tcp_frame_);
    move_group_.setPoseReferenceFrame(planning_frame_);
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPlanningTime(parameterOrDeclare(node_, "planning_time_s", 5.0));
    move_group_.setNumPlanningAttempts(parameterOrDeclare(node_, "planning_attempts", 6));
    move_group_.setMaxVelocityScalingFactor(parameterOrDeclare(node_, "velocity_scaling", 0.10));
    move_group_.setMaxAccelerationScalingFactor(parameterOrDeclare(node_, "acceleration_scaling", 0.10));
    move_group_.setGoalPositionTolerance(parameterOrDeclare(node_, "position_tolerance_m", 0.005));
    move_group_.setGoalOrientationTolerance(parameterOrDeclare(node_, "orientation_tolerance_rad", 0.03));

    observe_client_ = rclcpp_action::create_client<Observe>(node_, observe_name_);
    estimate_client_ = node_->create_client<srv::EstimateCube>(estimate_name_);
    state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity");
    gripper_client_ = rclcpp_action::create_client<Gripper>(node_, "/gripper_controller/gripper_cmd");
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm/debug/cube_grasp_markers", rclcpp::QoS(1).transient_local().reliable());
    server_ = rclcpp_action::create_server<Pick>(
      node_, action_name_,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Pick::Goal> goal) {
        if (busy_.exchange(true) || goal->target.header.frame_id.empty() ||
          goal->stop_after > Pick::Goal::GRASP_AND_LIFT)
        {
          busy_.store(false);
          return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_.store(false);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<PickHandle>) {
        cancel_.store(true); move_group_.stop();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<PickHandle> handle) {
        std::thread([this, handle]() { execute(handle); }).detach();
      });
    RCLCPP_INFO(node_->get_logger(), "PickObject action server ready: %s", action_name_.c_str());
  }

private:
  std::vector<double> descendingValues(double first, double last, double step) const
  {
    std::vector<double> values;
    if (step <= 0.0 || first < last) return values;
    for (double value = first; value >= last - 1e-9; value -= step) values.push_back(value);
    return values;
  }

  void feedback(const std::shared_ptr<PickHandle>& handle, const std::string& state,
    float progress, const std::string& detail)
  {
    auto value = std::make_shared<Pick::Feedback>();
    value->current_state = state; value->progress = progress; value->detail = detail;
    handle->publish_feedback(value);
  }

  bool isStowed()
  {
    const auto current = move_group_.getCurrentJointValues();
    if (current.size() != stowed_.size()) return false;
    for (std::size_t i = 0; i < current.size(); ++i) {
      if (std::abs(current[i] - stowed_[i]) > stowed_tolerance_) return false;
    }
    return true;
  }

  bool returnStowed()
  {
    if (isStowed()) return true;
    move_group_.setStartStateToCurrentState();
    if (!move_group_.setJointValueTarget(stowed_)) return false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS && isStowed();
  }

  void fail(const std::shared_ptr<PickHandle>& handle, uint8_t category,
    const std::string& state, const std::string& detail, bool canceled = false)
  {
    const bool returned = returnStowed();
    auto result = std::make_shared<Pick::Result>();
    result->success = false; result->failure_category = category;
    result->failed_state = state; result->detail = detail;
    result->returned_to_stowed = returned;
    if (canceled) handle->canceled(result); else handle->abort(result);
    busy_.store(false);
  }

  srv::EstimateCube::Response::SharedPtr estimate(uint8_t stage,
    const geometry_msgs::msg::PointStamped& hint, const Eigen::Vector3d& normal = Eigen::Vector3d::Zero(),
    double offset = 0.0)
  {
    if (!estimate_client_->wait_for_service(5s)) throw std::runtime_error("EstimateCube service unavailable");
    auto request = std::make_shared<srv::EstimateCube::Request>();
    request->stage = stage; request->target_hint = hint;
    request->ground_normal.x = normal.x(); request->ground_normal.y = normal.y(); request->ground_normal.z = normal.z();
    request->ground_offset = offset;
    auto future = estimate_client_->async_send_request(request);
    if (future.wait_for(15s) != std::future_status::ready) throw std::runtime_error("EstimateCube timed out");
    return future.get();
  }

  bool observe(const geometry_msgs::msg::PointStamped& target)
  {
    if (!observe_client_->wait_for_action_server(5s)) return false;
    Observe::Goal goal; goal.target = target;
    auto sent = observe_client_->async_send_goal(goal);
    if (sent.wait_for(5s) != std::future_status::ready || !sent.get()) return false;
    auto result = observe_client_->async_get_result(sent.get());
    return result.wait_for(40s) == std::future_status::ready &&
      result.get().code == rclcpp_action::ResultCode::SUCCEEDED && result.get().result->success;
  }

  Eigen::Isometry3d lookup(const std::string& target, const std::string& source)
  {
    return tf2::transformToEigen(tf_buffer_.lookupTransform(target, source, tf2::TimePointZero, 3s));
  }

  bool moveCameraTopDown(const Eigen::Vector3d& target, const Eigen::Vector3d& up)
  {
    Eigen::Vector3d camera_z = -up;
    Eigen::Vector3d reference = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(camera_z) * camera_z;
    if (reference.norm() < 1e-6) reference = Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitY().dot(camera_z) * camera_z;
    reference.normalize();
    const Eigen::Isometry3d link6_from_camera = lookup(link6_frame_, camera_frame_);
    for (const double distance : top_distances_) {
      for (const double roll_deg : top_rolls_) {
        const Eigen::Vector3d image_up = Eigen::AngleAxisd(roll_deg * M_PI / 180.0, camera_z) * reference;
        const Eigen::Vector3d camera_y = -image_up;
        const Eigen::Vector3d camera_x = camera_y.cross(camera_z).normalized();
        Eigen::Isometry3d planning_from_camera = Eigen::Isometry3d::Identity();
        planning_from_camera.linear().col(0) = camera_x;
        planning_from_camera.linear().col(1) = camera_y;
        planning_from_camera.linear().col(2) = camera_z;
        planning_from_camera.translation() = target + distance * up;
        const Eigen::Isometry3d planning_from_link6 = planning_from_camera * link6_from_camera.inverse();
        move_group_.setEndEffectorLink(link6_frame_);
        move_group_.setStartStateToCurrentState();
        if (!move_group_.setJointValueTarget(poseMessage(planning_from_link6), link6_frame_)) continue;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) continue;
        RCLCPP_INFO(node_->get_logger(), "Top observation candidate: distance=%.2f roll=%.1f deg", distance, roll_deg);
        if (move_group_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
        move_group_.setEndEffectorLink(tcp_frame_);
        std::this_thread::sleep_for(std::chrono::duration<double>(camera_settle_));
        return true;
      }
    }
    move_group_.setEndEffectorLink(tcp_frame_);
    return false;
  }

  bool commandGripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(5s)) return false;
    Gripper::Goal goal; goal.command.position = position;
    auto sent = gripper_client_->async_send_goal(goal);
    if (sent.wait_for(5s) != std::future_status::ready || !sent.get()) return false;
    auto result = gripper_client_->async_get_result(sent.get());
    return result.wait_for(8s) == std::future_status::ready &&
      result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool executeCartesian(const geometry_msgs::msg::Pose& pose)
  {
    move_group_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = move_group_.computeCartesianPath({pose}, cartesian_step_, 0.0, message, true);
    if (fraction < minimum_fraction_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Cartesian path fraction %.3f is below %.3f",
        fraction, minimum_fraction_);
      diagnoseCartesianCollision(pose);
      return false;
    }
    auto state = move_group_.getCurrentState(2.0);
    if (!state) return false;
    robot_trajectory::RobotTrajectory trajectory(move_group_.getRobotModel(), "arm");
    trajectory.setRobotTrajectoryMsg(*state, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.15, 0.15)) return false;
    trajectory.getRobotTrajectoryMsg(message);
    moveit::planning_interface::MoveGroupInterface::Plan plan; plan.trajectory_ = message;
    return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool executeTrajectory(moveit_msgs::msg::RobotTrajectory message)
  {
    auto state = move_group_.getCurrentState(2.0);
    if (!state) return false;
    if (!message.joint_trajectory.points.empty()) {
      auto& first = message.joint_trajectory.points.front();
      first.positions.resize(message.joint_trajectory.joint_names.size());
      for (std::size_t i = 0; i < message.joint_trajectory.joint_names.size(); ++i) {
        first.positions[i] = state->getVariablePosition(message.joint_trajectory.joint_names[i]);
      }
      first.velocities.clear(); first.accelerations.clear(); first.effort.clear();
      first.time_from_start = rclcpp::Duration(0, 0);
    }
    robot_trajectory::RobotTrajectory trajectory(move_group_.getRobotModel(), "arm");
    trajectory.setRobotTrajectoryMsg(*state, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.15, 0.15)) return false;
    trajectory.getRobotTrajectoryMsg(message);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = std::move(message);
    return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  moveit_msgs::msg::RobotTrajectory reverseLiftTrajectory(
    const moveit_msgs::msg::RobotTrajectory& descent, const Eigen::Vector3d& up)
  {
    moveit_msgs::msg::RobotTrajectory lift;
    lift.joint_trajectory.header = descent.joint_trajectory.header;
    lift.joint_trajectory.joint_names = descent.joint_trajectory.joint_names;
    if (descent.joint_trajectory.points.empty()) return lift;

    auto state = move_group_.getCurrentState(2.0);
    if (!state) return lift;
    const auto& names = descent.joint_trajectory.joint_names;
    auto set_positions = [&](const trajectory_msgs::msg::JointTrajectoryPoint& point) {
      for (std::size_t i = 0; i < std::min(names.size(), point.positions.size()); ++i) {
        state->setVariablePosition(names[i], point.positions[i]);
      }
      state->update();
    };
    set_positions(descent.joint_trajectory.points.back());
    const Eigen::Vector3d start = state->getGlobalLinkTransform(tcp_frame_).translation();
    for (auto iterator = descent.joint_trajectory.points.rbegin();
      iterator != descent.joint_trajectory.points.rend(); ++iterator)
    {
      auto point = *iterator;
      point.velocities.clear(); point.accelerations.clear(); point.effort.clear();
      point.time_from_start = rclcpp::Duration(0, 0);
      lift.joint_trajectory.points.push_back(std::move(point));
      set_positions(*iterator);
      const double height = (state->getGlobalLinkTransform(tcp_frame_).translation() - start).dot(up);
      if (height >= lift_distance_ - 1e-4) break;
    }
    return lift;
  }

  double computeCartesianFromPlanEnd(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const geometry_msgs::msg::Pose& target, bool avoid_collisions,
    moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    auto current = move_group_.getCurrentState(2.0);
    if (!current) return 0.0;
    robot_trajectory::RobotTrajectory planned(move_group_.getRobotModel(), "arm");
    planned.setRobotTrajectoryMsg(*current, plan.trajectory_);
    if (planned.empty()) return 0.0;
    move_group_.setStartState(planned.getLastWayPoint());
    const double fraction = move_group_.computeCartesianPath(
      {target}, cartesian_step_, 0.0, trajectory, avoid_collisions);
    move_group_.setStartStateToCurrentState();
    return fraction;
  }

  double cartesianFractionFromPlanEnd(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const geometry_msgs::msg::Pose& target)
  {
    moveit_msgs::msg::RobotTrajectory validation;
    // The grasped object is intentionally contacted, so this pre-check only
    // verifies that the selected IK branch remains continuous through the
    // complete vertical descent. Collision checking remains enabled during
    // the real descent after the target collision object is removed.
    const double fraction = computeCartesianFromPlanEnd(plan, target, false, validation);
    return fraction;
  }

  void diagnoseCartesianCollision(const geometry_msgs::msg::Pose& pose)
  {
    if (!state_validity_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(node_->get_logger(), "State-validity service unavailable for Cartesian diagnosis");
      return;
    }

    // Recompute the same straight path without collision rejection, then ask
    // MoveIt's planning scene for the first state that becomes invalid.
    move_group_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory unchecked;
    const double fraction = move_group_.computeCartesianPath(
      {pose}, cartesian_step_, 0.0, unchecked, false);
    if (fraction < minimum_fraction_ || unchecked.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unchecked Cartesian path also stopped at %.3f; failure is IK/joint continuity, not collision",
        fraction);
      return;
    }

    auto state = move_group_.getCurrentState(2.0);
    if (!state) return;
    const auto& names = unchecked.joint_trajectory.joint_names;
    for (std::size_t index = 0; index < unchecked.joint_trajectory.points.size(); ++index) {
      const auto& positions = unchecked.joint_trajectory.points[index].positions;
      for (std::size_t joint = 0; joint < std::min(names.size(), positions.size()); ++joint) {
        state->setVariablePosition(names[joint], positions[joint]);
      }
      state->update();
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      moveit::core::robotStateToRobotStateMsg(*state, request->robot_state);
      request->group_name = "arm";
      auto response_future = state_validity_client_->async_send_request(request);
      if (response_future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "State-validity request timed out");
        return;
      }
      const auto response = response_future.get();
      if (!response->valid) {
        if (response->contacts.empty()) {
          RCLCPP_ERROR(
            node_->get_logger(), "Cartesian state %zu/%zu is invalid without reported contacts",
            index + 1, unchecked.joint_trajectory.points.size());
        }
        for (const auto& contact : response->contacts) {
          RCLCPP_ERROR(
            node_->get_logger(), "Cartesian collision at state %zu/%zu: %s <-> %s, depth=%.6f m",
            index + 1, unchecked.joint_trajectory.points.size(), contact.contact_body_1.c_str(),
            contact.contact_body_2.c_str(), contact.depth);
        }
        return;
      }
    }
    RCLCPP_ERROR(node_->get_logger(), "Unchecked Cartesian states are all valid; planning-scene race suspected");
  }

  bool removeTargetCollision()
  {
    const std::vector<std::string> target_ids{"yellow_cube", "observe_target"};
    planning_scene_.removeCollisionObjects(target_ids);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto names = planning_scene_.getKnownObjectNames();
      std::string joined;
      for (const auto& name : names) {
        if (!joined.empty()) joined += ",";
        joined += name;
      }
      RCLCPP_INFO(node_->get_logger(), "MoveIt world objects before descent: [%s]", joined.c_str());
      const bool target_remains = std::any_of(
        target_ids.begin(), target_ids.end(), [&names](const std::string& id) {
          return std::find(names.begin(), names.end(), id) != names.end();
        });
      if (!target_remains) {
        // PlanningSceneInterface observes the service-side world first;
        // MoveGroup's PlanningSceneMonitor consumes the update asynchronously.
        std::this_thread::sleep_for(1s);
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "target collision remained in the MoveIt world after removal");
    return false;
  }

  bool restoreTargetCollision(
    const std::map<std::string, moveit_msgs::msg::CollisionObject>& target_objects)
  {
    std::vector<moveit_msgs::msg::CollisionObject> restore;
    for (const auto& entry : target_objects) restore.push_back(entry.second);
    if (restore.empty()) return true;
    if (!planning_scene_.applyCollisionObjects(restore)) return false;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto names = planning_scene_.getKnownObjectNames();
      const bool restored = std::any_of(
        target_objects.begin(), target_objects.end(), [&names](const auto& entry) {
          return std::find(names.begin(), names.end(), entry.first) != names.end();
        });
      if (restored) {
        std::this_thread::sleep_for(1s);
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  void publishMarkers(const srv::EstimateCube::Response& estimate,
    const geometry_msgs::msg::Pose& grasp, const geometry_msgs::msg::Pose& pregrasp)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear; clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    visualization_msgs::msg::Marker plane;
    plane.header.frame_id = planning_frame_; plane.ns = "ground_plane"; plane.id = 0;
    plane.type = visualization_msgs::msg::Marker::CUBE; plane.action = visualization_msgs::msg::Marker::ADD;
    Eigen::Vector3d normal(estimate.ground_normal.x, estimate.ground_normal.y, estimate.ground_normal.z);
    Eigen::Quaterniond plane_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
    plane.pose.orientation.x = plane_q.x(); plane.pose.orientation.y = plane_q.y();
    plane.pose.orientation.z = plane_q.z(); plane.pose.orientation.w = plane_q.w();
    Eigen::Vector3d on_plane = -estimate.ground_offset * normal;
    plane.pose.position.x = on_plane.x(); plane.pose.position.y = on_plane.y(); plane.pose.position.z = on_plane.z();
    plane.scale.x = 0.8; plane.scale.y = 0.6; plane.scale.z = 0.003; plane.color = color(0.3F, 0.7F, 1.0F, 0.25F);
    array.markers.push_back(plane);
    int id = 1;
    for (const auto& value : {std::make_pair(std::string("cube_center"), grasp), std::make_pair(std::string("pregrasp"), pregrasp)}) {
      visualization_msgs::msg::Marker axes;
      axes.header.frame_id = planning_frame_; axes.ns = value.first; axes.id = id++;
      axes.type = visualization_msgs::msg::Marker::ARROW; axes.action = visualization_msgs::msg::Marker::ADD;
      axes.pose = value.second; axes.scale.x = 0.10; axes.scale.y = 0.015; axes.scale.z = 0.02;
      axes.color = value.first == "cube_center" ? color(1, 0, 1) : color(0, 1, 1);
      array.markers.push_back(axes);
    }
    visualization_msgs::msg::Marker polygon;
    polygon.header.frame_id = planning_frame_; polygon.ns = "fitted_top_square"; polygon.id = id++;
    polygon.type = visualization_msgs::msg::Marker::LINE_STRIP; polygon.action = visualization_msgs::msg::Marker::ADD;
    polygon.scale.x = 0.006; polygon.color = color(1, 1, 0);
    for (const auto& point : estimate.top_polygon.polygon.points) {
      geometry_msgs::msg::Point p; p.x = point.x; p.y = point.y; p.z = point.z; polygon.points.push_back(p);
    }
    if (!polygon.points.empty()) polygon.points.push_back(polygon.points.front());
    array.markers.push_back(polygon);
    marker_publisher_->publish(array);
  }

  void execute(const std::shared_ptr<PickHandle>& handle)
  {
    try {
      const auto goal = handle->get_goal();
      feedback(handle, "PLAN_OBSERVE", 0.05F, "Moving to oblique observation pose");
      if (!observe(goal->target)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "MOVE_OBSERVE", "oblique observation failed"); return;
      }
      if (cancel_.load()) { fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "OBSERVING", "canceled", true); return; }
      feedback(handle, "ESTIMATE_COARSE", 0.20F, "Fitting ground and coarse cube centre");
      const auto coarse = estimate(srv::EstimateCube::Request::COARSE, goal->target);
      if (!coarse->success) {
        fail(handle, Pick::Result::FAILURE_INCOMPLETE_INFORMATION, "ESTIMATE_POSE", coarse->detail); return;
      }
      Eigen::Vector3d up(coarse->ground_normal.x, coarse->ground_normal.y, coarse->ground_normal.z);
      Eigen::Vector3d coarse_center(coarse->center.point.x, coarse->center.point.y, coarse->center.point.z);
      feedback(handle, "MOVE_TOP_OBSERVE", 0.35F, "Moving RGB camera directly above coarse centre");
      if (!moveCameraTopDown(coarse_center, up.normalized())) {
        fail(handle, Pick::Result::FAILURE_THEORETICALLY_INFEASIBLE, "MOVE_TOP_OBSERVE", "top observation pose is not plannable"); return;
      }
      geometry_msgs::msg::PointStamped fine_hint = coarse->center;
      feedback(handle, "ESTIMATE_FINE", 0.50F, "Projecting mask contour onto known cube top plane");
      const auto fine = estimate(srv::EstimateCube::Request::FINE, fine_hint, up.normalized(), coarse->ground_offset);
      if (!fine->success) {
        fail(handle, Pick::Result::FAILURE_INCOMPLETE_INFORMATION, "ESTIMATE_POSE", fine->detail); return;
      }
      Eigen::Vector3d center(fine->center.point.x, fine->center.point.y, fine->center.point.z);
      Eigen::Vector3d edge(fine->edge_direction.x, fine->edge_direction.y, fine->edge_direction.z);
      up.normalize(); edge = (edge - edge.dot(up) * up).normalized();

      geometry_msgs::msg::Pose grasp, pregrasp;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool found = false;
      double selected_pregrasp_distance = 0.0;
      double selected_grasp_distance = 0.0;
      double selected_yaw_degrees = 0.0;
      double selected_tilt_degrees = 0.0;
      Eigen::Matrix3d selected_rotation = Eigen::Matrix3d::Identity();
      const Eigen::Vector3d top_center = center + 0.5 * cube_size_ * up;
      const auto target_objects = planning_scene_.getObjects({"yellow_cube", "observe_target"});
      const auto pregrasp_distances = descendingValues(
        pregrasp_distance_max_, pregrasp_distance_min_, pregrasp_distance_step_);
      std::vector<double> grasp_distances;
      for (double value = grasp_distance_min_;
        value <= grasp_distance_max_ + 1e-9; value += grasp_distance_step_)
      {
        grasp_distances.push_back(value);
      }
      for (const double pregrasp_distance : pregrasp_distances) {
        for (int quarter_turn : {0, 1, -1, 2}) {
          const Eigen::Vector3d x = Eigen::AngleAxisd(quarter_turn * M_PI_2, up) * edge;
          const Eigen::Vector3d z = -up;
          const Eigen::Vector3d y = z.cross(x).normalized();
          Eigen::Matrix3d vertical_rotation;
          vertical_rotation.col(0) = x; vertical_rotation.col(1) = y; vertical_rotation.col(2) = z;
          // Exact vertical alignment puts Joint4 almost at zero and makes the
          // numerical IK branch unstable. Two degrees is visually negligible
          // while keeping the wrist away from that singular configuration.
          for (const double tilt_deg : {2.0, -2.0, 4.0, -4.0}) {
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.linear() = vertical_rotation *
              Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
            transform.translation() = top_center + pregrasp_distance * up;
            pregrasp = poseMessage(transform);
            move_group_.setEndEffectorLink(tcp_frame_);
            move_group_.setStartStateToCurrentState();
            if (!move_group_.setJointValueTarget(pregrasp, tcp_frame_)) continue;
            if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) continue;
            if (!removeTargetCollision()) {
              fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "PLAN_PREGRASP",
                "target collision removal did not reach the planning scene");
              return;
            }
            for (const double grasp_distance : grasp_distances) {
              transform.translation() = top_center + grasp_distance * up;
              const auto candidate_grasp = poseMessage(transform);
              moveit_msgs::msg::RobotTrajectory candidate_descent;
              const double descent_fraction = computeCartesianFromPlanEnd(
                plan, candidate_grasp, true, candidate_descent);
              if (descent_fraction < minimum_fraction_) continue;
              grasp = candidate_grasp;
              selected_pregrasp_distance = pregrasp_distance;
              selected_grasp_distance = grasp_distance;
              selected_yaw_degrees = std::atan2(x.y(), x.x()) * 180.0 / M_PI;
              selected_tilt_degrees = tilt_deg;
              selected_rotation = transform.rotation();
              found = true;
              break;
            }
            if (!restoreTargetCollision(target_objects)) {
              fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "PLAN_PREGRASP",
                "target collision restoration did not reach the planning scene");
              return;
            }
            if (found) break;
          }
          if (found) break;
        }
        if (found) break;
      }
      if (!found) {
        fail(handle, Pick::Result::FAILURE_REPOSITION_REQUIRED, "PLAN_PREGRASP",
          "no pregrasp in +80..+15 mm has a feasible downstream grasp; reposition Go2");
        return;
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "Selected pregrasp: distance=%+.0f mm provisional_grasp=%+.0f mm yaw=%.1f deg tilt=%.1f deg",
        1000.0 * selected_pregrasp_distance, 1000.0 * selected_grasp_distance,
        selected_yaw_degrees, selected_tilt_degrees);
      publishMarkers(*fine, grasp, pregrasp);
      auto result = std::make_shared<Pick::Result>();
      result->class_name = "yellow_cube"; result->estimated_center = fine->center;
      result->grasp_pose.header.frame_id = planning_frame_; result->grasp_pose.pose = grasp;
      result->pregrasp_pose.header.frame_id = planning_frame_; result->pregrasp_pose.pose = pregrasp;
      result->pregrasp_distance_m = selected_pregrasp_distance;
      result->grasp_distance_m = selected_grasp_distance;
      result->grasp_yaw_degrees = selected_yaw_degrees;
      result->approach_tilt_degrees = selected_tilt_degrees;
      result->grasp_pose.header.stamp = result->pregrasp_pose.header.stamp = node_->now();
      if (goal->stop_after == Pick::Goal::COMPUTE_ONLY) {
        result->success = true; result->detail = "cube grasp pose computed and visualized";
        handle->succeed(result); busy_.store(false); return;
      }
      feedback(handle, "MOVE_PREGRASP", 0.65F, "Executing selected pregrasp plan");
      if (!commandGripper(gripper_open_) || move_group_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "MOVE_PREGRASP", "pregrasp execution failed"); return;
      }
      if (goal->stop_after == Pick::Goal::MOVE_PREGRASP) {
        result->success = true; result->detail = "pregrasp pose reached"; handle->succeed(result); busy_.store(false); return;
      }
      if (!removeTargetCollision()) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "DESCEND",
          "target collision removal did not reach the planning scene");
        return;
      }
      moveit_msgs::msg::RobotTrajectory descent_trajectory;
      bool grasp_found = false;
      for (const double grasp_distance : grasp_distances) {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() = selected_rotation;
        transform.translation() = top_center + grasp_distance * up;
        const auto candidate_grasp = poseMessage(transform);
        move_group_.setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory candidate_descent;
        const double fraction = move_group_.computeCartesianPath(
          {candidate_grasp}, cartesian_step_, 0.0, candidate_descent, true);
        if (fraction < minimum_fraction_) continue;
        grasp = candidate_grasp;
        selected_grasp_distance = grasp_distance;
        descent_trajectory = std::move(candidate_descent);
        grasp_found = true;
        break;
      }
      if (!grasp_found) {
        fail(handle, Pick::Result::FAILURE_REPOSITION_REQUIRED, "DESCEND",
          "no grasp in -40..-24 mm is reachable from the actual pregrasp; reposition Go2");
        return;
      }
      result->grasp_pose.pose = grasp;
      result->grasp_pose.header.stamp = node_->now();
      result->grasp_distance_m = selected_grasp_distance;
      RCLCPP_INFO(
        node_->get_logger(), "Confirmed grasp from live pregrasp: distance=%+.0f mm",
        1000.0 * selected_grasp_distance);
      feedback(handle, "DESCEND", 0.78F, "Descending along fitted ground normal");
      if (!executeTrajectory(descent_trajectory)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "DESCEND", "Cartesian descent failed"); return;
      }
      if (goal->stop_after == Pick::Goal::DESCEND) {
        result->success = true; result->detail = "grasp pose reached with gripper open"; handle->succeed(result); busy_.store(false); return;
      }
      feedback(handle, "GRASP", 0.88F, "Closing gripper");
      if (!commandGripper(gripper_closed_)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "GRASP", "gripper close failed"); return;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(grasp_settle_));
      feedback(handle, "LIFT", 0.95F, "Lifting along fitted ground normal");
      auto lift_trajectory = reverseLiftTrajectory(descent_trajectory, up);
      if (lift_trajectory.joint_trajectory.points.size() < 2 || !executeTrajectory(lift_trajectory)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "LIFT", "Cartesian lift failed"); return;
      }
      result->success = true; result->failure_category = Pick::Result::FAILURE_NONE;
      result->detail = "yellow_cube visually estimated, grasped and lifted";
      handle->succeed(result); busy_.store(false);
      RCLCPP_INFO(node_->get_logger(), "PICK STAGE SUCCEEDED: GRASP_AND_LIFT");
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "PickObject error: %s", error.what());
      fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "INTERNAL", error.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<Observe>::SharedPtr observe_client_;
  rclcpp::Client<srv::EstimateCube>::SharedPtr estimate_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp_action::Client<Gripper>::SharedPtr gripper_client_;
  rclcpp_action::Server<Pick>::SharedPtr server_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  std::atomic<bool> busy_{false}, cancel_{false};
  std::string action_name_, observe_name_, estimate_name_, planning_frame_;
  std::string link6_frame_, tcp_frame_, camera_frame_;
  std::vector<double> stowed_;
  std::vector<double> top_distances_, top_rolls_;
  double cube_size_{}, pregrasp_distance_max_{}, pregrasp_distance_min_{};
  double pregrasp_distance_step_{}, grasp_distance_min_{}, grasp_distance_max_{};
  double grasp_distance_step_{}, lift_distance_{}, camera_settle_{};
  double cartesian_step_{}, minimum_fraction_{}, gripper_open_{}, gripper_closed_{};
  double grasp_settle_{}, stowed_tolerance_{};
};
}  // namespace d1_manipulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("d1_pick_object", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<d1_manipulation::PickObjectServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node); executor.spin();
  server.reset(); rclcpp::shutdown(); return 0;
}
