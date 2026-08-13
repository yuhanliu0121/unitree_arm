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
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "d1_manipulation/action/observe_target.hpp"
#include "d1_manipulation/action/pick_object.hpp"
#include "d1_manipulation/pick_strategy.hpp"
#include "d1_manipulation/srv/detect_target.hpp"
#include "d1_manipulation/srv/verify_held_object.hpp"

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

class PickObjectServer : public PickStrategyRuntime
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
    detect_name_ = parameterOrDeclare(
      node_, "target_detection_service_name", std::string("/arm/perception/detect_target"));
    verify_name_ = parameterOrDeclare(
      node_, "held_object_verification_service_name",
      std::string("/arm/perception/verify_held_object"));
    planning_frame_ = parameterOrDeclare(node_, "planning_frame", std::string("base_link"));
    link6_frame_ = parameterOrDeclare(node_, "link6_frame", std::string("Link6"));
    tcp_frame_ = parameterOrDeclare(node_, "tcp_frame", std::string("tcp_link"));
    camera_frame_ = parameterOrDeclare(node_, "color_optical_frame", std::string("wrist_camera_color_optical_frame"));
    carry_ = parameterOrDeclare(
      node_, "carry_joint_positions", std::vector<double>{0, -1.5, 1.5, 0, -0.6, 0});
    top_distances_ = parameterOrDeclare(node_, "top_observation_distances_m", std::vector<double>{0.35, 0.40, 0.45, 0.50, 0.55, 0.60});
    top_rolls_ = parameterOrDeclare(node_, "top_observation_roll_degrees", std::vector<double>{0, 90, -90, 180, 45, -45, 135, -135});
    camera_settle_ = parameterOrDeclare(node_, "camera_settle_s", 0.5);
    cartesian_step_ = parameterOrDeclare(node_, "cartesian_step_m", 0.005);
    minimum_fraction_ = parameterOrDeclare(node_, "minimum_cartesian_fraction", 0.95);
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
    detect_client_ = node_->create_client<srv::DetectTarget>(detect_name_);
    verify_client_ = node_->create_client<srv::VerifyHeldObject>(verify_name_);
    gripper_client_ = rclcpp_action::create_client<Gripper>(node_, "/gripper_controller/gripper_cmd");
    strategies_.emplace(
      "yellow_cube", makeYellowCubePickStrategy(node_, *this));
    server_ = rclcpp_action::create_server<Pick>(
      node_, action_name_,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Pick::Goal> goal) {
        if (goal->target.header.frame_id.empty() ||
          goal->stop_after > Pick::Goal::GRASP_AND_CARRY) return rclcpp_action::GoalResponse::REJECT;
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) {
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

  std::string detectClass(const geometry_msgs::msg::PointStamped& hint, std::string& detail)
  {
    if (!detect_client_->wait_for_service(5s)) {
      detail = "target detection service unavailable";
      return {};
    }
    auto request = std::make_shared<srv::DetectTarget::Request>();
    request->target_hint = hint;
    auto future = detect_client_->async_send_request(request);
    if (future.wait_for(15s) != std::future_status::ready) {
      detail = "target detection timed out";
      return {};
    }
    const auto response = future.get();
    detail = response->detail;
    return response->success ? response->class_name : std::string{};
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

public:
  moveit::planning_interface::MoveGroupInterface& moveGroup() override { return move_group_; }
  moveit::planning_interface::PlanningSceneInterface& planningScene() override
  {
    return planning_scene_;
  }
  const std::string& planningFrame() const override { return planning_frame_; }
  const std::string& tcpFrame() const override { return tcp_frame_; }
  const std::string& link6Frame() const override { return link6_frame_; }
  double cartesianStep() const override { return cartesian_step_; }
  double minimumCartesianFraction() const override { return minimum_fraction_; }

  Eigen::Isometry3d lookup(
    const std::string& target, const std::string& source) override
  {
    return tf2::transformToEigen(tf_buffer_.lookupTransform(target, source, tf2::TimePointZero, 3s));
  }

  bool moveCameraTopDown(
    const Eigen::Vector3d& target, const Eigen::Vector3d& up) override
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

private:

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
    const moveit_msgs::msg::RobotTrajectory& descent, const Eigen::Vector3d& up,
    double lift_distance)
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
      if (height >= lift_distance - 1e-4) break;
    }
    return lift;
  }

  double computeCartesianFromPlanEnd(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const geometry_msgs::msg::Pose& target, bool avoid_collisions,
    moveit_msgs::msg::RobotTrajectory& trajectory) override
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

  bool removeTargetCollision(const std::vector<std::string>& target_ids) override
  {
    const auto known = planning_scene_.getKnownObjectNames();
    std::vector<std::string> present_targets;
    for (const auto& id : target_ids) {
      if (std::find(known.begin(), known.end(), id) != known.end()) {
        present_targets.push_back(id);
      }
    }
    if (!present_targets.empty()) {
      planning_scene_.removeCollisionObjects(present_targets);
    }
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

  bool applyEstimatedGround(
    const Eigen::Vector3d& raw_normal, double offset) override
  {
    Eigen::Vector3d normal = raw_normal.normalized();
    if (!normal.allFinite() || normal.norm() < 1e-6 || !std::isfinite(offset)) return false;

    constexpr double ground_thickness = 0.02;
    const Eigen::Vector3d point_on_surface = -offset * normal;
    const Eigen::Vector3d centre = point_on_surface - 0.5 * ground_thickness * normal;
    const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), normal);

    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = planning_frame_;
    ground.id = "ground";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {2.0, 2.0, ground_thickness};
    ground.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    pose.position.x = centre.x(); pose.position.y = centre.y(); pose.position.z = centre.z();
    pose.orientation.x = orientation.x(); pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z(); pose.orientation.w = orientation.w();
    ground.primitive_poses.push_back(pose);
    ground.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::ObjectColor ground_color;
    ground_color.id = ground.id;
    ground_color.color = color(1.0F, 0.45F, 0.05F, 1.0F);
    if (!planning_scene_.applyCollisionObjects({ground}, {ground_color})) return false;
    std::this_thread::sleep_for(300ms);
    return true;
  }

  bool attachEstimatedObject(
    PickStrategy& strategy, const PreparedPick& pick,
    geometry_msgs::msg::PointStamped& expected)
  {
    const auto attached = strategy.makeAttachedObject(pick, expected);
    if (!planning_scene_.applyAttachedCollisionObject(attached)) {
      return false;
    }
    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool moveToCarry()
  {
    move_group_.setStartStateToCurrentState();
    if (!move_group_.setJointValueTarget(carry_)) return false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool verifyHeldObject(const std::string& class_name,
    const geometry_msgs::msg::PointStamped& expected, std::string& detail)
  {
    if (!verify_client_->wait_for_service(5s)) {
      detail = "held-object verification service unavailable";
      return false;
    }
    auto request = std::make_shared<srv::VerifyHeldObject::Request>();
    request->class_name = class_name;
    request->expected_center = expected;
    auto future = verify_client_->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready) {
      detail = "held-object verification timed out";
      return false;
    }
    const auto response = future.get();
    detail = response->detail;
    return response->success && response->held;
  }

  bool restoreTargetCollision(
    const std::map<std::string, moveit_msgs::msg::CollisionObject>& target_objects) override
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

  void execute(const std::shared_ptr<PickHandle>& handle)
  {
    try {
      const auto goal = handle->get_goal();
      feedback(handle, "PLAN_OBSERVE", 0.05F, "Moving to oblique observation pose");
      if (!observe(goal->target)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "MOVE_OBSERVE", "oblique observation failed"); return;
      }
      if (cancel_.load()) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "OBSERVING", "canceled", true);
        return;
      }
      feedback(handle, "CLASSIFY_TARGET", 0.15F, "Selecting object-specific grasp strategy");
      std::string detection_detail;
      const std::string class_name = detectClass(goal->target, detection_detail);
      if (class_name.empty()) {
        fail(handle, Pick::Result::FAILURE_INCOMPLETE_INFORMATION,
          "CLASSIFY_TARGET", detection_detail);
        return;
      }
      const auto strategy_entry = strategies_.find(class_name);
      if (strategy_entry == strategies_.end()) {
        fail(handle, Pick::Result::FAILURE_THEORETICALLY_INFEASIBLE,
          "SELECT_STRATEGY", "no grasp strategy is registered for " + class_name);
        return;
      }
      PickStrategy& strategy = *strategy_entry->second;
      PreparedPick prepared;
      StrategyFailure strategy_failure;
      feedback(handle, "PREPARE_GRASP", 0.20F, "Running " + class_name + " grasp strategy");
      if (!strategy.prepare(goal->target, prepared, strategy_failure)) {
        fail(handle, strategy_failure.category, strategy_failure.state, strategy_failure.detail);
        return;
      }

      auto result = std::make_shared<Pick::Result>();
      result->class_name = prepared.class_name;
      result->estimated_center = prepared.estimated_center;
      result->grasp_pose.header.frame_id = planning_frame_;
      result->grasp_pose.pose = prepared.grasp_pose;
      result->pregrasp_pose.header.frame_id = planning_frame_;
      result->pregrasp_pose.pose = prepared.pregrasp_pose;
      result->pregrasp_distance_m = prepared.pregrasp_distance_m;
      result->grasp_distance_m = prepared.grasp_distance_m;
      result->grasp_yaw_degrees = prepared.grasp_yaw_degrees;
      result->approach_tilt_degrees = prepared.approach_tilt_degrees;
      result->grasp_pose.header.stamp = result->pregrasp_pose.header.stamp = node_->now();
      if (goal->stop_after == Pick::Goal::COMPUTE_ONLY) {
        result->success = true;
        result->detail = prepared.class_name + " grasp pose computed and visualized";
        handle->succeed(result); busy_.store(false); return;
      }
      feedback(handle, "MOVE_PREGRASP", 0.65F, "Executing selected pregrasp plan");
      if (!commandGripper(prepared.gripper_open_m) ||
        move_group_.execute(prepared.pregrasp_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "MOVE_PREGRASP", "pregrasp execution failed"); return;
      }
      if (goal->stop_after == Pick::Goal::MOVE_PREGRASP) {
        result->success = true; result->detail = "pregrasp pose reached"; handle->succeed(result); busy_.store(false); return;
      }
      if (!strategy.confirmDescent(prepared, strategy_failure)) {
        fail(handle, strategy_failure.category, strategy_failure.state, strategy_failure.detail);
        return;
      }
      result->grasp_pose.pose = prepared.grasp_pose;
      result->grasp_pose.header.stamp = node_->now();
      result->grasp_distance_m = prepared.grasp_distance_m;
      RCLCPP_INFO(
        node_->get_logger(), "Confirmed grasp from live pregrasp: distance=%+.0f mm",
        1000.0 * prepared.grasp_distance_m);
      feedback(handle, "DESCEND", 0.78F, "Executing strategy approach trajectory");
      if (!executeTrajectory(prepared.descent_trajectory)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "DESCEND", "Cartesian descent failed"); return;
      }
      if (goal->stop_after == Pick::Goal::DESCEND) {
        result->success = true; result->detail = "grasp pose reached with gripper open"; handle->succeed(result); busy_.store(false); return;
      }
      feedback(handle, "GRASP", 0.88F, "Closing gripper");
      if (!commandGripper(prepared.gripper_closed_m)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "GRASP", "gripper close failed"); return;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(prepared.grasp_settle_s));
      feedback(handle, "LIFT", 0.95F, "Reversing the strategy approach trajectory");
      auto lift_trajectory = reverseLiftTrajectory(
        prepared.descent_trajectory, prepared.lift_direction, prepared.lift_distance_m);
      if (lift_trajectory.joint_trajectory.points.size() < 2 || !executeTrajectory(lift_trajectory)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "LIFT", "Cartesian lift failed"); return;
      }
      if (goal->stop_after == Pick::Goal::GRASP_AND_LIFT) {
        result->success = true; result->failure_category = Pick::Result::FAILURE_NONE;
        result->detail = prepared.class_name + " visually estimated, grasped and lifted";
        handle->succeed(result); busy_.store(false);
        RCLCPP_INFO(node_->get_logger(), "PICK STAGE SUCCEEDED: GRASP_AND_LIFT");
        return;
      }
      feedback(handle, "ATTACH_OBJECT", 0.96F,
        "Attaching perception-estimated " + prepared.class_name + " to TCP");
      geometry_msgs::msg::PointStamped expected_held_center;
      if (!attachEstimatedObject(strategy, prepared, expected_held_center)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "ATTACH_OBJECT",
          "failed to attach perception-estimated object to TCP");
        return;
      }
      feedback(handle, "CARRY", 0.98F,
        "Moving grasped " + prepared.class_name + " to CARRY pose");
      if (!moveToCarry()) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "CARRY", "CARRY trajectory failed");
        return;
      }
      feedback(handle, "VERIFY_GRASP", 0.99F,
        "Checking " + prepared.class_name + " in wrist-camera ROI");
      std::string verification_detail;
      if (!verifyHeldObject(prepared.class_name, expected_held_center, verification_detail)) {
        fail(handle, Pick::Result::FAILURE_EXECUTION_ERROR, "VERIFY_GRASP", verification_detail);
        return;
      }
      result->success = true; result->failure_category = Pick::Result::FAILURE_NONE;
      result->detail = prepared.class_name + " grasped, carried and visually verified: " +
        verification_detail;
      handle->succeed(result); busy_.store(false);
      RCLCPP_INFO(node_->get_logger(), "PICK STAGE SUCCEEDED: GRASP_AND_CARRY");
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
  rclcpp::Client<srv::DetectTarget>::SharedPtr detect_client_;
  rclcpp::Client<srv::VerifyHeldObject>::SharedPtr verify_client_;
  rclcpp_action::Client<Gripper>::SharedPtr gripper_client_;
  rclcpp_action::Server<Pick>::SharedPtr server_;
  std::map<std::string, std::unique_ptr<PickStrategy>> strategies_;
  std::atomic<bool> busy_{false}, cancel_{false};
  std::string action_name_, observe_name_, detect_name_, verify_name_, planning_frame_;
  std::string link6_frame_, tcp_frame_, camera_frame_;
  std::vector<double> stowed_;
  std::vector<double> carry_;
  std::vector<double> top_distances_, top_rolls_;
  double camera_settle_{};
  double cartesian_step_{}, minimum_fraction_{}, stowed_tolerance_{};
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
