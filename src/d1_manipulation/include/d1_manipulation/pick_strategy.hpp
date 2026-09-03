#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace d1_manipulation
{

struct StrategyFailure
{
  uint8_t category{0};
  std::string state;
  std::string detail;
};

struct PickStrategyState
{
  virtual ~PickStrategyState() = default;
};

struct PreparedPick
{
  std::string class_name;
  geometry_msgs::msg::PointStamped estimated_center;
  geometry_msgs::msg::Pose grasp_pose;
  geometry_msgs::msg::Pose pregrasp_pose;
  moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
  moveit_msgs::msg::RobotTrajectory descent_trajectory;
  Eigen::Vector3d lift_direction{Eigen::Vector3d::UnitZ()};
  double pregrasp_distance_m{0.0};
  double grasp_distance_m{0.0};
  double grasp_yaw_degrees{0.0};
  double approach_tilt_degrees{0.0};
  double gripper_open_m{0.03};
  double gripper_closed_m{0.0};
  double gripper_held_threshold_m{0.0};
  double grasp_settle_s{0.5};
  double lift_distance_m{0.10};

  // Opaque state used only by the strategy that prepared this pick.
  std::shared_ptr<PickStrategyState> strategy_state;
};

class PickStrategyRuntime
{
public:
  virtual ~PickStrategyRuntime() = default;
  virtual moveit::planning_interface::MoveGroupInterface& moveGroup() = 0;
  virtual moveit::planning_interface::PlanningSceneInterface& planningScene() = 0;
  virtual const std::string& planningFrame() const = 0;
  virtual const std::string& tcpFrame() const = 0;
  virtual const std::string& link6Frame() const = 0;
  virtual const std::string& cameraFrame() const = 0;
  virtual Eigen::Vector3d gravityUp() = 0;
  virtual Eigen::Isometry3d lookup(const std::string& target, const std::string& source) = 0;
  virtual bool moveCameraTopDown(
    const Eigen::Vector3d& target, const Eigen::Vector3d& up,
    StrategyFailure& failure) = 0;
  virtual bool applyEstimatedGround(const Eigen::Vector3d& normal, double offset) = 0;
  virtual bool removeTargetCollision(const std::vector<std::string>& ids) = 0;
  virtual bool restoreTargetCollision(
    const std::map<std::string, moveit_msgs::msg::CollisionObject>& objects) = 0;
  virtual double computeCartesianFromPlanEnd(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const geometry_msgs::msg::Pose& target,
    bool avoid_collisions,
    moveit_msgs::msg::RobotTrajectory& trajectory) = 0;
  virtual double cartesianStep() const = 0;
  virtual double minimumCartesianFraction() const = 0;
  virtual bool executePlan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;
  virtual bool executeFineTunePlan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;
};

class PickStrategy
{
public:
  virtual ~PickStrategy() = default;
  virtual const std::string& className() const = 0;
  virtual bool prepare(
    const geometry_msgs::msg::PointStamped& coarse_hint,
    PreparedPick& output,
    StrategyFailure& failure) = 0;
  virtual bool fineTune(PreparedPick&, StrategyFailure&) {return true;}
  virtual bool confirmDescent(PreparedPick& plan, StrategyFailure& failure) = 0;
  virtual moveit_msgs::msg::AttachedCollisionObject makeAttachedObject(
    const PreparedPick& plan,
    geometry_msgs::msg::PointStamped& expected_center) = 0;
};

std::unique_ptr<PickStrategy> makeYellowCubePickStrategy(
  const rclcpp::Node::SharedPtr& node,
  PickStrategyRuntime& runtime);

std::unique_ptr<PickStrategy> makeZucchiniPickStrategy(
  const rclcpp::Node::SharedPtr& node,
  PickStrategyRuntime& runtime);

std::unique_ptr<PickStrategy> makeBowlPickStrategy(
  const rclcpp::Node::SharedPtr& node,
  PickStrategyRuntime& runtime);

}  // namespace d1_manipulation
