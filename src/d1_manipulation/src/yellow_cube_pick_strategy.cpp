#include "d1_manipulation/pick_strategy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <Eigen/SVD>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "d1_interfaces/action/pick_object.hpp"
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

bool jointMotionCost(
  const std::vector<std::string>& joint_names,
  const std::vector<double>& start,
  const std::vector<double>& goal,
  double joint5_weight, double& cost)
{
  if (start.size() != goal.size() || start.size() != joint_names.size()) return false;

  cost = 0.0;
  for (std::size_t index = 0; index < goal.size(); ++index) {
    const double delta = goal[index] - start[index];
    const double weight = joint_names[index] == "Joint5" ? joint5_weight : 1.0;
    cost += weight * delta * delta;
  }
  return true;
}

struct CubeCandidate
{
  geometry_msgs::msg::Pose pregrasp_pose;
  geometry_msgs::msg::Pose grasp_pose;
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  std::vector<double> joint_target;
  double pregrasp_distance{0.0};
  double grasp_distance{0.0};
  double yaw_degrees{0.0};
  double tilt_degrees{0.0};
  double motion_cost{std::numeric_limits<double>::infinity()};
  bool grasp_feasible{false};
};

struct YellowCubeState final : PickStrategyState
{
  Eigen::Vector3d top_center{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d grasp_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d object_rotation{Eigen::Matrix3d::Identity()};
  std::vector<double> grasp_distances;
  std::vector<std::string> target_collision_ids;
};

struct FineTuneMeasurement
{
  Eigen::Vector3d top_center_camera{Eigen::Vector3d::Zero()};
  Eigen::Vector3d top_center_planning{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ground_normal{Eigen::Vector3d::UnitZ()};
  double ground_offset{0.0};
  double closing_coordinate{0.0};
  double finger_coordinate{0.0};
  double closing_stddev{0.0};
  double finger_stddev{0.0};
};
}  // namespace

class YellowCubePickStrategy final : public PickStrategy
{
public:
  YellowCubePickStrategy(const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
  : node_(node), runtime_(runtime)
  {
    estimate_name_ = parameterOrDeclare(
      node_, "cube_estimation_service_name", std::string("/arm/perception/estimate_cube"));
    cube_size_ = parameterOrDeclare(node_, "cube_size_m", 0.05);
    pregrasp_max_ = parameterOrDeclare(node_, "pregrasp_distance_max_m", 0.080);
    pregrasp_min_ = parameterOrDeclare(node_, "pregrasp_distance_min_m", 0.015);
    pregrasp_step_ = parameterOrDeclare(node_, "pregrasp_distance_step_m", 0.005);
    grasp_min_ = parameterOrDeclare(node_, "grasp_distance_min_m", -0.040);
    grasp_max_ = parameterOrDeclare(node_, "grasp_distance_max_m", -0.024);
    grasp_step_ = parameterOrDeclare(node_, "grasp_distance_step_m", 0.002);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_closed_ = parameterOrDeclare(node_, "cube_gripper_closed_m", 0.0216666667);
    gripper_held_threshold_ = parameterOrDeclare(
      node_, "cube_gripper_held_threshold_m", 0.0225);
    grasp_settle_ = parameterOrDeclare(node_, "grasp_settle_s", 0.5);
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    joint5_motion_weight_ = parameterOrDeclare(node_, "cube_joint5_motion_weight", 2.0);
    finetune_enabled_ = parameterOrDeclare(node_, "cube_finetune_enabled", true);
    finetune_camera_frame_ = parameterOrDeclare(
      node_, "cube_finetune_camera_frame", std::string{"wrist_camera_color_optical_frame"});
    const auto closing_axis = parameterOrDeclare(
      node_, "cube_finetune_closing_axis_camera", std::vector<double>{-1.0, 0.0, 0.0});
    const auto finger_axis = parameterOrDeclare(
      node_, "cube_finetune_finger_axis_camera", std::vector<double>{0.0, 1.0, 0.0});
    const auto closing_bounds = parameterOrDeclare(
      node_, "cube_finetune_closing_bounds_m", std::vector<double>{-0.01, 0.01});
    const auto finger_bounds = parameterOrDeclare(
      node_, "cube_finetune_finger_bounds_m", std::vector<double>{-0.02, 0.02});
    if (closing_axis.size() != 3 || finger_axis.size() != 3 ||
      closing_bounds.size() != 2 || finger_bounds.size() != 2)
    {
      throw std::invalid_argument("cube finetune axes must have 3 values and bounds 2 values");
    }
    closing_axis_camera_ = Eigen::Vector3d(closing_axis[0], closing_axis[1], closing_axis[2]);
    finger_axis_camera_ = Eigen::Vector3d(finger_axis[0], finger_axis[1], finger_axis[2]);
    closing_bounds_ = Eigen::Vector2d(closing_bounds[0], closing_bounds[1]);
    finger_bounds_ = Eigen::Vector2d(finger_bounds[0], finger_bounds[1]);
    finetune_gain_ = parameterOrDeclare(node_, "cube_finetune_gain", 0.7);
    finetune_max_step_ = parameterOrDeclare(node_, "cube_finetune_max_step_m", 0.008);
    finetune_max_total_ = parameterOrDeclare(node_, "cube_finetune_max_total_m", 0.020);
    finetune_max_corrections_ = parameterOrDeclare(node_, "cube_finetune_max_corrections", 3);
    finetune_samples_ = parameterOrDeclare(node_, "cube_finetune_samples", 3);
    finetune_max_stddev_ = parameterOrDeclare(node_, "cube_finetune_max_stddev_m", 0.002);
    finetune_settle_s_ = parameterOrDeclare(node_, "cube_finetune_settle_s", 0.5);
    if (cube_size_ <= 0.0 || pregrasp_min_ < 0.0 || pregrasp_max_ < pregrasp_min_ ||
      pregrasp_step_ <= 0.0 || grasp_min_ > grasp_max_ || grasp_max_ >= 0.0 ||
      grasp_step_ <= 0.0 || gripper_closed_ < 0.0 || joint5_motion_weight_ <= 0.0 ||
      gripper_held_threshold_ <= gripper_closed_ || gripper_held_threshold_ > gripper_open_)
    {
      throw std::invalid_argument("invalid signed cube pregrasp/grasp search parameters");
    }
    if (closing_axis_camera_.norm() < 0.9 || finger_axis_camera_.norm() < 0.9 ||
      closing_bounds_.x() >= closing_bounds_.y() || finger_bounds_.x() >= finger_bounds_.y() ||
      finetune_gain_ <= 0.0 || finetune_gain_ > 1.0 || finetune_max_step_ <= 0.0 ||
      finetune_max_total_ < finetune_max_step_ || finetune_max_corrections_ < 0 ||
      finetune_samples_ < 1 || finetune_max_stddev_ <= 0.0 ||
      finetune_settle_s_ < 0.0)
    {
      throw std::invalid_argument("invalid cube finetune parameters");
    }
    closing_axis_camera_.normalize();
    finger_axis_camera_ -= closing_axis_camera_ *
      closing_axis_camera_.dot(finger_axis_camera_);
    if (finger_axis_camera_.norm() < 0.9) {
      throw std::invalid_argument("cube finetune camera axes are nearly collinear");
    }
    finger_axis_camera_.normalize();
    estimate_client_ = node_->create_client<srv::EstimateCube>(estimate_name_);
    state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity");
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm/debug/cube_grasp_markers", rclcpp::QoS(1).transient_local().reliable());
  }

  const std::string& className() const override { return class_name_; }

  bool prepare(const geometry_msgs::msg::PointStamped& hint, PreparedPick& output,
    StrategyFailure& failure) override
  {
    const auto coarse = estimate(srv::EstimateCube::Request::COARSE, hint);
    if (!coarse || !coarse->success) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", coarse ? coarse->detail : "cube coarse estimation unavailable"};
      return false;
    }
    Eigen::Vector3d surface_up(
      coarse->ground_normal.x, coarse->ground_normal.y, coarse->ground_normal.z);
    surface_up.normalize();
    const Eigen::Vector3d gravity_up = runtime_.gravityUp();
    Eigen::Vector3d coarse_center(
      coarse->center.point.x, coarse->center.point.y, coarse->center.point.z);
    if (!runtime_.applyEstimatedGround(surface_up, coarse->ground_offset)) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply perception-fitted ground to MoveIt"};
      return false;
    }
    if (!runtime_.moveCameraTopDown(coarse_center, gravity_up, failure)) {
      return false;
    }
    const auto fine = estimate(
      srv::EstimateCube::Request::FINE, coarse->center, surface_up, coarse->ground_offset);
    if (!fine || !fine->success) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", fine ? fine->detail : "cube fine estimation unavailable"};
      return false;
    }

    surface_up = Eigen::Vector3d(
      fine->ground_normal.x, fine->ground_normal.y, fine->ground_normal.z);
    if (!surface_up.allFinite() || surface_up.norm() < 0.9) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "cube fine ground normal is invalid"};
      return false;
    }
    surface_up.normalize();
    if (!runtime_.applyEstimatedGround(surface_up, fine->ground_offset)) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply fine-observation ground to MoveIt"};
      return false;
    }

    Eigen::Vector3d center(fine->center.point.x, fine->center.point.y, fine->center.point.z);
    Eigen::Vector3d object_edge(
      fine->edge_direction.x, fine->edge_direction.y, fine->edge_direction.z);
    object_edge = (object_edge - object_edge.dot(surface_up) * surface_up).normalized();
    Eigen::Vector3d grasp_edge =
      object_edge - object_edge.dot(gravity_up) * gravity_up;
    if (!object_edge.allFinite() || grasp_edge.norm() < 1e-6) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "cube edge is invalid after gravity projection"};
      return false;
    }
    grasp_edge.normalize();
    const Eigen::Vector3d top_center = center + 0.5 * cube_size_ * surface_up;
    const double surface_gravity_angle_deg = std::acos(std::clamp(
      surface_up.dot(gravity_up), -1.0, 1.0)) * 180.0 / M_PI;
    RCLCPP_INFO(node_->get_logger(),
      "Cube orientation references: surface_up=(%.3f, %.3f, %.3f) "
      "gravity_up=(%.3f, %.3f, %.3f) separation=%.2f deg; tool axis uses gravity",
      surface_up.x(), surface_up.y(), surface_up.z(),
      gravity_up.x(), gravity_up.y(), gravity_up.z(), surface_gravity_angle_deg);
    const std::vector<std::string> target_ids{"yellow_cube", "observe_target"};
    const auto target_objects = runtime_.planningScene().getObjects(target_ids);
    const auto pregrasp_distances = descending(pregrasp_max_, pregrasp_min_, pregrasp_step_);
    const auto grasp_distances = ascending(grasp_min_, grasp_max_, grasp_step_);

    CubeCandidate selected;
    moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
    std::size_t selected_ik_candidates = 0;
    std::size_t selected_descent_candidates = 0;
    std::size_t full_plan_attempts = 0;
    bool found = false;
    auto& move_group = runtime_.moveGroup();
    move_group.setEndEffectorLink(runtime_.tcpFrame());
    for (const double pregrasp_distance : pregrasp_distances) {
      const auto current_state = move_group.getCurrentState();
      if (!current_state) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
          "PLAN_PREGRASP", "current robot state is unavailable for cube candidate ranking"};
        return false;
      }
      const auto* joint_model_group = current_state->getJointModelGroup(move_group.getName());
      if (!joint_model_group) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "PLAN_PREGRASP", "MoveIt arm joint model group is unavailable"};
        return false;
      }
      std::vector<double> current_joints;
      current_state->copyJointGroupPositions(joint_model_group, current_joints);
      const auto joint_names = joint_model_group->getVariableNames();
      std::vector<CubeCandidate> candidates;

      // Stage 1: generate IK endpoints only. This avoids publishing and solving a
      // complete OMPL path for every cube-symmetric yaw/tilt candidate.
      for (const int quarter_turn : {0, 1, -1, 2}) {
        const Eigen::Vector3d x =
          Eigen::AngleAxisd(quarter_turn * M_PI_2, gravity_up) * grasp_edge;
        const Eigen::Vector3d z = -gravity_up;
        const Eigen::Vector3d y = z.cross(x).normalized();
        Eigen::Matrix3d vertical;
        vertical.col(0) = x; vertical.col(1) = y; vertical.col(2) = z;
        // A tilted approach sweeps the fingers laterally during descent.  The
        // 50 mm cube leaves only about 10 mm of total clearance in the open
        // gripper, so even four degrees over the full approach can hit the top
        // face.  Cube grasps therefore require a strictly gravity-aligned tool
        // axis; if that endpoint is infeasible the caller must reposition.
        for (const double tilt_deg : {0.0}) {
          Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
          transform.linear() = vertical *
            Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
          transform.translation() = top_center + pregrasp_distance * gravity_up;
          const auto pregrasp = poseMessage(transform);
          move_group.setStartStateToCurrentState();
          if (!move_group.setJointValueTarget(pregrasp, runtime_.tcpFrame())) continue;
          std::vector<double> joint_target;
          move_group.getJointValueTarget(joint_target);
          auto endpoint_state = *current_state;
          endpoint_state.setJointGroupPositions(joint_model_group, joint_target);
          endpoint_state.update();
          if (!endpoint_state.satisfiesBounds(joint_model_group)) continue;
          double motion_cost = 0.0;
          if (!jointMotionCost(
              joint_names, current_joints, joint_target, joint5_motion_weight_, motion_cost))
          {
            continue;
          }
          CubeCandidate candidate;
          candidate.pregrasp_pose = pregrasp;
          candidate.rotation = transform.rotation();
          candidate.joint_target = joint_target;
          candidate.pregrasp_distance = pregrasp_distance;
          candidate.yaw_degrees = std::atan2(x.y(), x.x()) * 180.0 / M_PI;
          candidate.tilt_degrees = tilt_deg;
          candidate.motion_cost = motion_cost;
          candidates.push_back(std::move(candidate));
        }
      }

      // Validate downstream grasp depth from each IK endpoint without first
      // computing an OMPL path to that endpoint.
      if (!runtime_.removeTargetCollision(target_ids)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "PLAN_PREGRASP", "target collision removal did not reach the planning scene"};
        return false;
      }
      for (auto& candidate : candidates) {
        auto endpoint_state = *current_state;
        endpoint_state.setJointGroupPositions(joint_model_group, candidate.joint_target);
        endpoint_state.update();
        move_group.setStartState(endpoint_state);
        for (const double grasp_distance : grasp_distances) {
          Eigen::Isometry3d grasp_transform = Eigen::Isometry3d::Identity();
          grasp_transform.linear() = candidate.rotation;
          grasp_transform.translation() = top_center + grasp_distance * gravity_up;
          moveit_msgs::msg::RobotTrajectory descent;
          const auto grasp_pose = poseMessage(grasp_transform);
          const double fraction = move_group.computeCartesianPath(
            {grasp_pose}, runtime_.cartesianStep(), 0.0, descent, true);
          if (fraction < runtime_.minimumCartesianFraction()) continue;
          candidate.grasp_pose = grasp_pose;
          candidate.grasp_distance = grasp_distance;
          candidate.grasp_feasible = true;
          break;
        }
      }
      if (!runtime_.restoreTargetCollision(target_objects)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "PLAN_PREGRASP", "target collision restoration did not reach the planning scene"};
        return false;
      }

      const std::size_t ik_candidates = candidates.size();
      candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(), [](const CubeCandidate& candidate) {
          return !candidate.grasp_feasible;
        }),
        candidates.end());
      const std::size_t descent_candidates = candidates.size();
      std::stable_sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        constexpr double kTolerance = 1e-9;
        if (std::abs(lhs.grasp_distance - rhs.grasp_distance) > kTolerance) {
          return lhs.grasp_distance < rhs.grasp_distance;
        }
        return lhs.motion_cost < rhs.motion_cost;
      });

      RCLCPP_INFO(node_->get_logger(),
        "Cube candidate ranking: pregrasp=%+.0f mm feasible=%zu/%zu; "
        "policy=deepest_grasp_first, then minimum_weighted_joint_motion (Joint5 weight %.1f)",
        1000.0 * pregrasp_distance, descent_candidates, ik_candidates,
        joint5_motion_weight_);

      // Stage 2: solve and publish complete paths only in ranked order. Stop at
      // the first candidate whose pregrasp path and downstream descent both pass.
      for (std::size_t rank = 0; rank < candidates.size(); ++rank) {
        const auto& candidate = candidates[rank];
        ++full_plan_attempts;
        RCLCPP_INFO(node_->get_logger(),
          "Cube candidate attempt rank=%zu/%zu pregrasp=%+.0f mm grasp=%+.0f mm "
          "yaw=%.1f deg tilt=%.1f deg joint_motion_cost=%.4f",
          rank + 1, candidates.size(), 1000.0 * candidate.pregrasp_distance,
          1000.0 * candidate.grasp_distance, candidate.yaw_degrees,
          candidate.tilt_degrees, candidate.motion_cost);
        move_group.setStartStateToCurrentState();
        if (!move_group.setJointValueTarget(candidate.joint_target)) {
          RCLCPP_WARN(node_->get_logger(),
            "Cube candidate rejected rank=%zu reason=JOINT_TARGET_REJECTED", rank + 1);
          continue;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(node_->get_logger(),
            "Cube candidate rejected rank=%zu reason=PREGRASP_OMPL_PLAN_FAILED", rank + 1);
          continue;
        }
        if (!runtime_.removeTargetCollision(target_ids)) {
          failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
            "PLAN_PREGRASP", "target collision removal did not reach the planning scene"};
          return false;
        }
        moveit_msgs::msg::RobotTrajectory verified_descent;
        const double fraction = runtime_.computeCartesianFromPlanEnd(
          plan, candidate.grasp_pose, true, verified_descent);
        if (!runtime_.restoreTargetCollision(target_objects)) {
          failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
            "PLAN_PREGRASP", "target collision restoration did not reach the planning scene"};
          return false;
        }
        if (fraction < runtime_.minimumCartesianFraction()) {
          RCLCPP_WARN(node_->get_logger(),
            "Cube candidate rejected rank=%zu reason=PLANNED_ENDPOINT_DESCENT_INCOMPLETE "
            "fraction=%.1f%% required=%.1f%%",
            rank + 1, 100.0 * fraction, 100.0 * runtime_.minimumCartesianFraction());
          continue;
        }
        selected = candidate;
        selected_plan = std::move(plan);
        selected_ik_candidates = ik_candidates;
        selected_descent_candidates = descent_candidates;
        found = true;
        break;
      }
      if (found) break;
    }
    if (!found) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
        "PLAN_PREGRASP", "no cube pregrasp has a feasible downstream grasp; reposition Go2"};
      return false;
    }

    output.class_name = class_name_;
    output.estimated_center = fine->center;
    output.grasp_pose = selected.grasp_pose;
    output.pregrasp_pose = selected.pregrasp_pose;
    output.pregrasp_plan = std::move(selected_plan);
    output.lift_direction = gravity_up;
    output.pregrasp_distance_m = selected.pregrasp_distance;
    output.grasp_distance_m = selected.grasp_distance;
    output.grasp_yaw_degrees = selected.yaw_degrees;
    output.approach_tilt_degrees = selected.tilt_degrees;
    output.gripper_open_m = gripper_open_;
    output.gripper_closed_m = gripper_closed_;
    output.gripper_held_threshold_m = gripper_held_threshold_;
    output.grasp_settle_s = grasp_settle_;
    output.lift_distance_m = lift_distance_;
    auto state = std::make_shared<YellowCubeState>();
    state->top_center = top_center;
    state->grasp_rotation = selected.rotation;
    state->object_rotation.col(0) = object_edge;
    state->object_rotation.col(2) = surface_up;
    state->object_rotation.col(1) = surface_up.cross(object_edge).normalized();
    state->grasp_distances = grasp_distances;
    state->target_collision_ids = target_ids;
    output.strategy_state = std::move(state);
    publishMarkers(*coarse, *fine, output);
    RCLCPP_INFO(node_->get_logger(),
      "Cube strategy selected pregrasp=%+.0f mm grasp=%+.0f mm yaw=%.1f deg tilt=%.1f deg "
      "joint_motion_cost=%.4f IK_candidates=%zu descent_candidates=%zu full_plan_attempts=%zu "
      "reason=first complete plan in depth-first/joint-motion-ranked candidates",
      1000.0 * selected.pregrasp_distance, 1000.0 * selected.grasp_distance,
      selected.yaw_degrees, selected.tilt_degrees, selected.motion_cost,
      selected_ik_candidates, selected_descent_candidates, full_plan_attempts);
    return true;
  }

  bool fineTune(PreparedPick& plan, StrategyFailure& failure) override
  {
    if (!finetune_enabled_) {
      RCLCPP_INFO(node_->get_logger(),
        "Cube FINETUNE_GRASP is disabled; retaining the prepared pregrasp");
      return true;
    }
    if (runtime_.cameraFrame() != finetune_camera_frame_) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "FINETUNE_GRASP", "configured camera frame does not match the calibrated safe region"};
      return false;
    }
    const auto state = std::dynamic_pointer_cast<YellowCubeState>(plan.strategy_state);
    if (!state) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "FINETUNE_GRASP", "yellow-cube strategy state is missing"};
      return false;
    }

    const double closing_midpoint = 0.5 * (closing_bounds_.x() + closing_bounds_.y());
    const double finger_midpoint = 0.5 * (finger_bounds_.x() + finger_bounds_.y());
    // Keep every stop-and-look correction tied to the gravity-aligned
    // PREGRASP orientation selected by prepare().  Reusing getCurrentPose()'s
    // orientation would bake each endpoint residual into the next correction
    // and allow the tool axis to drift over repeated corrections.
    const auto pregrasp_orientation = plan.pregrasp_pose.orientation;
    double cumulative_correction = 0.0;
    for (int correction = 0; correction <= finetune_max_corrections_; ++correction) {
      FineTuneMeasurement measurement;
      std::string measurement_error;
      if (!measureFineTune(plan, measurement, measurement_error)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
          "FINETUNE_GRASP", measurement_error};
        return false;
      }
      publishFineTuneMarkers(measurement);

      const double closing_outside = intervalError(
        measurement.closing_coordinate, closing_bounds_);
      const double finger_outside = intervalError(
        measurement.finger_coordinate, finger_bounds_);
      const double outside_error = std::hypot(closing_outside, finger_outside);
      RCLCPP_INFO(node_->get_logger(),
        "Cube FINETUNE_GRASP observation %d/%d: closing=%.1f mm [%.1f, %.1f] "
        "finger=%.1f mm [%.1f, %.1f] std=(%.2f, %.2f) mm outside=%.2f mm",
        correction + 1, finetune_max_corrections_ + 1,
        1000.0 * measurement.closing_coordinate,
        1000.0 * closing_bounds_.x(), 1000.0 * closing_bounds_.y(),
        1000.0 * measurement.finger_coordinate,
        1000.0 * finger_bounds_.x(), 1000.0 * finger_bounds_.y(),
        1000.0 * measurement.closing_stddev,
        1000.0 * measurement.finger_stddev, 1000.0 * outside_error);

      if (measurement.closing_stddev > finetune_max_stddev_ ||
        measurement.finger_stddev > finetune_max_stddev_)
      {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(1)
               << "cube top centre is unstable across " << finetune_samples_
               << " frames: closing_std=" << 1000.0 * measurement.closing_stddev
               << " mm finger_std=" << 1000.0 * measurement.finger_stddev
               << " mm; keep PREGRASP and reposition Go2";
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
          "FINETUNE_GRASP", detail.str()};
        return false;
      }

      if (outside_error <= 1e-9) {
        if (!acceptFineTune(plan, *state, measurement)) {
          failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
            "FINETUNE_GRASP", "failed to update MoveIt with the refreshed ground plane"};
          return false;
        }
        RCLCPP_INFO(node_->get_logger(),
          "Cube FINETUNE_GRASP accepted inside calibrated safe prism after %d correction(s)",
          correction);
        return true;
      }
      if (correction == finetune_max_corrections_) break;

      Eigen::Vector3d correction_camera = finetune_gain_ * (
        (measurement.closing_coordinate - closing_midpoint) * closing_axis_camera_ +
        (measurement.finger_coordinate - finger_midpoint) * finger_axis_camera_);
      if (correction_camera.norm() > finetune_max_step_) {
        correction_camera *= finetune_max_step_ / correction_camera.norm();
      }
      const double remaining = finetune_max_total_ - cumulative_correction;
      if (remaining <= 1e-6) break;
      if (correction_camera.norm() > remaining) {
        correction_camera *= remaining / correction_camera.norm();
      }

      Eigen::Vector3d correction_planning =
        runtime_.lookup(runtime_.planningFrame(), runtime_.cameraFrame()).linear() *
        correction_camera;
      const Eigen::Vector3d gravity_up = runtime_.gravityUp();
      correction_planning -= gravity_up * correction_planning.dot(gravity_up);
      const double correction_norm = correction_planning.norm();
      if (correction_norm < 1e-5) break;

      auto& move_group = runtime_.moveGroup();
      move_group.setEndEffectorLink(runtime_.tcpFrame());
      auto target = move_group.getCurrentPose(runtime_.tcpFrame()).pose;
      target.position.x += correction_planning.x();
      target.position.y += correction_planning.y();
      target.position.z += correction_planning.z();
      target.orientation = pregrasp_orientation;
      move_group.setStartStateToCurrentState();
      if (!move_group.setPoseTarget(target, runtime_.tcpFrame())) {
        move_group.clearPoseTargets();
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
          "FINETUNE_GRASP", "visual correction endpoint has no IK solution; reposition Go2"};
        return false;
      }
      moveit::planning_interface::MoveGroupInterface::Plan correction_plan;
      const bool planned =
        move_group.plan(correction_plan) == moveit::core::MoveItErrorCode::SUCCESS;
      move_group.clearPoseTargets();
      if (!planned) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
          "FINETUNE_GRASP", "visual correction is not collision-free/plannable; reposition Go2"};
        return false;
      }
      RCLCPP_INFO(node_->get_logger(),
        "Cube FINETUNE_GRASP correction %d: camera_delta=(%+.1f, %+.1f, %+.1f) mm "
        "planning_horizontal_delta=(%+.1f, %+.1f, %+.1f) mm",
        correction + 1, 1000.0 * correction_camera.x(),
        1000.0 * correction_camera.y(), 1000.0 * correction_camera.z(),
        1000.0 * correction_planning.x(), 1000.0 * correction_planning.y(),
        1000.0 * correction_planning.z());
      if (!runtime_.executeFineTunePlan(correction_plan)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "FINETUNE_GRASP", "visual correction execution failed; motion stopped"};
        return false;
      }
      cumulative_correction += correction_norm;
      std::this_thread::sleep_for(std::chrono::duration<double>(finetune_settle_s_));
    }

    std::ostringstream detail;
    detail << "cube top centre remains outside the calibrated safe prism after "
           << finetune_max_corrections_
           << " corrections; keep PREGRASP and reposition Go2";
    failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
      "FINETUNE_GRASP", detail.str()};
    return false;
  }

  bool confirmDescent(PreparedPick& plan, StrategyFailure& failure) override
  {
    const auto state = std::dynamic_pointer_cast<YellowCubeState>(plan.strategy_state);
    if (!state) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "yellow-cube strategy state is missing"};
      return false;
    }
    if (!runtime_.removeTargetCollision(state->target_collision_ids)) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "target collision removal did not reach the planning scene"};
      return false;
    }
    auto& move_group = runtime_.moveGroup();
    const auto descent_start = move_group.getCurrentState(2.0);
    if (!descent_start) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "DESCEND", "current robot state is unavailable at the actual pregrasp"};
      return false;
    }
    const auto* arm_group = descent_start->getJointModelGroup(move_group.getName());
    if (!arm_group) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "MoveIt arm joint model group is unavailable at pregrasp"};
      return false;
    }
    std::vector<double> start_joints;
    descent_start->copyJointGroupPositions(arm_group, start_joints);
    std::ostringstream start_joint_text;
    start_joint_text << std::fixed << std::setprecision(2) << '[';
    for (std::size_t index = 0; index < start_joints.size(); ++index) {
      if (index > 0) start_joint_text << ", ";
      start_joint_text << start_joints[index] * 180.0 / M_PI;
    }
    start_joint_text << ']';
    RCLCPP_INFO(node_->get_logger(),
      "Cube live descent start snapshot: joints_deg=%s; all depth candidates reuse this state",
      start_joint_text.str().c_str());

    bool diagnosed_preferred_depth = false;
    std::size_t rejected_depths = 0;
    for (const double distance : state->grasp_distances) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.linear() = state->grasp_rotation;
      transform.translation() = state->top_center + distance * plan.lift_direction;
      const auto grasp = poseMessage(transform);
      move_group.setStartState(*descent_start);
      moveit_msgs::msg::RobotTrajectory descent;
      const double fraction = move_group.computeCartesianPath(
        {grasp}, runtime_.cartesianStep(), 0.0, descent, true);
      if (fraction < runtime_.minimumCartesianFraction()) {
        RCLCPP_WARN(node_->get_logger(),
          "Cube live descent rejected: target=%+.0f mm fraction=%.1f%% reached_depth=%+.1f mm",
          1000.0 * distance, 100.0 * fraction,
          1000.0 * (plan.pregrasp_distance_m +
          fraction * (distance - plan.pregrasp_distance_m)));
        if (!diagnosed_preferred_depth) {
          diagnoseCartesianFailure(
            *descent_start, grasp, fraction, plan.pregrasp_distance_m, distance);
          diagnosed_preferred_depth = true;
        }
        ++rejected_depths;
        continue;
      }
      RCLCPP_INFO(node_->get_logger(),
        "Cube live descent selected: target=%+.0f mm fraction=%.1f%% "
        "after_rejecting_deeper_targets=%zu reason=first_complete_live_cartesian_path",
        1000.0 * distance, 100.0 * fraction, rejected_depths);
      plan.grasp_pose = grasp;
      plan.grasp_distance_m = distance;
      plan.descent_trajectory = std::move(descent);
      return true;
    }
    failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
      "DESCEND", "no cube grasp is reachable from the actual pregrasp; reposition Go2"};
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject makeAttachedObject(
    const PreparedPick& plan, geometry_msgs::msg::PointStamped& expected) override
  {
    const auto state = std::dynamic_pointer_cast<YellowCubeState>(plan.strategy_state);
    if (!state) throw std::runtime_error("yellow-cube strategy state is missing");
    const Eigen::Vector3d center(
      plan.estimated_center.point.x,
      plan.estimated_center.point.y,
      plan.estimated_center.point.z);
    Eigen::Isometry3d world_from_tcp_grasp = Eigen::Isometry3d::Identity();
    world_from_tcp_grasp.linear() = state->grasp_rotation;
    world_from_tcp_grasp.translation() = Eigen::Vector3d(
      plan.grasp_pose.position.x, plan.grasp_pose.position.y,
      plan.grasp_pose.position.z);
    Eigen::Isometry3d world_from_object = Eigen::Isometry3d::Identity();
    world_from_object.linear() = state->object_rotation;
    world_from_object.translation() = center;
    const Eigen::Isometry3d tcp_from_object =
      world_from_tcp_grasp.inverse() * world_from_object;
    const Eigen::Vector3d tcp_center = tcp_from_object.translation();
    const Eigen::Quaterniond tcp_orientation(tcp_from_object.rotation());

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = runtime_.tcpFrame();
    attached.touch_links = {
      runtime_.tcpFrame(), runtime_.link6Frame(), "left_finger", "right_finger"};
    attached.object.header.frame_id = runtime_.tcpFrame();
    attached.object.id = "held/yellow_cube";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {cube_size_, cube_size_, cube_size_};
    attached.object.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    pose.position.x = tcp_center.x(); pose.position.y = tcp_center.y();
    pose.position.z = tcp_center.z();
    pose.orientation.x = tcp_orientation.x(); pose.orientation.y = tcp_orientation.y();
    pose.orientation.z = tcp_orientation.z(); pose.orientation.w = tcp_orientation.w();
    attached.object.primitive_poses.push_back(pose);
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    expected.header.frame_id = runtime_.tcpFrame();
    expected.header.stamp = node_->now();
    expected.point.x = tcp_center.x(); expected.point.y = tcp_center.y();
    expected.point.z = tcp_center.z();
    return attached;
  }

private:
  static double intervalError(double value, const Eigen::Vector2d& bounds)
  {
    if (value < bounds.x()) return bounds.x() - value;
    if (value > bounds.y()) return value - bounds.y();
    return 0.0;
  }

  bool measureFineTune(
    const PreparedPick& plan, FineTuneMeasurement& output, std::string& error)
  {
    std::vector<Eigen::Vector3d> camera_points;
    std::vector<Eigen::Vector3d> planning_points;
    camera_points.reserve(static_cast<std::size_t>(finetune_samples_));
    planning_points.reserve(static_cast<std::size_t>(finetune_samples_));
    Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();
    double offset_sum = 0.0;
    for (int sample = 0; sample < finetune_samples_; ++sample) {
      const auto estimate_response = estimate(
        srv::EstimateCube::Request::FINETUNE, plan.estimated_center);
      if (!estimate_response || !estimate_response->success) {
        error = estimate_response ? estimate_response->detail :
          "cube FINETUNE perception service is unavailable";
        return false;
      }
      Eigen::Vector3d normal(
        estimate_response->ground_normal.x,
        estimate_response->ground_normal.y,
        estimate_response->ground_normal.z);
      if (!normal.allFinite() || normal.norm() < 0.9) {
        error = "cube FINETUNE returned an invalid ground normal";
        return false;
      }
      normal.normalize();
      const Eigen::Vector3d center(
        estimate_response->center.point.x,
        estimate_response->center.point.y,
        estimate_response->center.point.z);
      const Eigen::Vector3d top_center = center + 0.5 * cube_size_ * normal;
      const Eigen::Isometry3d camera_from_planning = runtime_.lookup(
        runtime_.cameraFrame(), estimate_response->center.header.frame_id);
      const Eigen::Vector3d top_camera = camera_from_planning * top_center;
      if (!top_camera.allFinite()) {
        error = "cube FINETUNE top centre is non-finite after TF conversion";
        return false;
      }
      planning_points.push_back(top_center);
      camera_points.push_back(top_camera);
      normal_sum += normal;
      offset_sum += estimate_response->ground_offset;
    }

    for (const auto& point : camera_points) output.top_center_camera += point;
    for (const auto& point : planning_points) output.top_center_planning += point;
    output.top_center_camera /= static_cast<double>(camera_points.size());
    output.top_center_planning /= static_cast<double>(planning_points.size());
    output.ground_normal = normal_sum.normalized();
    output.ground_offset = offset_sum / static_cast<double>(camera_points.size());
    output.closing_coordinate = output.top_center_camera.dot(closing_axis_camera_);
    output.finger_coordinate = output.top_center_camera.dot(finger_axis_camera_);
    double closing_variance = 0.0;
    double finger_variance = 0.0;
    for (const auto& point : camera_points) {
      closing_variance += std::pow(
        point.dot(closing_axis_camera_) - output.closing_coordinate, 2);
      finger_variance += std::pow(
        point.dot(finger_axis_camera_) - output.finger_coordinate, 2);
    }
    output.closing_stddev = std::sqrt(
      closing_variance / static_cast<double>(camera_points.size()));
    output.finger_stddev = std::sqrt(
      finger_variance / static_cast<double>(camera_points.size()));
    return true;
  }

  bool acceptFineTune(
    PreparedPick& plan, YellowCubeState& state,
    const FineTuneMeasurement& measurement)
  {
    state.top_center = measurement.top_center_planning;
    plan.estimated_center.header.frame_id = runtime_.planningFrame();
    plan.estimated_center.header.stamp = node_->now();
    const Eigen::Vector3d center =
      measurement.top_center_planning - 0.5 * cube_size_ * measurement.ground_normal;
    plan.estimated_center.point.x = center.x();
    plan.estimated_center.point.y = center.y();
    plan.estimated_center.point.z = center.z();
    auto current_pose = runtime_.moveGroup().getCurrentPose(runtime_.tcpFrame()).pose;
    // Refresh the position from live feedback without replacing the canonical
    // gravity-aligned PREGRASP orientation with an endpoint residual.
    current_pose.orientation = plan.pregrasp_pose.orientation;
    plan.pregrasp_pose = current_pose;
    return runtime_.applyEstimatedGround(
      measurement.ground_normal, measurement.ground_offset);
  }

  void publishFineTuneMarkers(const FineTuneMeasurement& measurement)
  {
    visualization_msgs::msg::MarkerArray array;
    const Eigen::Vector3d extrusion_axis =
      closing_axis_camera_.cross(finger_axis_camera_).normalized();
    const double gravity_coordinate =
      measurement.top_center_camera.dot(extrusion_axis);
    const Eigen::Vector3d centre =
      0.5 * (closing_bounds_.x() + closing_bounds_.y()) * closing_axis_camera_ +
      0.5 * (finger_bounds_.x() + finger_bounds_.y()) * finger_axis_camera_ +
      gravity_coordinate * extrusion_axis;
    Eigen::Matrix3d orientation;
    orientation.col(0) = closing_axis_camera_;
    orientation.col(1) = finger_axis_camera_;
    orientation.col(2) = extrusion_axis;
    const Eigen::Quaterniond quaternion(orientation);

    visualization_msgs::msg::Marker prism;
    prism.header.frame_id = runtime_.cameraFrame();
    prism.header.stamp = node_->now();
    prism.ns = "finetune_safe_prism";
    prism.id = 100;
    prism.type = visualization_msgs::msg::Marker::CUBE;
    prism.action = visualization_msgs::msg::Marker::ADD;
    prism.pose.position.x = centre.x();
    prism.pose.position.y = centre.y();
    prism.pose.position.z = centre.z();
    prism.pose.orientation.x = quaternion.x();
    prism.pose.orientation.y = quaternion.y();
    prism.pose.orientation.z = quaternion.z();
    prism.pose.orientation.w = quaternion.w();
    prism.scale.x = closing_bounds_.y() - closing_bounds_.x();
    prism.scale.y = finger_bounds_.y() - finger_bounds_.x();
    prism.scale.z = 0.20;
    prism.color = color(0.1F, 1.0F, 0.2F, 0.22F);
    array.markers.push_back(prism);

    visualization_msgs::msg::Marker measured;
    measured.header = prism.header;
    measured.ns = "finetune_measured_top_center";
    measured.id = 101;
    measured.type = visualization_msgs::msg::Marker::SPHERE;
    measured.action = visualization_msgs::msg::Marker::ADD;
    measured.pose.position.x = measurement.top_center_camera.x();
    measured.pose.position.y = measurement.top_center_camera.y();
    measured.pose.position.z = measurement.top_center_camera.z();
    measured.pose.orientation.w = 1.0;
    measured.scale.x = measured.scale.y = measured.scale.z = 0.012;
    const bool inside =
      intervalError(measurement.closing_coordinate, closing_bounds_) == 0.0 &&
      intervalError(measurement.finger_coordinate, finger_bounds_) == 0.0;
    measured.color = inside ? color(0.1F, 1.0F, 0.2F) : color(1.0F, 0.1F, 0.1F);
    array.markers.push_back(measured);
    marker_publisher_->publish(array);
  }

  void diagnoseCartesianFailure(
    const moveit::core::RobotState& start_state,
    const geometry_msgs::msg::Pose& target, double collision_aware_fraction,
    double start_distance, double target_distance)
  {
    auto& move_group = runtime_.moveGroup();
    const auto* group = start_state.getJointModelGroup(move_group.getName());
    const auto* tcp = start_state.getLinkModel(runtime_.tcpFrame());
    if (!group || !tcp) {
      RCLCPP_ERROR(node_->get_logger(),
        "Cube descent diagnosis unavailable: arm group or TCP link model is missing");
      return;
    }

    const Eigen::Quaterniond orientation(
      target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
    Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
    target_transform.linear() = orientation.normalized().toRotationMatrix();
    target_transform.translation() = Eigen::Vector3d(
      target.position.x, target.position.y, target.position.z);

    auto probe = start_state;
    std::vector<moveit::core::RobotStatePtr> ik_path;
    const double ik_fraction = probe.computeCartesianPath(
      group, ik_path, tcp, target_transform, true, runtime_.cartesianStep(), 0.0);
    const double ik_reached_distance =
      start_distance + ik_fraction * (target_distance - start_distance);

    if (ik_fraction < runtime_.minimumCartesianFraction()) {
      const auto& last = ik_path.empty() ? start_state : *ik_path.back();
      Eigen::MatrixXd jacobian;
      double min_singular_value = std::numeric_limits<double>::quiet_NaN();
      double condition_number = std::numeric_limits<double>::infinity();
      double nearest_limit_margin = std::numeric_limits<double>::infinity();
      std::string nearest_limit_joint{"none"};
      for (const auto& variable : group->getVariableNames()) {
        const auto& bounds = last.getRobotModel()->getVariableBounds(variable);
        if (!bounds.position_bounded_) continue;
        const double position = last.getVariablePosition(variable);
        const double margin = std::min(
          position - bounds.min_position_, bounds.max_position_ - position);
        if (margin < nearest_limit_margin) {
          nearest_limit_margin = margin;
          nearest_limit_joint = variable;
        }
      }
      if (last.getJacobian(group, tcp, Eigen::Vector3d::Zero(), jacobian) &&
        jacobian.rows() > 0 && jacobian.cols() > 0)
      {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
        const auto singular_values = svd.singularValues();
        if (singular_values.size() > 0) {
          min_singular_value = singular_values(singular_values.size() - 1);
          if (min_singular_value > 1e-12) {
            condition_number = singular_values(0) / min_singular_value;
          }
        }
      }
      RCLCPP_ERROR(node_->get_logger(),
        "Cube descent diagnosis: reason=IK_INCOMPLETE target=%+.0f mm "
        "collision_aware=%.1f%% IK_only=%.1f%% IK_reached=%+.1f mm "
        "nearest_limit=%s margin=%.2fdeg jacobian_sigma_min=%.3e condition=%.1f",
        1000.0 * target_distance, 100.0 * collision_aware_fraction,
        100.0 * ik_fraction, 1000.0 * ik_reached_distance,
        nearest_limit_joint.c_str(), nearest_limit_margin * 180.0 / M_PI,
        min_singular_value, condition_number);
      return;
    }

    if (!state_validity_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(node_->get_logger(),
        "Cube descent diagnosis: collision is suspected but /check_state_validity is unavailable");
      return;
    }
    for (std::size_t index = 0; index < ik_path.size(); ++index) {
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      moveit::core::robotStateToRobotStateMsg(*ik_path[index], request->robot_state);
      request->group_name = move_group.getName();
      auto future = state_validity_client_->async_send_request(request);
      if (future.wait_for(1s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
          "Cube descent diagnosis: /check_state_validity timed out at sample %zu/%zu",
          index + 1, ik_path.size());
        return;
      }
      const auto response = future.get();
      if (response->valid) continue;
      const double sample_fraction = ik_path.size() <= 1 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(ik_path.size() - 1);
      const double sample_distance =
        start_distance + sample_fraction * (target_distance - start_distance);
      std::string contacts;
      for (const auto& contact : response->contacts) {
        if (!contacts.empty()) contacts += ", ";
        contacts += contact.contact_body_1 + "<->" + contact.contact_body_2;
      }
      if (contacts.empty()) contacts = "not reported by MoveIt";
      RCLCPP_ERROR(node_->get_logger(),
        "Cube descent diagnosis: reason=COLLISION target=%+.0f mm first_invalid=%+.1f mm "
        "sample=%zu/%zu contacts=[%s]",
        1000.0 * target_distance, 1000.0 * sample_distance,
        index + 1, ik_path.size(), contacts.c_str());
      return;
    }
    RCLCPP_ERROR(node_->get_logger(),
      "Cube descent diagnosis: reason=SCENE_MISMATCH target=%+.0f mm "
      "collision_aware=%.1f%% IK_only=%.1f%%; all sampled states were reported valid",
      1000.0 * target_distance, 100.0 * collision_aware_fraction, 100.0 * ik_fraction);
  }

  srv::EstimateCube::Response::SharedPtr estimate(uint8_t stage,
    const geometry_msgs::msg::PointStamped& hint,
    const Eigen::Vector3d& normal = Eigen::Vector3d::Zero(), double offset = 0.0)
  {
    if (!estimate_client_->wait_for_service(5s)) return {};
    auto request = std::make_shared<srv::EstimateCube::Request>();
    request->stage = stage; request->target_hint = hint;
    request->ground_normal.x = normal.x(); request->ground_normal.y = normal.y();
    request->ground_normal.z = normal.z(); request->ground_offset = offset;
    auto future = estimate_client_->async_send_request(request);
    if (future.wait_for(15s) != std::future_status::ready) return {};
    return future.get();
  }

  static std::vector<double> descending(double first, double last, double step)
  {
    std::vector<double> values;
    for (double value = first; value >= last - 1e-9; value -= step) values.push_back(value);
    return values;
  }

  static std::vector<double> ascending(double first, double last, double step)
  {
    std::vector<double> values;
    for (double value = first; value <= last + 1e-9; value += step) values.push_back(value);
    return values;
  }

  void publishMarkers(
    const srv::EstimateCube::Response& coarse,
    const srv::EstimateCube::Response& fine,
    const PreparedPick& plan)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    const auto append_ground = [&](const auto& estimate, const std::string& name,
        int id, const std_msgs::msg::ColorRGBA& plane_color)
      {
        visualization_msgs::msg::Marker plane;
        plane.header.frame_id = runtime_.planningFrame(); plane.ns = name; plane.id = id;
        plane.type = visualization_msgs::msg::Marker::CUBE;
        plane.action = visualization_msgs::msg::Marker::ADD;
        Eigen::Vector3d normal(
          estimate.ground_normal.x, estimate.ground_normal.y, estimate.ground_normal.z);
        normal.normalize();
        const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
          Eigen::Vector3d::UnitZ(), normal);
        plane.pose.orientation.x = q.x(); plane.pose.orientation.y = q.y();
        plane.pose.orientation.z = q.z(); plane.pose.orientation.w = q.w();
        const Eigen::Vector3d on_plane = -estimate.ground_offset * normal;
        plane.pose.position.x = on_plane.x(); plane.pose.position.y = on_plane.y();
        plane.pose.position.z = on_plane.z();
        plane.scale.x = 0.8; plane.scale.y = 0.6; plane.scale.z = 0.003;
        plane.color = plane_color;
        array.markers.push_back(plane);
      };
    append_ground(coarse, "coarse_ground_plane", 0, color(1.0F, 0.75F, 0.1F, 0.18F));
    append_ground(fine, "fine_ground_plane", 1, color(0.2F, 0.7F, 1.0F, 0.32F));
    int id = 2;
    for (const auto& item : {
      std::make_pair(std::string("cube_center"), plan.grasp_pose),
      std::make_pair(std::string("pregrasp"), plan.pregrasp_pose)})
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = runtime_.planningFrame(); arrow.ns = item.first; arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD; arrow.pose = item.second;
      arrow.scale.x = 0.10; arrow.scale.y = 0.015; arrow.scale.z = 0.02;
      arrow.color = item.first == "cube_center" ? color(1, 0, 1) : color(0, 1, 1);
      array.markers.push_back(arrow);
    }
    visualization_msgs::msg::Marker polygon;
    polygon.header.frame_id = runtime_.planningFrame(); polygon.ns = "fitted_top_square";
    polygon.id = id; polygon.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon.action = visualization_msgs::msg::Marker::ADD;
    polygon.scale.x = 0.006; polygon.color = color(1, 1, 0);
    for (const auto& point : fine.top_polygon.polygon.points) {
      geometry_msgs::msg::Point p;
      p.x = point.x; p.y = point.y; p.z = point.z; polygon.points.push_back(p);
    }
    if (!polygon.points.empty()) polygon.points.push_back(polygon.points.front());
    array.markers.push_back(polygon);
    marker_publisher_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  PickStrategyRuntime& runtime_;
  rclcpp::Client<srv::EstimateCube>::SharedPtr estimate_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  const std::string class_name_{"yellow_cube"};
  std::string estimate_name_;
  double cube_size_{}, pregrasp_max_{}, pregrasp_min_{}, pregrasp_step_{};
  double grasp_min_{}, grasp_max_{}, grasp_step_{};
  double gripper_open_{}, gripper_closed_{}, gripper_held_threshold_{};
  double grasp_settle_{}, lift_distance_{};
  double joint5_motion_weight_{};
  bool finetune_enabled_{false};
  std::string finetune_camera_frame_;
  Eigen::Vector3d closing_axis_camera_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d finger_axis_camera_{Eigen::Vector3d::UnitY()};
  Eigen::Vector2d closing_bounds_{-0.01, 0.01};
  Eigen::Vector2d finger_bounds_{-0.02, 0.02};
  double finetune_gain_{0.7};
  double finetune_max_step_{0.008};
  double finetune_max_total_{0.020};
  int finetune_max_corrections_{3};
  int finetune_samples_{3};
  double finetune_max_stddev_{0.002};
  double finetune_settle_s_{0.5};
};

std::unique_ptr<PickStrategy> makeYellowCubePickStrategy(
  const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
{
  return std::make_unique<YellowCubePickStrategy>(node, runtime);
}

}  // namespace d1_manipulation
