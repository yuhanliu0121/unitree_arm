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

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "d1_interfaces/action/pick_object.hpp"
#include "d1_manipulation/srv/estimate_zucchini.hpp"

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

struct ZucchiniCandidate
{
  geometry_msgs::msg::Pose pregrasp_pose;
  geometry_msgs::msg::Pose grasp_pose;
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d closing{Eigen::Vector3d::UnitY()};
  std::vector<double> joint_target;
  double pregrasp_ground_clearance{0.0};
  double grasp_clearance{0.0};
  double yaw_degrees{0.0};
  double motion_cost{std::numeric_limits<double>::infinity()};
  bool grasp_feasible{false};
};

struct ZucchiniState final : PickStrategyState
{
  Eigen::Vector3d grasp_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_up{Eigen::Vector3d::UnitZ()};
  Eigen::Matrix3d grasp_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d object_rotation{Eigen::Matrix3d::Identity()};
  std::vector<double> grasp_clearances;
  std::vector<std::string> target_collision_ids;
};

struct ZucchiniFineTuneMeasurement
{
  Eigen::Vector3d center_camera{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_planning{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ground_normal{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d object_axis_planning{Eigen::Vector3d::UnitX()};
  double ground_offset{0.0};
  double closing_coordinate{0.0};
  double closing_stddev{0.0};
};

shape_msgs::msg::Mesh ellipsoidMesh(double radius_x, double radius_y, double radius_z)
{
  // MuJoCo uses an analytic ellipsoid. MoveIt has no ellipsoid primitive, so
  // represent the same physical volume with a deterministic UV triangle mesh.
  constexpr uint32_t longitude_count = 24;
  constexpr uint32_t latitude_count = 12;
  shape_msgs::msg::Mesh mesh;
  geometry_msgs::msg::Point top;
  top.z = radius_z;
  mesh.vertices.push_back(top);
  for (uint32_t latitude = 1; latitude < latitude_count; ++latitude) {
    const double phi = M_PI * latitude / latitude_count;
    for (uint32_t longitude = 0; longitude < longitude_count; ++longitude) {
      const double theta = 2.0 * M_PI * longitude / longitude_count;
      geometry_msgs::msg::Point vertex;
      vertex.x = radius_x * std::sin(phi) * std::cos(theta);
      vertex.y = radius_y * std::sin(phi) * std::sin(theta);
      vertex.z = radius_z * std::cos(phi);
      mesh.vertices.push_back(vertex);
    }
  }
  const uint32_t bottom_index = static_cast<uint32_t>(mesh.vertices.size());
  geometry_msgs::msg::Point bottom;
  bottom.z = -radius_z;
  mesh.vertices.push_back(bottom);

  auto triangle = [&mesh](uint32_t a, uint32_t b, uint32_t c) {
      shape_msgs::msg::MeshTriangle face;
      face.vertex_indices = {a, b, c};
      mesh.triangles.push_back(face);
    };
  for (uint32_t longitude = 0; longitude < longitude_count; ++longitude) {
    const uint32_t next = (longitude + 1) % longitude_count;
    triangle(0, 1 + longitude, 1 + next);
  }
  for (uint32_t latitude = 0; latitude < latitude_count - 2; ++latitude) {
    const uint32_t current = 1 + latitude * longitude_count;
    const uint32_t following = current + longitude_count;
    for (uint32_t longitude = 0; longitude < longitude_count; ++longitude) {
      const uint32_t next = (longitude + 1) % longitude_count;
      triangle(current + longitude, following + longitude, following + next);
      triangle(current + longitude, following + next, current + next);
    }
  }
  const uint32_t last_ring = 1 + (latitude_count - 2) * longitude_count;
  for (uint32_t longitude = 0; longitude < longitude_count; ++longitude) {
    const uint32_t next = (longitude + 1) % longitude_count;
    triangle(last_ring + longitude, bottom_index, last_ring + next);
  }
  return mesh;
}
}  // namespace

class ZucchiniPickStrategy final : public PickStrategy
{
public:
  ZucchiniPickStrategy(const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
  : node_(node), runtime_(runtime)
  {
    estimate_name_ = parameterOrDeclare(
      node_, "zucchini_estimation_service_name",
      std::string("/arm/perception/estimate_zucchini"));
    length_ = parameterOrDeclare(node_, "zucchini_length_m", 0.15);
    width_ = parameterOrDeclare(node_, "zucchini_width_m", 0.040);
    height_ = parameterOrDeclare(node_, "zucchini_height_m", 0.03312);
    pregrasp_ground_clearance_max_ = parameterOrDeclare(
      node_, "zucchini_pregrasp_ground_clearance_max_m", 0.090);
    pregrasp_ground_clearance_min_ = parameterOrDeclare(
      node_, "zucchini_pregrasp_ground_clearance_min_m", 0.065);
    pregrasp_ground_clearance_step_ = parameterOrDeclare(
      node_, "zucchini_pregrasp_ground_clearance_step_m", 0.005);
    tcp_ground_clearance_ = parameterOrDeclare(
      node_, "zucchini_tcp_ground_clearance_m", 0.015);
    tcp_ground_clearance_max_ = parameterOrDeclare(
      node_, "zucchini_tcp_ground_clearance_max_m", 0.025);
    tcp_ground_clearance_step_ = parameterOrDeclare(
      node_, "zucchini_tcp_ground_clearance_step_m", 0.002);
    joint5_motion_weight_ = parameterOrDeclare(
      node_, "zucchini_joint5_motion_weight", 2.0);
    finetune_enabled_ = parameterOrDeclare(node_, "zucchini_finetune_enabled", true);
    finetune_camera_frame_ = parameterOrDeclare(
      node_, "zucchini_finetune_camera_frame",
      std::string{"wrist_camera_color_optical_frame"});
    const auto finetune_closing_axis = parameterOrDeclare(
      node_, "zucchini_finetune_closing_axis_camera",
      std::vector<double>{-1.0, 0.0, 0.0});
    const auto finetune_gravity_axis = parameterOrDeclare(
      node_, "zucchini_finetune_gravity_axis_camera",
      std::vector<double>{0.0, 0.0, -1.0});
    const auto finetune_closing_bounds = parameterOrDeclare(
      node_, "zucchini_finetune_closing_bounds_m",
      std::vector<double>{-0.01, 0.01});
    if (finetune_closing_axis.size() != 3 || finetune_gravity_axis.size() != 3 ||
      finetune_closing_bounds.size() != 2)
    {
      throw std::invalid_argument(
              "zucchini finetune axes must have 3 values and bounds 2 values");
    }
    finetune_closing_axis_camera_ = Eigen::Vector3d(
      finetune_closing_axis[0], finetune_closing_axis[1], finetune_closing_axis[2]);
    finetune_gravity_axis_camera_ = Eigen::Vector3d(
      finetune_gravity_axis[0], finetune_gravity_axis[1], finetune_gravity_axis[2]);
    finetune_closing_bounds_ = Eigen::Vector2d(
      finetune_closing_bounds[0], finetune_closing_bounds[1]);
    finetune_gain_ = parameterOrDeclare(node_, "zucchini_finetune_gain", 0.7);
    finetune_max_step_ = parameterOrDeclare(node_, "zucchini_finetune_max_step_m", 0.008);
    finetune_max_total_ = parameterOrDeclare(node_, "zucchini_finetune_max_total_m", 0.020);
    finetune_max_corrections_ = parameterOrDeclare(
      node_, "zucchini_finetune_max_corrections", 3);
    finetune_samples_ = parameterOrDeclare(node_, "zucchini_finetune_samples", 3);
    finetune_max_stddev_ = parameterOrDeclare(
      node_, "zucchini_finetune_max_stddev_m", 0.0015);
    finetune_settle_s_ = parameterOrDeclare(node_, "zucchini_finetune_settle_s", 0.5);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_closed_ = parameterOrDeclare(node_, "zucchini_gripper_closed_m", 0.01);
    gripper_held_threshold_ = parameterOrDeclare(
      node_, "zucchini_gripper_held_threshold_m", 0.011);
    grasp_settle_ = parameterOrDeclare(node_, "grasp_settle_s", 0.5);
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    if (length_ <= 0.0 || width_ <= 0.0 || height_ <= 0.0 ||
      pregrasp_ground_clearance_min_ < 0.0 ||
      pregrasp_ground_clearance_max_ < pregrasp_ground_clearance_min_ ||
      pregrasp_ground_clearance_step_ <= 0.0 ||
      tcp_ground_clearance_ < 0.0 ||
      tcp_ground_clearance_max_ < tcp_ground_clearance_ ||
      tcp_ground_clearance_step_ <= 0.0 || joint5_motion_weight_ <= 0.0 ||
      gripper_closed_ < 0.0 ||
      gripper_held_threshold_ <= gripper_closed_ || gripper_held_threshold_ > gripper_open_)
    {
      throw std::invalid_argument("invalid zucchini geometry or grasp search parameters");
    }
    if (finetune_closing_axis_camera_.norm() < 0.9 ||
      finetune_gravity_axis_camera_.norm() < 0.9 ||
      finetune_closing_bounds_.x() >= finetune_closing_bounds_.y() ||
      finetune_gain_ <= 0.0 || finetune_gain_ > 1.0 ||
      finetune_max_step_ <= 0.0 || finetune_max_total_ < finetune_max_step_ ||
      finetune_max_corrections_ < 0 || finetune_samples_ < 1 ||
      finetune_max_stddev_ <= 0.0 || finetune_settle_s_ < 0.0)
    {
      throw std::invalid_argument("invalid zucchini finetune parameters");
    }
    finetune_gravity_axis_camera_.normalize();
    finetune_closing_axis_camera_ -= finetune_gravity_axis_camera_ *
      finetune_closing_axis_camera_.dot(finetune_gravity_axis_camera_);
    if (finetune_closing_axis_camera_.norm() < 0.9) {
      throw std::invalid_argument("zucchini finetune closing and gravity axes are collinear");
    }
    finetune_closing_axis_camera_.normalize();
    estimate_client_ = node_->create_client<srv::EstimateZucchini>(estimate_name_);
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm/debug/zucchini_grasp_markers", rclcpp::QoS(1).transient_local().reliable());
  }

  const std::string& className() const override { return class_name_; }

  bool prepare(const geometry_msgs::msg::PointStamped& hint, PreparedPick& output,
    StrategyFailure& failure) override
  {
    const auto coarse = estimate(srv::EstimateZucchini::Request::COARSE, hint);
    if (!coarse || !coarse->success) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", coarse ? coarse->detail : "zucchini coarse estimation unavailable"};
      return false;
    }
    Eigen::Vector3d up(
      coarse->ground_normal.x, coarse->ground_normal.y, coarse->ground_normal.z);
    up.normalize();
    const Eigen::Vector3d coarse_center(
      coarse->center.point.x, coarse->center.point.y, coarse->center.point.z);
    if (!runtime_.applyEstimatedGround(up, coarse->ground_offset)) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply perception-fitted ground to MoveIt"};
      return false;
    }
    if (!runtime_.moveCameraTopDown(coarse_center, up, failure)) {
      return false;
    }
    const auto fine = estimate(
      srv::EstimateZucchini::Request::FINE, coarse->center, up, coarse->ground_offset);
    if (!fine || !fine->success) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", fine ? fine->detail : "zucchini fine estimation unavailable"};
      return false;
    }

    const Eigen::Vector3d surface_up_raw(
      fine->ground_normal.x, fine->ground_normal.y, fine->ground_normal.z);
    if (!surface_up_raw.allFinite() || surface_up_raw.norm() < 0.9) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "zucchini fine ground normal is invalid"};
      return false;
    }
    const Eigen::Vector3d surface_up = surface_up_raw.normalized();
    Eigen::Vector3d gravity_up = runtime_.gravityUp();
    if (!gravity_up.allFinite() || gravity_up.norm() < 0.9) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "zucchini IMU gravity direction is invalid"};
      return false;
    }
    gravity_up.normalize();
    const double ground_gravity_alignment = surface_up.dot(gravity_up);
    if (ground_gravity_alignment < 0.8) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "zucchini ground plane is inconsistent with IMU gravity"};
      return false;
    }
    if (!runtime_.applyEstimatedGround(surface_up, fine->ground_offset)) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply fine-observation ground to MoveIt"};
      return false;
    }

    const Eigen::Vector3d center(
      fine->center.point.x, fine->center.point.y, fine->center.point.z);
    Eigen::Vector3d axis(
      fine->axis_direction.x, fine->axis_direction.y, fine->axis_direction.z);
    axis = (axis - axis.dot(gravity_up) * gravity_up).normalized();
    if (!axis.allFinite()) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "zucchini local axis is invalid"};
      return false;
    }
    // Intersect the gravity line through the perceived centre with the
    // fine-observation ground plane. PREGRASP and GRASP heights are then both
    // explicit TCP clearances above that same ground point.
    const double centre_to_ground_along_gravity =
      (surface_up.dot(center) + fine->ground_offset) / ground_gravity_alignment;
    const Eigen::Vector3d ground_point =
      center - centre_to_ground_along_gravity * gravity_up;
    const Eigen::Vector3d grasp_point =
      ground_point + tcp_ground_clearance_ * gravity_up;
    RCLCPP_INFO(node_->get_logger(),
      "Zucchini fine geometry: center=(%.3f, %.3f, %.3f) axis=(%.3f, %.3f, %.3f) "
      "visible=(%.3f x %.3f) tcp_ground_clearance=%.0f mm",
      center.x(), center.y(), center.z(), axis.x(), axis.y(), axis.z(),
      fine->visible_length_m, fine->visible_width_m,
      1000.0 * tcp_ground_clearance_);
    const std::vector<std::string> target_ids{"zucchini", "observe_target"};
    const auto target_objects = runtime_.planningScene().getObjects(target_ids);
    const auto pregrasp_ground_clearances = descending(
      pregrasp_ground_clearance_max_, pregrasp_ground_clearance_min_,
      pregrasp_ground_clearance_step_);
    const auto grasp_clearances = ascending(
      tcp_ground_clearance_, tcp_ground_clearance_max_, tcp_ground_clearance_step_);

    ZucchiniCandidate selected;
    moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
    std::size_t selected_ik_candidates = 0;
    std::size_t selected_descent_candidates = 0;
    std::size_t full_plan_attempts = 0;
    bool found = false;
    auto& move_group = runtime_.moveGroup();
    move_group.setEndEffectorLink(runtime_.tcpFrame());
    for (const double pregrasp_ground_clearance : pregrasp_ground_clearances) {
      const auto current_state = move_group.getCurrentState();
      if (!current_state) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
          "PLAN_PREGRASP", "current robot state is unavailable for zucchini candidate ranking"};
        return false;
      }
      const auto* joint_group = current_state->getJointModelGroup(move_group.getName());
      if (!joint_group) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "PLAN_PREGRASP", "MoveIt arm joint model group is unavailable"};
        return false;
      }
      std::vector<double> current_joints;
      current_state->copyJointGroupPositions(joint_group, current_joints);
      const auto joint_names = joint_group->getVariableNames();
      std::vector<ZucchiniCandidate> candidates;

      for (const double direction_sign : {1.0, -1.0}) {
        const Eigen::Vector3d closing =
          direction_sign * gravity_up.cross(axis).normalized();
        const Eigen::Vector3d z = -gravity_up;
        // tcp_link inherits Link6 axes. The line joining the two fingertips
        // (and therefore the gripper closing axis) is TCP +Y, not TCP +X.
        // Keep TCP +Z downward and assign the desired closing direction to +Y.
        const Eigen::Vector3d y = closing;
        const Eigen::Vector3d x = y.cross(z).normalized();
        Eigen::Matrix3d vertical;
        vertical.col(0) = x; vertical.col(1) = y; vertical.col(2) = z;
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() = vertical;
        transform.translation() =
          ground_point + pregrasp_ground_clearance * gravity_up;
        const auto pregrasp_pose = poseMessage(transform);
        move_group.setStartStateToCurrentState();
        if (!move_group.setJointValueTarget(pregrasp_pose, runtime_.tcpFrame())) continue;
        std::vector<double> joint_target;
        move_group.getJointValueTarget(joint_target);
        auto endpoint_state = *current_state;
        endpoint_state.setJointGroupPositions(joint_group, joint_target);
        endpoint_state.update();
        if (!endpoint_state.satisfiesBounds(joint_group)) continue;
        double motion_cost = 0.0;
        if (!jointMotionCost(
            joint_names, current_joints, joint_target, joint5_motion_weight_, motion_cost))
        {
          continue;
        }
        ZucchiniCandidate candidate;
        candidate.pregrasp_pose = pregrasp_pose;
        candidate.rotation = vertical;
        candidate.closing = closing;
        candidate.joint_target = std::move(joint_target);
        candidate.pregrasp_ground_clearance = pregrasp_ground_clearance;
        candidate.yaw_degrees = std::atan2(closing.y(), closing.x()) * 180.0 / M_PI;
        candidate.motion_cost = motion_cost;
        candidates.push_back(std::move(candidate));
      }

      if (!runtime_.removeTargetCollision(target_ids)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "PLAN_PREGRASP", "target collision removal did not reach the planning scene"};
        return false;
      }
      for (auto& candidate : candidates) {
        auto endpoint_state = *current_state;
        endpoint_state.setJointGroupPositions(joint_group, candidate.joint_target);
        endpoint_state.update();
        move_group.setStartState(endpoint_state);
        for (const double clearance : grasp_clearances) {
          Eigen::Isometry3d grasp_transform = Eigen::Isometry3d::Identity();
          grasp_transform.linear() = candidate.rotation;
          grasp_transform.translation() = ground_point + clearance * gravity_up;
          moveit_msgs::msg::RobotTrajectory descent;
          const auto grasp_pose = poseMessage(grasp_transform);
          const double fraction = move_group.computeCartesianPath(
            {grasp_pose}, runtime_.cartesianStep(), 0.0, descent, true);
          if (fraction < runtime_.minimumCartesianFraction()) continue;
          candidate.grasp_pose = grasp_pose;
          candidate.grasp_clearance = clearance;
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
        std::remove_if(candidates.begin(), candidates.end(), [](const auto& candidate) {
          return !candidate.grasp_feasible;
        }),
        candidates.end());
      const std::size_t descent_candidates = candidates.size();
      std::stable_sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        constexpr double kTolerance = 1e-9;
        if (std::abs(lhs.grasp_clearance - rhs.grasp_clearance) > kTolerance) {
          return lhs.grasp_clearance < rhs.grasp_clearance;
        }
        return lhs.motion_cost < rhs.motion_cost;
      });
      RCLCPP_INFO(node_->get_logger(),
        "Zucchini candidate ranking: pregrasp_ground_clearance=%.0f mm feasible=%zu/%zu; "
        "policy=lowest_ground_clearance_first, then minimum_weighted_joint_motion "
        "(Joint5 weight %.1f)",
        1000.0 * pregrasp_ground_clearance, descent_candidates, ik_candidates,
        joint5_motion_weight_);

      for (std::size_t rank = 0; rank < candidates.size(); ++rank) {
        const auto& candidate = candidates[rank];
        ++full_plan_attempts;
        RCLCPP_INFO(node_->get_logger(),
          "Zucchini candidate attempt rank=%zu/%zu pregrasp_ground_clearance=%.0f mm "
          "tcp_ground_clearance=%.0f mm yaw=%.1f deg tilt=0.0 deg "
          "joint_motion_cost=%.4f",
          rank + 1, candidates.size(), 1000.0 * candidate.pregrasp_ground_clearance,
          1000.0 * candidate.grasp_clearance, candidate.yaw_degrees,
          candidate.motion_cost);
        move_group.setStartStateToCurrentState();
        if (!move_group.setJointValueTarget(candidate.joint_target)) continue;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) continue;
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
        if (fraction < runtime_.minimumCartesianFraction()) continue;
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
        "PLAN_PREGRASP", "no zucchini pregrasp has a feasible downstream grasp; reposition Go2"};
      return false;
    }

    output.class_name = class_name_; output.estimated_center = fine->center;
    output.grasp_pose = selected.grasp_pose; output.pregrasp_pose = selected.pregrasp_pose;
    output.pregrasp_plan = std::move(selected_plan); output.lift_direction = gravity_up;
    output.pregrasp_distance_m = selected.pregrasp_ground_clearance;
    output.grasp_distance_m = selected.grasp_clearance;
    output.grasp_yaw_degrees = selected.yaw_degrees;
    output.approach_tilt_degrees = 0.0;
    output.gripper_open_m = gripper_open_; output.gripper_closed_m = gripper_closed_;
    output.gripper_held_threshold_m = gripper_held_threshold_;
    output.grasp_settle_s = grasp_settle_; output.lift_distance_m = lift_distance_;
    auto state = std::make_shared<ZucchiniState>();
    state->grasp_point = grasp_point; state->gravity_up = gravity_up;
    state->grasp_rotation = selected.rotation;
    state->grasp_clearances = grasp_clearances;
    state->object_rotation.col(0) = axis;
    state->object_rotation.col(1) = selected.closing;
    state->object_rotation.col(2) = gravity_up;
    state->target_collision_ids = target_ids;
    output.strategy_state = std::move(state);
    publishMarkers(*coarse, *fine, output, axis, selected.closing);
    RCLCPP_INFO(node_->get_logger(),
      "Zucchini strategy selected pregrasp_ground_clearance=%.0f mm "
      "tcp_ground_clearance=%.0f mm "
      "yaw=%.1f deg tilt=0.0 deg joint_motion_cost=%.4f "
      "IK_candidates=%zu descent_candidates=%zu full_plan_attempts=%zu",
      1000.0 * selected.pregrasp_ground_clearance,
      1000.0 * selected.grasp_clearance,
      selected.yaw_degrees, selected.motion_cost, selected_ik_candidates,
      selected_descent_candidates, full_plan_attempts);
    return true;
  }

  bool fineTune(PreparedPick& plan, StrategyFailure& failure) override
  {
    if (!finetune_enabled_) {
      RCLCPP_INFO(node_->get_logger(),
        "Zucchini FINETUNE_GRASP is disabled; retaining the prepared pregrasp");
      return true;
    }
    if (runtime_.cameraFrame() != finetune_camera_frame_) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "FINETUNE_GRASP",
        "configured camera frame does not match the calibrated zucchini safe slab"};
      return false;
    }
    const auto state = std::dynamic_pointer_cast<ZucchiniState>(plan.strategy_state);
    if (!state) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "FINETUNE_GRASP", "zucchini strategy state is missing"};
      return false;
    }

    const double target_coordinate =
      0.5 * (finetune_closing_bounds_.x() + finetune_closing_bounds_.y());
    // Reuse the gravity-aligned PREGRASP orientation for every correction.
    // Feeding the measured FK orientation into the next target would turn
    // each endpoint residual into cumulative tool-axis drift.
    const auto pregrasp_orientation = plan.pregrasp_pose.orientation;
    double cumulative_correction = 0.0;
    for (int correction = 0; correction <= finetune_max_corrections_; ++correction) {
      ZucchiniFineTuneMeasurement measurement;
      std::string measurement_error;
      if (!measureFineTune(plan, measurement, measurement_error)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
          "FINETUNE_GRASP", measurement_error};
        return false;
      }
      publishFineTuneMarkers(measurement);
      const double target_error = std::abs(
        measurement.closing_coordinate - target_coordinate);
      const double outside_error = intervalError(
        measurement.closing_coordinate, finetune_closing_bounds_);
      const bool inside_slab =
        outside_error <= 1e-9;
      RCLCPP_INFO(node_->get_logger(),
        "Zucchini FINETUNE_GRASP observation %d/%d: closing=%.1f mm "
        "slab=[%.1f, %.1f] correction_target=%.1f mm centre_error=%.2f mm "
        "outside=%.2f mm std=%.2f mm inside=%s",
        correction + 1, finetune_max_corrections_ + 1,
        1000.0 * measurement.closing_coordinate,
        1000.0 * finetune_closing_bounds_.x(),
        1000.0 * finetune_closing_bounds_.y(), 1000.0 * target_coordinate,
        1000.0 * target_error, 1000.0 * outside_error,
        1000.0 * measurement.closing_stddev,
        inside_slab ? "true" : "false");

      if (measurement.closing_stddev > finetune_max_stddev_) {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(1)
               << "zucchini centre is unstable across " << finetune_samples_
               << " frames: closing_std=" << 1000.0 * measurement.closing_stddev
               << " mm; keep PREGRASP and reposition Go2";
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
          "FINETUNE_GRASP", detail.str()};
        return false;
      }

      if (inside_slab) {
        if (!acceptFineTune(plan, *state, measurement)) {
          failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
            "FINETUNE_GRASP", "failed to update the refreshed zucchini geometry"};
          return false;
        }
        RCLCPP_INFO(node_->get_logger(),
          "Zucchini FINETUNE_GRASP accepted inside calibrated safe slab after "
          "%d correction(s); centre is a correction target, not an acceptance threshold",
          correction);
        return true;
      }
      if (correction == finetune_max_corrections_) break;

      Eigen::Vector3d correction_camera = finetune_gain_ *
        (measurement.closing_coordinate - target_coordinate) *
        finetune_closing_axis_camera_;
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
      Eigen::Vector3d gravity_up = runtime_.gravityUp();
      gravity_up.normalize();
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
          "FINETUNE_GRASP", "zucchini visual correction has no IK solution; reposition Go2"};
        return false;
      }
      moveit::planning_interface::MoveGroupInterface::Plan correction_plan;
      const bool planned =
        move_group.plan(correction_plan) == moveit::core::MoveItErrorCode::SUCCESS;
      move_group.clearPoseTargets();
      if (!planned) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
          "FINETUNE_GRASP",
          "zucchini visual correction is not collision-free/plannable; reposition Go2"};
        return false;
      }
      RCLCPP_INFO(node_->get_logger(),
        "Zucchini FINETUNE_GRASP correction %d: camera_closing_delta="
        "(%+.1f, %+.1f, %+.1f) mm planning_horizontal_delta="
        "(%+.1f, %+.1f, %+.1f) mm",
        correction + 1, 1000.0 * correction_camera.x(),
        1000.0 * correction_camera.y(), 1000.0 * correction_camera.z(),
        1000.0 * correction_planning.x(), 1000.0 * correction_planning.y(),
        1000.0 * correction_planning.z());
      if (!runtime_.executeFineTunePlan(correction_plan)) {
        failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
          "FINETUNE_GRASP", "zucchini visual correction execution failed; motion stopped"};
        return false;
      }
      cumulative_correction += correction_norm;
      std::this_thread::sleep_for(std::chrono::duration<double>(finetune_settle_s_));
    }

    std::ostringstream detail;
    detail << "zucchini centre remains outside the calibrated safe slab after "
           << finetune_max_corrections_
           << " corrections; keep PREGRASP and reposition Go2";
    failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
      "FINETUNE_GRASP", detail.str()};
    return false;
  }

  bool confirmDescent(PreparedPick& plan, StrategyFailure& failure) override
  {
    const auto state = std::dynamic_pointer_cast<ZucchiniState>(plan.strategy_state);
    if (!state) {
      failure = {d1_interfaces::action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "zucchini strategy state is missing"};
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
        "DESCEND", "current robot state is unavailable at the actual zucchini pregrasp"};
      return false;
    }
    std::size_t rejected_clearances = 0;
    for (const double clearance : state->grasp_clearances) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.linear() = state->grasp_rotation;
      transform.translation() = state->grasp_point +
        (clearance - tcp_ground_clearance_) * state->gravity_up;
      move_group.setStartState(*descent_start);
      moveit_msgs::msg::RobotTrajectory descent;
      const double fraction = move_group.computeCartesianPath(
        {poseMessage(transform)}, runtime_.cartesianStep(), 0.0, descent, true);
      if (fraction < runtime_.minimumCartesianFraction()) {
        RCLCPP_WARN(node_->get_logger(),
          "Zucchini live descent rejected: tcp_ground_clearance=%.0f mm "
          "fraction=%.1f%% required=%.1f%%",
          1000.0 * clearance, 100.0 * fraction,
          100.0 * runtime_.minimumCartesianFraction());
        ++rejected_clearances;
        continue;
      }
      RCLCPP_INFO(node_->get_logger(),
        "Zucchini live descent selected: tcp_ground_clearance=%.0f mm "
        "fraction=%.1f%% after_rejecting_deeper_targets=%zu "
        "reason=first_complete_live_cartesian_path",
        1000.0 * clearance, 100.0 * fraction, rejected_clearances);
      plan.grasp_pose = poseMessage(transform);
      plan.grasp_distance_m = clearance;
      plan.descent_trajectory = std::move(descent);
      return true;
    }
    failure = {d1_interfaces::action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
      "DESCEND", "no ground-referenced zucchini grasp height is reachable from the actual "
      "pregrasp; reposition Go2"};
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject makeAttachedObject(
    const PreparedPick& plan, geometry_msgs::msg::PointStamped& expected) override
  {
    const auto state = std::dynamic_pointer_cast<ZucchiniState>(plan.strategy_state);
    if (!state) throw std::runtime_error("zucchini strategy state is missing");
    const Eigen::Vector3d center(
      plan.estimated_center.point.x, plan.estimated_center.point.y,
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

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = runtime_.tcpFrame();
    attached.touch_links = {
      runtime_.tcpFrame(), runtime_.link6Frame(), "left_finger", "right_finger"};
    attached.object.header.frame_id = runtime_.tcpFrame();
    attached.object.id = "held/zucchini";
    attached.object.meshes.push_back(
      ellipsoidMesh(0.5 * length_, 0.5 * width_, 0.5 * height_));
    attached.object.mesh_poses.push_back(poseMessage(tcp_from_object));
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    expected.header.frame_id = runtime_.tcpFrame(); expected.header.stamp = node_->now();
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
    const PreparedPick& plan, ZucchiniFineTuneMeasurement& output,
    std::string& error)
  {
    std::vector<Eigen::Vector3d> camera_points;
    std::vector<Eigen::Vector3d> planning_points;
    camera_points.reserve(static_cast<std::size_t>(finetune_samples_));
    planning_points.reserve(static_cast<std::size_t>(finetune_samples_));
    Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_reference = Eigen::Vector3d::Zero();
    double offset_sum = 0.0;
    for (int sample = 0; sample < finetune_samples_; ++sample) {
      const auto response = estimate(
        srv::EstimateZucchini::Request::FINE, plan.estimated_center);
      if (!response || !response->success) {
        error = response ? response->detail :
          "zucchini FINETUNE perception service is unavailable";
        return false;
      }
      Eigen::Vector3d normal(
        response->ground_normal.x, response->ground_normal.y,
        response->ground_normal.z);
      Eigen::Vector3d axis(
        response->axis_direction.x, response->axis_direction.y,
        response->axis_direction.z);
      const Eigen::Vector3d center(
        response->center.point.x, response->center.point.y,
        response->center.point.z);
      if (!normal.allFinite() || normal.norm() < 0.9 ||
        !axis.allFinite() || axis.norm() < 0.9 || !center.allFinite())
      {
        error = "zucchini FINETUNE returned invalid centre, axis, or ground geometry";
        return false;
      }
      normal.normalize();
      axis.normalize();
      if (sample == 0) axis_reference = axis;
      if (axis.dot(axis_reference) < 0.0) axis = -axis;
      const Eigen::Isometry3d camera_from_planning = runtime_.lookup(
        runtime_.cameraFrame(), response->center.header.frame_id);
      const Eigen::Vector3d center_camera = camera_from_planning * center;
      if (!center_camera.allFinite()) {
        error = "zucchini FINETUNE centre is non-finite after TF conversion";
        return false;
      }
      camera_points.push_back(center_camera);
      planning_points.push_back(center);
      normal_sum += normal;
      axis_sum += axis;
      offset_sum += response->ground_offset;
    }

    for (const auto& point : camera_points) output.center_camera += point;
    for (const auto& point : planning_points) output.center_planning += point;
    output.center_camera /= static_cast<double>(camera_points.size());
    output.center_planning /= static_cast<double>(planning_points.size());
    output.ground_normal = normal_sum.normalized();
    output.object_axis_planning = axis_sum.normalized();
    output.ground_offset = offset_sum / static_cast<double>(camera_points.size());
    output.closing_coordinate =
      output.center_camera.dot(finetune_closing_axis_camera_);
    double variance = 0.0;
    for (const auto& point : camera_points) {
      variance += std::pow(
        point.dot(finetune_closing_axis_camera_) - output.closing_coordinate, 2);
    }
    output.closing_stddev = std::sqrt(
      variance / static_cast<double>(camera_points.size()));
    return true;
  }

  bool acceptFineTune(
    PreparedPick& plan, ZucchiniState& state,
    const ZucchiniFineTuneMeasurement& measurement)
  {
    Eigen::Vector3d gravity_up = runtime_.gravityUp();
    if (!gravity_up.allFinite() || gravity_up.norm() < 0.9) return false;
    gravity_up.normalize();
    const double alignment = measurement.ground_normal.dot(gravity_up);
    if (alignment < 0.8) return false;
    const double centre_to_ground =
      (measurement.ground_normal.dot(measurement.center_planning) +
      measurement.ground_offset) / alignment;
    const Eigen::Vector3d ground_point =
      measurement.center_planning - centre_to_ground * gravity_up;
    state.grasp_point = ground_point + tcp_ground_clearance_ * gravity_up;
    state.gravity_up = gravity_up;
    Eigen::Vector3d axis = measurement.object_axis_planning -
      measurement.object_axis_planning.dot(gravity_up) * gravity_up;
    if (!axis.allFinite() || axis.norm() < 0.9) return false;
    axis.normalize();
    if (axis.dot(state.object_rotation.col(0)) < 0.0) axis = -axis;
    Eigen::Vector3d closing = gravity_up.cross(axis).normalized();
    if (closing.dot(state.grasp_rotation.col(1)) < 0.0) closing = -closing;
    state.object_rotation.col(0) = axis;
    state.object_rotation.col(1) = closing;
    state.object_rotation.col(2) = gravity_up;
    plan.estimated_center.header.frame_id = runtime_.planningFrame();
    plan.estimated_center.header.stamp = node_->now();
    plan.estimated_center.point.x = measurement.center_planning.x();
    plan.estimated_center.point.y = measurement.center_planning.y();
    plan.estimated_center.point.z = measurement.center_planning.z();
    auto current_pose = runtime_.moveGroup().getCurrentPose(runtime_.tcpFrame()).pose;
    current_pose.orientation = plan.pregrasp_pose.orientation;
    plan.pregrasp_pose = current_pose;
    return runtime_.applyEstimatedGround(
      measurement.ground_normal, measurement.ground_offset);
  }

  void publishFineTuneMarkers(const ZucchiniFineTuneMeasurement& measurement)
  {
    visualization_msgs::msg::MarkerArray array;
    Eigen::Vector3d finger_axis =
      finetune_gravity_axis_camera_.cross(finetune_closing_axis_camera_);
    finger_axis.normalize();
    Eigen::Matrix3d orientation;
    orientation.col(0) = finetune_closing_axis_camera_;
    orientation.col(1) = finger_axis;
    orientation.col(2) = finetune_gravity_axis_camera_;
    const Eigen::Quaterniond quaternion(orientation);
    const double closing_midpoint =
      0.5 * (finetune_closing_bounds_.x() + finetune_closing_bounds_.y());
    const double finger_coordinate = measurement.center_camera.dot(finger_axis);
    const double gravity_coordinate =
      measurement.center_camera.dot(finetune_gravity_axis_camera_);
    const Eigen::Vector3d slab_centre =
      closing_midpoint * finetune_closing_axis_camera_ +
      finger_coordinate * finger_axis +
      gravity_coordinate * finetune_gravity_axis_camera_;

    visualization_msgs::msg::Marker slab;
    slab.header.frame_id = runtime_.cameraFrame();
    slab.header.stamp = node_->now();
    slab.ns = "zucchini_finetune_safe_slab";
    slab.id = 200;
    slab.type = visualization_msgs::msg::Marker::CUBE;
    slab.action = visualization_msgs::msg::Marker::ADD;
    slab.pose.position.x = slab_centre.x();
    slab.pose.position.y = slab_centre.y();
    slab.pose.position.z = slab_centre.z();
    slab.pose.orientation.x = quaternion.x();
    slab.pose.orientation.y = quaternion.y();
    slab.pose.orientation.z = quaternion.z();
    slab.pose.orientation.w = quaternion.w();
    slab.scale.x = finetune_closing_bounds_.y() - finetune_closing_bounds_.x();
    slab.scale.y = 0.25;
    slab.scale.z = 0.20;
    slab.color = color(0.1F, 1.0F, 0.2F, 0.20F);
    array.markers.push_back(slab);

    visualization_msgs::msg::Marker measured;
    measured.header = slab.header;
    measured.ns = "zucchini_finetune_measured_centre";
    measured.id = 201;
    measured.type = visualization_msgs::msg::Marker::SPHERE;
    measured.action = visualization_msgs::msg::Marker::ADD;
    measured.pose.position.x = measurement.center_camera.x();
    measured.pose.position.y = measurement.center_camera.y();
    measured.pose.position.z = measurement.center_camera.z();
    measured.pose.orientation.w = 1.0;
    measured.scale.x = measured.scale.y = measured.scale.z = 0.012;
    const bool inside =
      measurement.closing_coordinate >= finetune_closing_bounds_.x() &&
      measurement.closing_coordinate <= finetune_closing_bounds_.y();
    measured.color = inside ? color(0.1F, 1.0F, 0.2F) : color(1.0F, 0.1F, 0.1F);
    array.markers.push_back(measured);
    marker_publisher_->publish(array);
  }

  srv::EstimateZucchini::Response::SharedPtr estimate(uint8_t stage,
    const geometry_msgs::msg::PointStamped& hint,
    const Eigen::Vector3d& normal = Eigen::Vector3d::Zero(), double offset = 0.0)
  {
    if (!estimate_client_->wait_for_service(5s)) return {};
    auto request = std::make_shared<srv::EstimateZucchini::Request>();
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
    const srv::EstimateZucchini::Response& coarse,
    const srv::EstimateZucchini::Response& fine,
    const PreparedPick& plan, const Eigen::Vector3d& axis, const Eigen::Vector3d& closing)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear; clear.action = visualization_msgs::msg::Marker::DELETEALL;
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
    const Eigen::Vector3d center(
      fine.center.point.x, fine.center.point.y, fine.center.point.z);
    int id = 2;
    for (const auto& item : {
      std::make_pair(std::string("local_axis"), axis),
      std::make_pair(std::string("closing_direction"), closing)})
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = runtime_.planningFrame(); arrow.ns = item.first; arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point start, end;
      start.x = center.x(); start.y = center.y(); start.z = center.z();
      const Eigen::Vector3d endpoint = center + 0.10 * item.second;
      end.x = endpoint.x(); end.y = endpoint.y(); end.z = endpoint.z();
      arrow.points = {start, end}; arrow.scale.x = 0.008; arrow.scale.y = 0.016;
      arrow.scale.z = 0.020;
      arrow.color = item.first == "local_axis" ? color(0, 1, 0) : color(1, 0, 1);
      array.markers.push_back(arrow);
    }
    for (const auto& item : {
      std::make_pair(std::string("grasp"), plan.grasp_pose),
      std::make_pair(std::string("pregrasp"), plan.pregrasp_pose)})
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = runtime_.planningFrame(); arrow.ns = item.first; arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD; arrow.pose = item.second;
      arrow.scale.x = 0.10; arrow.scale.y = 0.015; arrow.scale.z = 0.02;
      arrow.color = item.first == "grasp" ? color(1, 1, 0) : color(0, 1, 1);
      array.markers.push_back(arrow);
    }
    marker_publisher_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  PickStrategyRuntime& runtime_;
  rclcpp::Client<srv::EstimateZucchini>::SharedPtr estimate_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  const std::string class_name_{"zucchini"};
  std::string estimate_name_;
  double length_{}, width_{}, height_{};
  double pregrasp_ground_clearance_max_{}, pregrasp_ground_clearance_min_{};
  double pregrasp_ground_clearance_step_{};
  double tcp_ground_clearance_{}, tcp_ground_clearance_max_{}, tcp_ground_clearance_step_{};
  double joint5_motion_weight_{};
  double gripper_open_{}, gripper_closed_{}, gripper_held_threshold_{};
  double grasp_settle_{}, lift_distance_{};
  bool finetune_enabled_{false};
  std::string finetune_camera_frame_;
  Eigen::Vector3d finetune_closing_axis_camera_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d finetune_gravity_axis_camera_{0.0, 0.0, -1.0};
  Eigen::Vector2d finetune_closing_bounds_{-0.01, 0.01};
  double finetune_gain_{0.7};
  double finetune_max_step_{0.008};
  double finetune_max_total_{0.020};
  int finetune_max_corrections_{3};
  int finetune_samples_{3};
  double finetune_max_stddev_{0.0015};
  double finetune_settle_s_{0.5};
};

std::unique_ptr<PickStrategy> makeZucchiniPickStrategy(
  const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
{
  return std::make_unique<ZucchiniPickStrategy>(node, runtime);
}

}  // namespace d1_manipulation
