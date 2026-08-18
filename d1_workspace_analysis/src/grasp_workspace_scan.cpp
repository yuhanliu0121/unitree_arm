#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>

#include "d1_manipulation/observation_geometry.hpp"

namespace d1_workspace_analysis
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

template<typename T>
T parameterOrDeclare(
  const rclcpp::Node::SharedPtr& node, const std::string& name, const T& fallback)
{
  if (node->has_parameter(name)) {
    T value;
    if (node->get_parameter(name, value)) return value;
  }
  return node->declare_parameter<T>(name, fallback);
}

geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

moveit_msgs::msg::CollisionObject boxObject(
  const std::string& id, const std::string& frame, const Eigen::Vector3d& centre,
  const Eigen::Vector3d& dimensions)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {dimensions.x(), dimensions.y(), dimensions.z()};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.position.x = centre.x();
  pose.position.y = centre.y();
  pose.position.z = centre.z();
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std::vector<double> sequence(double first, double last, double step)
{
  std::vector<double> values;
  if (step <= 0.0) return values;
  for (double value = first; value <= last + 1e-9; value += step) {
    values.push_back(value);
  }
  return values;
}

std::vector<double> orderedBowlAzimuths()
{
  std::vector<double> values{0.0};
  for (int angle = 15; angle <= 165; angle += 15) {
    values.push_back(-static_cast<double>(angle));
    values.push_back(static_cast<double>(angle));
  }
  values.push_back(180.0);
  return values;
}

shape_msgs::msg::Mesh ellipsoidMesh(double radius_x, double radius_y, double radius_z)
{
  constexpr uint32_t longitude_count = 24;
  constexpr uint32_t latitude_count = 12;
  shape_msgs::msg::Mesh mesh;
  geometry_msgs::msg::Point top;
  top.z = radius_z;
  mesh.vertices.push_back(top);
  for (uint32_t latitude = 1; latitude < latitude_count; ++latitude) {
    const double phi = kPi * latitude / latitude_count;
    for (uint32_t longitude = 0; longitude < longitude_count; ++longitude) {
      const double theta = 2.0 * kPi * longitude / longitude_count;
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

struct TrialResult
{
  bool success{false};
  std::string failed_stage{"UNKNOWN"};
  double selected_pregrasp_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_grasp_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_tilt_deg{std::numeric_limits<double>::quiet_NaN()};
  double selected_grasp_yaw_deg{std::numeric_limits<double>::quiet_NaN()};
  double planning_time_s{0.0};
};
}  // namespace

class GraspWorkspaceScan
{
public:
  explicit GraspWorkspaceScan(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    object_type_ = parameterOrDeclare(node_, "object_type", std::string("yellow_cube"));
    output_csv_ = parameterOrDeclare(
      node_, "output_csv", std::string("/tmp/grasp_workspace.csv"));
    x_min_ = parameterOrDeclare(node_, "x_min_m", -0.70);
    x_max_ = parameterOrDeclare(node_, "x_max_m", 0.70);
    y_min_ = parameterOrDeclare(node_, "y_min_m", -0.70);
    y_max_ = parameterOrDeclare(node_, "y_max_m", 0.70);
    xy_step_ = parameterOrDeclare(node_, "xy_step_m", 0.05);
    yaw_step_deg_ = parameterOrDeclare(node_, "yaw_step_deg", 10.0);
    ground_z_ = parameterOrDeclare(node_, "ground_surface_z_m", -0.225248769402);
    cube_size_ = parameterOrDeclare(node_, "cube_size_m", 0.05);
    zucchini_length_ = parameterOrDeclare(node_, "zucchini_length_m", 0.15);
    zucchini_width_ = parameterOrDeclare(node_, "zucchini_width_m", 0.040);
    zucchini_height_ = parameterOrDeclare(node_, "zucchini_height_m", 0.03312);
    zucchini_tcp_ground_clearance_ = parameterOrDeclare(
      node_, "zucchini_tcp_ground_clearance_m", 0.015);
    bowl_radius_ = parameterOrDeclare(node_, "bowl_radius_m", 0.058);
    bowl_height_ = parameterOrDeclare(node_, "bowl_height_m", 0.05001143);
    ik_timeout_s_ = parameterOrDeclare(node_, "ik_timeout_s", 0.025);
    planning_time_s_ = parameterOrDeclare(node_, "scan_planning_time_s", 0.35);
    cartesian_step_ = parameterOrDeclare(node_, "cartesian_step_m", 0.005);
    minimum_cartesian_fraction_ = parameterOrDeclare(node_, "minimum_cartesian_fraction", 0.95);

    beta_degrees_ = parameterOrDeclare<std::vector<double>>(node_,
      "beta_degrees", {0.0, -5.0, 5.0, -10.0, 10.0, -15.0, 15.0});
    alpha_degrees_ = parameterOrDeclare<std::vector<double>>(node_,
      "alpha_degrees", {45.0, 40.0, 50.0, 35.0, 55.0, 30.0, 60.0, 65.0, 70.0});
    observation_distances_ = parameterOrDeclare<std::vector<double>>(node_,
      "distances_m", {0.35, 0.30, 0.40, 0.45, 0.50, 0.55, 0.60});
    top_distances_ = parameterOrDeclare<std::vector<double>>(node_,
      "top_observation_distances_m", {0.35, 0.40, 0.45, 0.50, 0.55, 0.60});
    top_rolls_ = parameterOrDeclare<std::vector<double>>(node_,
      "top_observation_roll_degrees", {0.0, 90.0, -90.0, 180.0, 45.0, -45.0, 135.0, -135.0});
    pregrasp_distances_ = parameterOrDeclare<std::vector<double>>(
      node_, "scan_pregrasp_distances_m", {0.080, 0.050, 0.020});
    grasp_distances_ = parameterOrDeclare<std::vector<double>>(
      node_, "scan_grasp_distances_m", {-0.040, -0.032, -0.024});
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    stowed_ = parameterOrDeclare<std::vector<double>>(node_,
      "stowed_joint_positions", {0.0, -1.5, 1.5, 0.0, 0.0, 0.0});
    carry_ = parameterOrDeclare<std::vector<double>>(node_,
      "carry_joint_positions", {0.0, -1.54, 1.546, 0.0, -0.6, 1.57});
    const auto link6_from_camera_values = parameterOrDeclare<std::vector<double>>(
      node_, "T_link6_color_optical", {
        0.007802511223, 0.969491460227, 0.245000876253, -0.113433495391,
        -0.999491412298, -0.000014772168, 0.031889128609, 0.033016808957,
        0.030919857055, -0.245125087105, 0.968998273534, -0.051772117970,
        0.0, 0.0, 0.0, 1.0});

    if (xy_step_ <= 0.0 || yaw_step_deg_ <= 0.0 || x_max_ < x_min_ || y_max_ < y_min_) {
      throw std::invalid_argument("invalid workspace scan bounds or steps");
    }
    if (object_type_ != "yellow_cube" && object_type_ != "zucchini" && object_type_ != "bowl") {
      throw std::invalid_argument("object_type must be yellow_cube, zucchini, or bowl");
    }
    if (link6_from_camera_values.size() != 16) {
      throw std::invalid_argument("T_link6_color_optical must contain 16 row-major values");
    }
    Eigen::Matrix4d link6_from_camera_matrix;
    for (std::size_t row = 0; row < 4; ++row) {
      for (std::size_t column = 0; column < 4; ++column) {
        link6_from_camera_matrix(row, column) = link6_from_camera_values[4 * row + column];
      }
    }
    link6_from_camera_.matrix() = link6_from_camera_matrix;

    robot_model_loader::RobotModelLoader loader(node_, "robot_description");
    model_ = loader.getModel();
    if (!model_) throw std::runtime_error("failed to load robot model");
    arm_ = model_->getJointModelGroup("arm");
    if (!arm_) throw std::runtime_error("MoveIt group 'arm' is unavailable");

    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    applyStaticScene();
    pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
      model_, node_, "ompl", "ompl_interface/OMPLPlanner", std::vector<std::string>{});
    pipeline_->displayComputedMotionPlans(false);
    pipeline_->publishReceivedRequests(false);

    stowed_state_ = std::make_shared<moveit::core::RobotState>(model_);
    stowed_state_->setToDefaultValues();
    stowed_state_->setJointGroupPositions(arm_, stowed_);
    stowed_state_->setVariablePosition("Joint6", 0.03);
    stowed_state_->update();
    carry_state_ = std::make_shared<moveit::core::RobotState>(*stowed_state_);
    carry_state_->setJointGroupPositions(arm_, carry_);
    carry_state_->setVariablePosition("Joint6", 0.025);
    carry_state_->update();
    if (!stowed_state_->satisfiesBounds(arm_) || !carry_state_->satisfiesBounds(arm_)) {
      throw std::runtime_error("configured STOWED or CARRY state violates joint bounds");
    }

  }

  void run()
  {
    const auto xs = sequence(x_min_, x_max_, xy_step_);
    const auto ys = sequence(y_min_, y_max_, xy_step_);
    const auto yaws = object_type_ == "yellow_cube" ?
      sequence(0.0, 80.0, yaw_step_deg_) :
      (object_type_ == "zucchini" ? sequence(0.0, 170.0, yaw_step_deg_) :
      std::vector<double>{0.0});
    std::filesystem::create_directories(std::filesystem::path(output_csv_).parent_path());
    std::ofstream output(output_csv_);
    if (!output) throw std::runtime_error("cannot open output CSV: " + output_csv_);
    output << "x_m,y_m,object_yaw_deg,success,failed_stage,pregrasp_m,grasp_m,"
              "grasp_yaw_deg,tilt_deg,planning_time_s\n";
    output << std::fixed << std::setprecision(6);

    const std::size_t point_count = xs.size() * ys.size();
    std::size_t point_index = 0;
    std::size_t successful_trials = 0;
    for (const double y : ys) {
      for (const double x : xs) {
        ++point_index;
        const double object_height = object_type_ == "yellow_cube" ? cube_size_ :
          (object_type_ == "zucchini" ? zucchini_height_ : bowl_height_);
        const Eigen::Vector3d centre(x, y, ground_z_ + 0.5 * object_height);
        moveit::core::RobotState observation_state(model_);
        moveit::core::RobotState top_state(model_);
        std::string shared_failure;
        const double footprint_radius = object_type_ == "yellow_cube" ?
          cube_size_ / std::sqrt(2.0) :
          (object_type_ == "zucchini" ? 0.5 * zucchini_length_ : bowl_radius_);
        const bool overlaps_go2_footprint =
          x >= -0.421484 - footprint_radius &&
          x <= 0.331958 + footprint_radius &&
          std::abs(y) <= 0.169127 + footprint_radius;
        if (overlaps_go2_footprint) shared_failure = "TARGET_OVERLAPS_GO2_FOOTPRINT";
        const bool observation_ok = !overlaps_go2_footprint &&
          findObservation(centre, observation_state, shared_failure);
        const bool top_ok = observation_ok && findTopObservation(
          centre, observation_state, top_state, shared_failure);
        for (const double yaw : yaws) {
          TrialResult result;
          if (!observation_ok || !top_ok) {
            result.failed_stage = shared_failure;
          } else {
            result = object_type_ == "yellow_cube" ? evaluateYaw(centre, yaw, top_state) :
              (object_type_ == "zucchini" ? evaluateZucchini(centre, yaw, top_state) :
              evaluateBowl(centre, top_state));
          }
          if (result.success) ++successful_trials;
          output << x << ',' << y << ',' << yaw << ',' << (result.success ? 1 : 0) << ','
                 << result.failed_stage << ',' << result.selected_pregrasp_m << ','
                 << result.selected_grasp_m << ',' << result.selected_grasp_yaw_deg << ','
                 << result.selected_tilt_deg << ',' << result.planning_time_s << '\n';
        }
        output.flush();
        if (point_index == 1 || point_index % 10 == 0 || point_index == point_count) {
          RCLCPP_INFO(
            node_->get_logger(), "%s workspace: %zu/%zu XY points, %zu successful yaw trials",
            object_type_.c_str(), point_index, point_count, successful_trials);
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(), "%s workspace CSV written: %s",
      object_type_.c_str(), output_csv_.c_str());
  }

private:
  void applyStaticScene()
  {
    const auto ground = boxObject(
      "ground", "base_link", Eigen::Vector3d(0.3, 0.0, ground_z_ - 0.01),
      Eigen::Vector3d(2.0, 2.0, 0.02));
    if (!scene_->processCollisionObjectMsg(ground)) {
      throw std::runtime_error("failed to add ground collision");
    }
    moveit_msgs::msg::AttachedCollisionObject go2;
    go2.link_name = "base_link";
    go2.touch_links = {"base_link"};
    go2.object = boxObject(
      "go2_platform", "base_link", Eigen::Vector3d(-0.044763, 0.0, -0.096688269402),
      Eigen::Vector3d(0.753442, 0.338254, 0.255121));
    if (!scene_->processAttachedCollisionObjectMsg(go2)) {
      throw std::runtime_error("failed to attach Go2 collision box");
    }
  }

  bool stateValid(moveit::core::RobotState& state, const planning_scene::PlanningSceneConstPtr& scene) const
  {
    state.update();
    return state.satisfiesBounds(arm_) && !scene->isStateColliding(state, "arm", false);
  }

  bool solveIk(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& seed,
    const Eigen::Isometry3d& target,
    const std::string& tip,
    moveit::core::RobotState& solution) const
  {
    solution = seed;
    const moveit::core::GroupStateValidityCallbackFn callback =
      [this, &scene](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
      const double* values)
      {
        state->setJointGroupPositions(group, values);
        return stateValid(*state, scene);
      };
    return solution.setFromIK(arm_, target, tip, ik_timeout_s_, callback);
  }

  bool planBetween(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& start,
    const moveit::core::RobotState& goal,
    moveit::core::RobotState& reached,
    double& elapsed) const
  {
    planning_interface::MotionPlanRequest request;
    request.group_name = "arm";
    request.planner_id = "RRTConnectkConfigDefault";
    request.allowed_planning_time = planning_time_s_;
    request.num_planning_attempts = 1;
    moveit::core::robotStateToRobotStateMsg(start, request.start_state);
    request.start_state.is_diff = true;
    request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal, arm_, 1e-3));
    planning_interface::MotionPlanResponse response;
    const bool generated = pipeline_->generatePlan(scene, request, response);
    elapsed += response.planning_time_;
    if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      !response.trajectory_ || response.trajectory_->empty())
    {
      return false;
    }
    reached = response.trajectory_->getLastWayPoint();
    return true;
  }

  bool findObservation(
    const Eigen::Vector3d& target,
    moveit::core::RobotState& reached,
    std::string& failure)
  {
    std::vector<d1_manipulation::ObservationCandidate> candidates;
    try {
      candidates = d1_manipulation::generateObservationCandidates(
        target, Eigen::Vector3d::UnitZ(), beta_degrees_, alpha_degrees_, observation_distances_);
    } catch (const std::exception&) {
      failure = "OBSERVE_GEOMETRY";
      return false;
    }
    for (const auto& candidate : candidates) {
      Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
      world_from_camera.linear() = candidate.planning_from_camera;
      world_from_camera.translation() = candidate.camera_position;
      const Eigen::Isometry3d world_from_link6 = world_from_camera * link6_from_camera_.inverse();
      moveit::core::RobotState goal(model_);
      if (!solveIk(scene_, *stowed_state_, world_from_link6, "Link6", goal)) continue;
      reached = goal;
      return true;
    }
    failure = "PLAN_OBSERVE";
    return false;
  }

  bool findTopObservation(
    const Eigen::Vector3d& target,
    const moveit::core::RobotState& start,
    moveit::core::RobotState& reached,
    std::string& failure)
  {
    const Eigen::Vector3d camera_z = -Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d reference = Eigen::Vector3d::UnitX();
    for (const double distance : top_distances_) {
      for (const double roll_deg : top_rolls_) {
        const Eigen::Vector3d image_up =
          Eigen::AngleAxisd(roll_deg * kPi / 180.0, camera_z) * reference;
        const Eigen::Vector3d camera_y = -image_up;
        const Eigen::Vector3d camera_x = camera_y.cross(camera_z).normalized();
        Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
        world_from_camera.linear().col(0) = camera_x;
        world_from_camera.linear().col(1) = camera_y;
        world_from_camera.linear().col(2) = camera_z;
        world_from_camera.translation() = target + distance * Eigen::Vector3d::UnitZ();
        const Eigen::Isometry3d world_from_link6 = world_from_camera * link6_from_camera_.inverse();
        moveit::core::RobotState goal(model_);
        if (!solveIk(scene_, start, world_from_link6, "Link6", goal)) continue;
        reached = goal;
        return true;
      }
    }
    failure = "PLAN_TOP_OBSERVE";
    return false;
  }

  bool cartesianPath(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& start,
    const Eigen::Isometry3d& target,
    moveit::core::RobotState& reached,
    std::vector<moveit::core::RobotState>& states) const
  {
    moveit::core::RobotState updated_start = start;
    updated_start.update();
    const Eigen::Isometry3d source = updated_start.getGlobalLinkTransform("tcp_link");
    const double linear_distance = (target.translation() - source.translation()).norm();
    const int requested_steps = std::max(1, static_cast<int>(std::ceil(linear_distance / cartesian_step_)));
    states.clear();
    states.push_back(updated_start);
    moveit::core::RobotState current = updated_start;
    int completed = 0;
    for (int index = 1; index <= requested_steps; ++index) {
      const double ratio = static_cast<double>(index) / requested_steps;
      Eigen::Isometry3d waypoint = Eigen::Isometry3d::Identity();
      waypoint.translation() = (1.0 - ratio) * source.translation() + ratio * target.translation();
      waypoint.linear() = Eigen::Quaterniond(source.rotation()).slerp(
        ratio, Eigen::Quaterniond(target.rotation())).normalized().toRotationMatrix();
      moveit::core::RobotState next(model_);
      if (!solveIk(scene, current, waypoint, "tcp_link", next)) break;
      states.push_back(next);
      current = next;
      ++completed;
    }
    const double fraction = static_cast<double>(completed) / requested_steps;
    if (fraction + 1e-9 < minimum_cartesian_fraction_) return false;
    reached = current;
    return true;
  }

  planning_scene::PlanningScenePtr sceneWithHeldCube(
    const Eigen::Vector3d& centre,
    double object_yaw_deg,
    const Eigen::Isometry3d& grasp,
    const moveit::core::RobotState& lift_state) const
  {
    auto held_scene = scene_->diff();
    held_scene->setCurrentState(lift_state);
    Eigen::Isometry3d world_from_object = Eigen::Isometry3d::Identity();
    world_from_object.translation() = centre;
    world_from_object.linear() = Eigen::AngleAxisd(
      object_yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Isometry3d tcp_from_object = grasp.inverse() * world_from_object;

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "tcp_link";
    attached.touch_links = {"tcp_link", "Link6", "left_finger", "right_finger"};
    attached.object.header.frame_id = "tcp_link";
    attached.object.id = "held/yellow_cube";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {cube_size_, cube_size_, cube_size_};
    attached.object.primitives.push_back(box);
    attached.object.primitive_poses.push_back(poseMessage(tcp_from_object));
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!held_scene->processAttachedCollisionObjectMsg(attached)) {
      return {};
    }
    return held_scene;
  }

  planning_scene::PlanningScenePtr sceneWithHeldObject(
    const Eigen::Isometry3d& world_from_object,
    const Eigen::Isometry3d& grasp,
    const moveit::core::RobotState& lift_state) const
  {
    auto held_scene = scene_->diff();
    held_scene->setCurrentState(lift_state);
    const Eigen::Isometry3d tcp_from_object = grasp.inverse() * world_from_object;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "tcp_link";
    attached.touch_links = {"tcp_link", "Link6", "left_finger", "right_finger"};
    attached.object.header.frame_id = "tcp_link";
    attached.object.id = "held/" + object_type_;
    if (object_type_ == "zucchini") {
      attached.object.meshes.push_back(ellipsoidMesh(
        0.5 * zucchini_length_, 0.5 * zucchini_width_, 0.5 * zucchini_height_));
      attached.object.mesh_poses.push_back(poseMessage(tcp_from_object));
    } else if (object_type_ == "bowl") {
      auto append_primitive = [&attached, &tcp_from_object](
        shape_msgs::msg::SolidPrimitive primitive,
        const Eigen::Isometry3d& model_from_primitive)
        {
          attached.object.primitives.push_back(std::move(primitive));
          attached.object.primitive_poses.push_back(
            poseMessage(tcp_from_object * model_from_primitive));
        };
      constexpr double center_x = -0.00023;
      constexpr double center_y = -0.00005;
      shape_msgs::msg::SolidPrimitive bottom;
      bottom.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      bottom.dimensions = {0.008, 0.0405};
      Eigen::Isometry3d model_from_bottom = Eigen::Isometry3d::Identity();
      model_from_bottom.translation() = Eigen::Vector3d(center_x, center_y, 0.004);
      append_primitive(bottom, model_from_bottom);
      constexpr int segment_count = 12;
      constexpr double bottom_radius = 0.039;
      constexpr double top_radius = 0.058;
      constexpr double bottom_z = 0.004;
      constexpr double top_z = 0.049;
      const double radial_delta = top_radius - bottom_radius;
      const double vertical_delta = top_z - bottom_z;
      const double slope = std::atan2(radial_delta, vertical_delta);
      const double mid_radius = 0.5 * (bottom_radius + top_radius);
      const double mid_z = 0.5 * (bottom_z + top_z);
      const double slant_half = 0.5 * std::hypot(radial_delta, vertical_delta);
      const double tangent_half =
        mid_radius * std::tan(kPi / static_cast<double>(segment_count)) + 0.0008;
      for (int index = 0; index < segment_count; ++index) {
        const double theta = 2.0 * kPi * index / segment_count;
        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        const Eigen::Vector3d tangent(-sin_theta, cos_theta, 0.0);
        const Eigen::Vector3d normal(
          -std::cos(slope) * cos_theta,
          -std::cos(slope) * sin_theta,
          std::sin(slope));
        Eigen::Isometry3d model_from_wall = Eigen::Isometry3d::Identity();
        model_from_wall.translation() = Eigen::Vector3d(
          center_x + mid_radius * cos_theta,
          center_y + mid_radius * sin_theta,
          mid_z);
        model_from_wall.linear().col(0) = tangent;
        model_from_wall.linear().col(1) = normal;
        model_from_wall.linear().col(2) = tangent.cross(normal).normalized();
        shape_msgs::msg::SolidPrimitive wall;
        wall.type = shape_msgs::msg::SolidPrimitive::BOX;
        wall.dimensions = {2.0 * tangent_half, 0.004, 2.0 * slant_half};
        append_primitive(wall, model_from_wall);
      }
    }
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!held_scene->processAttachedCollisionObjectMsg(attached)) return {};
    return held_scene;
  }

  bool evaluateCandidate(
    const moveit::core::RobotState& top_state,
    const Eigen::Isometry3d& pregrasp,
    const Eigen::Isometry3d& grasp,
    const Eigen::Isometry3d& world_from_object,
    double closed_gripper,
    TrialResult& result,
    bool& terminal_failure)
  {
    terminal_failure = false;
    moveit::core::RobotState pregrasp_goal(model_);
    if (!solveIk(scene_, top_state, pregrasp, "tcp_link", pregrasp_goal)) return false;
    moveit::core::RobotState preliminary_grasp(model_);
    std::vector<moveit::core::RobotState> cartesian_states;
    if (!cartesianPath(scene_, pregrasp_goal, grasp, preliminary_grasp, cartesian_states)) {
      return false;
    }
    moveit::core::RobotState pregrasp_reached(model_);
    if (!planBetween(
        scene_, top_state, pregrasp_goal, pregrasp_reached, result.planning_time_s))
    {
      return false;
    }
    moveit::core::RobotState grasp_state(model_);
    if (!cartesianPath(scene_, pregrasp_reached, grasp, grasp_state, cartesian_states)) {
      return false;
    }
    Eigen::Isometry3d lift = grasp;
    lift.translation() += lift_distance_ * Eigen::Vector3d::UnitZ();
    moveit::core::RobotState lift_state(model_);
    if (!cartesianPath(scene_, grasp_state, lift, lift_state, cartesian_states)) return false;
    auto held_scene = sceneWithHeldObject(world_from_object, grasp, lift_state);
    if (!held_scene) {
      result.failed_stage = "ATTACH_OBJECT";
      terminal_failure = true;
      return false;
    }
    moveit::core::RobotState held_lift = held_scene->getCurrentState();
    held_lift.setVariablePosition("Joint6", closed_gripper);
    held_lift.update();
    moveit::core::RobotState held_carry = held_lift;
    held_carry.setJointGroupPositions(arm_, carry_);
    held_carry.setVariablePosition("Joint6", closed_gripper);
    held_carry.update();
    if (!stateValid(held_lift, held_scene)) {
      result.failed_stage = "LIFT_WITH_OBJECT_COLLISION";
      terminal_failure = true;
      return false;
    }
    if (!stateValid(held_carry, held_scene)) {
      result.failed_stage = "CARRY_GOAL_COLLISION";
      terminal_failure = true;
      return false;
    }
    moveit::core::RobotState carry_reached(model_);
    if (!planBetween(
        held_scene, held_lift, held_carry, carry_reached, result.planning_time_s))
    {
      result.failed_stage = "PLAN_CARRY";
      terminal_failure = true;
      return false;
    }
    return true;
  }

  TrialResult evaluateZucchini(
    const Eigen::Vector3d& centre,
    double object_yaw_deg,
    const moveit::core::RobotState& top_state)
  {
    TrialResult result;
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d axis(
      std::cos(object_yaw_deg * kPi / 180.0),
      std::sin(object_yaw_deg * kPi / 180.0), 0.0);
    Eigen::Vector3d grasp_point = centre;
    grasp_point.z() = ground_z_ + zucchini_tcp_ground_clearance_;
    for (const double pregrasp_distance : pregrasp_distances_) {
      for (const double direction_sign : {1.0, -1.0}) {
        const Eigen::Vector3d closing = direction_sign * up.cross(axis).normalized();
        const Eigen::Vector3d z = -up;
        const Eigen::Vector3d y = closing;
        const Eigen::Vector3d x = y.cross(z).normalized();
        Eigen::Matrix3d vertical;
        vertical.col(0) = x;
        vertical.col(1) = y;
        vertical.col(2) = z;
        Eigen::Isometry3d world_from_object = Eigen::Isometry3d::Identity();
        world_from_object.translation() = centre;
        world_from_object.linear().col(0) = axis;
        world_from_object.linear().col(1) = closing;
        world_from_object.linear().col(2) = up;
        for (const double tilt_deg : {2.0, -2.0, 4.0, -4.0}) {
          Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();
          grasp.linear() = vertical * Eigen::AngleAxisd(
            tilt_deg * kPi / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
          grasp.translation() = grasp_point;
          Eigen::Isometry3d pregrasp = grasp;
          pregrasp.translation() += pregrasp_distance * up;
          bool terminal = false;
          if (evaluateCandidate(
              top_state, pregrasp, grasp, world_from_object, 0.01, result, terminal))
          {
            result.success = true;
            result.failed_stage = "NONE";
            result.selected_pregrasp_m = pregrasp_distance;
            result.selected_grasp_m = zucchini_tcp_ground_clearance_;
            result.selected_tilt_deg = tilt_deg;
            result.selected_grasp_yaw_deg = std::atan2(closing.y(), closing.x()) * 180.0 / kPi;
            return result;
          }
          if (terminal) return result;
        }
      }
    }
    result.failed_stage = "PREGRASP_DESCENT_OR_CARRY";
    return result;
  }

  TrialResult evaluateBowl(
    const Eigen::Vector3d& centre,
    const moveit::core::RobotState& top_state)
  {
    TrialResult result;
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d bottom(centre.x(), centre.y(), ground_z_);
    Eigen::Vector3d y = -bottom;
    y -= y.dot(up) * up;
    if (y.norm() < 1e-6) y = Eigen::Vector3d::UnitX();
    y.normalize();
    const Eigen::Vector3d x = y.cross(up).normalized();
    Eigen::Isometry3d world_from_model = Eigen::Isometry3d::Identity();
    world_from_model.translation() = bottom;
    world_from_model.linear().col(0) = x;
    world_from_model.linear().col(1) = y;
    world_from_model.linear().col(2) = up;
    Eigen::Isometry3d model_from_tcp = Eigen::Isometry3d::Identity();
    model_from_tcp.linear() = Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
    model_from_tcp.translation() = Eigen::Vector3d(0.0, 0.040684673935, 0.004748903215);
    for (const double pregrasp_distance : pregrasp_distances_) {
      for (const double azimuth_deg : orderedBowlAzimuths()) {
        for (const double flip_deg : {0.0, 180.0}) {
          const Eigen::Isometry3d grasp =
            world_from_model *
            Eigen::AngleAxisd(azimuth_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()) *
            model_from_tcp *
            Eigen::AngleAxisd(flip_deg * kPi / 180.0, Eigen::Vector3d::UnitZ());
          Eigen::Isometry3d pregrasp = grasp;
          pregrasp.translation() += pregrasp_distance * up;
          bool terminal = false;
          if (evaluateCandidate(
              top_state, pregrasp, grasp, world_from_model, 0.0, result, terminal))
          {
            result.success = true;
            result.failed_stage = "NONE";
            result.selected_pregrasp_m = pregrasp_distance;
            result.selected_grasp_m = 0.0;
            result.selected_tilt_deg = flip_deg;
            result.selected_grasp_yaw_deg = azimuth_deg;
            return result;
          }
          if (terminal) return result;
        }
      }
    }
    result.failed_stage = "PREGRASP_DESCENT_OR_CARRY";
    return result;
  }

  TrialResult evaluateYaw(
    const Eigen::Vector3d& centre,
    double object_yaw_deg,
    const moveit::core::RobotState& top_state)
  {
    TrialResult result;
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d edge(
      std::cos(object_yaw_deg * kPi / 180.0),
      std::sin(object_yaw_deg * kPi / 180.0), 0.0);
    const Eigen::Vector3d top_center = centre + 0.5 * cube_size_ * up;

    for (const double pregrasp_distance : pregrasp_distances_) {
      for (const int quarter_turn : {0, 1, -1, 2}) {
        const Eigen::Vector3d x = Eigen::AngleAxisd(quarter_turn * kPi / 2.0, up) * edge;
        const Eigen::Vector3d z = -up;
        const Eigen::Vector3d y = z.cross(x).normalized();
        Eigen::Matrix3d vertical;
        vertical.col(0) = x;
        vertical.col(1) = y;
        vertical.col(2) = z;
        for (const double tilt_deg : {2.0, -2.0, 4.0, -4.0}) {
          Eigen::Isometry3d pregrasp = Eigen::Isometry3d::Identity();
          pregrasp.linear() = vertical * Eigen::AngleAxisd(
            tilt_deg * kPi / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
          pregrasp.translation() = top_center + pregrasp_distance * up;
          moveit::core::RobotState pregrasp_goal(model_);
          if (!solveIk(scene_, top_state, pregrasp, "tcp_link", pregrasp_goal)) continue;
          for (const double grasp_distance : grasp_distances_) {
            Eigen::Isometry3d grasp = pregrasp;
            grasp.translation() = top_center + grasp_distance * up;
            moveit::core::RobotState grasp_state(model_);
            std::vector<moveit::core::RobotState> descent_states;
            if (!cartesianPath(
                scene_, pregrasp_goal, grasp, grasp_state, descent_states))
            {
              continue;
            }

            moveit::core::RobotState pregrasp_reached(model_);
            if (!planBetween(
                scene_, top_state, pregrasp_goal, pregrasp_reached, result.planning_time_s))
            {
              continue;
            }
            // Recompute from the actual OMPL terminal state, matching the
            // production strategy's downstream Cartesian confirmation.
            if (!cartesianPath(
                scene_, pregrasp_reached, grasp, grasp_state, descent_states))
            {
              continue;
            }

            const double lift_target = std::min(
              lift_distance_, pregrasp_distance - grasp_distance);
            const Eigen::Vector3d lift_point = grasp.translation() + lift_target * up;
            std::size_t lift_index = descent_states.size() - 1;
            for (std::size_t index = descent_states.size(); index-- > 0;) {
              const double height =
                (descent_states[index].getGlobalLinkTransform("tcp_link").translation() -
                grasp.translation()).dot(up);
              lift_index = index;
              if (height >= lift_target - 0.5 * cartesian_step_) break;
            }
            moveit::core::RobotState lift_state = descent_states[lift_index];
            if ((lift_state.getGlobalLinkTransform("tcp_link").translation() - lift_point).norm() >
              1.5 * cartesian_step_)
            {
              continue;
            }
            auto held_scene = sceneWithHeldCube(
              centre, object_yaw_deg, grasp, lift_state);
            if (!held_scene) {
              result.failed_stage = "ATTACH_OBJECT";
              return result;
            }
            moveit::core::RobotState held_lift = held_scene->getCurrentState();
            held_lift.update();
            moveit::core::RobotState held_carry = held_lift;
            held_carry.setJointGroupPositions(arm_, carry_);
            held_carry.setVariablePosition("Joint6", 0.025);
            held_carry.update();
            if (!stateValid(held_lift, held_scene)) {
              result.failed_stage = "LIFT_WITH_OBJECT_COLLISION";
              return result;
            }
            if (!stateValid(held_carry, held_scene)) {
              result.failed_stage = "CARRY_GOAL_COLLISION";
              return result;
            }
            moveit::core::RobotState carry_reached(model_);
            if (!planBetween(
                held_scene, held_lift, held_carry, carry_reached, result.planning_time_s))
            {
              result.failed_stage = "PLAN_CARRY";
              return result;
            }
            result.success = true;
            result.failed_stage = "NONE";
            result.selected_pregrasp_m = pregrasp_distance;
            result.selected_grasp_m = grasp_distance;
            result.selected_tilt_deg = tilt_deg;
            result.selected_grasp_yaw_deg = std::atan2(x.y(), x.x()) * 180.0 / kPi;
            return result;
          }
        }
      }
    }
    result.failed_stage = "PREGRASP_DESCENT_OR_CARRY";
    return result;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup* arm_{nullptr};
  planning_scene::PlanningScenePtr scene_;
  planning_pipeline::PlanningPipelinePtr pipeline_;
  moveit::core::RobotStatePtr stowed_state_;
  moveit::core::RobotStatePtr carry_state_;
  Eigen::Isometry3d link6_from_camera_{Eigen::Isometry3d::Identity()};
  std::string object_type_, output_csv_;
  double x_min_{}, x_max_{}, y_min_{}, y_max_{}, xy_step_{}, yaw_step_deg_{};
  double ground_z_{}, cube_size_{}, zucchini_length_{}, zucchini_width_{}, zucchini_height_{};
  double zucchini_tcp_ground_clearance_{}, bowl_radius_{}, bowl_height_{};
  double ik_timeout_s_{}, planning_time_s_{};
  double cartesian_step_{}, minimum_cartesian_fraction_{}, lift_distance_{};
  std::vector<double> beta_degrees_, alpha_degrees_, observation_distances_;
  std::vector<double> top_distances_, top_rolls_, pregrasp_distances_, grasp_distances_;
  std::vector<double> stowed_, carry_;
};

}  // namespace d1_workspace_analysis

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>(
      "d1_grasp_workspace_scan",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    d1_workspace_analysis::GraspWorkspaceScan scanner(node);
    scanner.run();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("d1_grasp_workspace_scan"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
