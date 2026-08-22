#include "d1_manipulation/pick_strategy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <utility>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "d1_manipulation/action/pick_object.hpp"
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

struct ZucchiniState final : PickStrategyState
{
  Eigen::Vector3d grasp_point{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d grasp_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d object_rotation{Eigen::Matrix3d::Identity()};
  std::vector<std::string> target_collision_ids;
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
    pregrasp_max_ = parameterOrDeclare(node_, "zucchini_pregrasp_distance_max_m", 0.080);
    pregrasp_min_ = parameterOrDeclare(node_, "zucchini_pregrasp_distance_min_m", 0.015);
    pregrasp_step_ = parameterOrDeclare(node_, "zucchini_pregrasp_distance_step_m", 0.005);
    tcp_ground_clearance_ = parameterOrDeclare(
      node_, "zucchini_tcp_ground_clearance_m", 0.015);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_closed_ = parameterOrDeclare(node_, "zucchini_gripper_closed_m", 0.01);
    gripper_held_threshold_ = parameterOrDeclare(
      node_, "zucchini_gripper_held_threshold_m", 0.011);
    grasp_settle_ = parameterOrDeclare(node_, "grasp_settle_s", 0.5);
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    if (length_ <= 0.0 || width_ <= 0.0 || height_ <= 0.0 ||
      pregrasp_min_ < 0.0 || pregrasp_max_ < pregrasp_min_ || pregrasp_step_ <= 0.0 ||
      tcp_ground_clearance_ < 0.0 || gripper_closed_ < 0.0 ||
      gripper_held_threshold_ <= gripper_closed_ || gripper_held_threshold_ > gripper_open_)
    {
      throw std::invalid_argument("invalid zucchini geometry or grasp search parameters");
    }
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
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", coarse ? coarse->detail : "zucchini coarse estimation unavailable"};
      return false;
    }
    Eigen::Vector3d up(
      coarse->ground_normal.x, coarse->ground_normal.y, coarse->ground_normal.z);
    up.normalize();
    const Eigen::Vector3d coarse_center(
      coarse->center.point.x, coarse->center.point.y, coarse->center.point.z);
    if (!runtime_.applyEstimatedGround(up, coarse->ground_offset)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply perception-fitted ground to MoveIt"};
      return false;
    }
    if (!runtime_.moveCameraTopDown(coarse_center, up)) {
      failure = {action::PickObject::Result::FAILURE_THEORETICALLY_INFEASIBLE,
        "MOVE_TOP_OBSERVE", "top observation pose is not plannable"};
      return false;
    }
    const auto fine = estimate(
      srv::EstimateZucchini::Request::FINE, coarse->center, up, coarse->ground_offset);
    if (!fine || !fine->success) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", fine ? fine->detail : "zucchini fine estimation unavailable"};
      return false;
    }

    up = Eigen::Vector3d(
      fine->ground_normal.x, fine->ground_normal.y, fine->ground_normal.z);
    if (!up.allFinite() || up.norm() < 0.9) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "zucchini fine ground normal is invalid"};
      return false;
    }
    up.normalize();
    if (!runtime_.applyEstimatedGround(up, fine->ground_offset)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply fine-observation ground to MoveIt"};
      return false;
    }

    const Eigen::Vector3d center(
      fine->center.point.x, fine->center.point.y, fine->center.point.z);
    Eigen::Vector3d axis(
      fine->axis_direction.x, fine->axis_direction.y, fine->axis_direction.z);
    axis = (axis - axis.dot(up) * up).normalized();
    if (!axis.allFinite()) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", "zucchini local axis is invalid"};
      return false;
    }
    // The zucchini has no reliable planar top surface. Preserve the
    // fine-estimated tangent-plane position, but derive TCP height directly
    // from the ground plane refitted during the fine observation.
    const double center_ground_distance = up.dot(center) + fine->ground_offset;
    const Eigen::Vector3d grasp_point =
      center + (tcp_ground_clearance_ - center_ground_distance) * up;
    RCLCPP_INFO(node_->get_logger(),
      "Zucchini fine geometry: center=(%.3f, %.3f, %.3f) axis=(%.3f, %.3f, %.3f) "
      "visible=(%.3f x %.3f) tcp_ground_clearance=%.0f mm",
      center.x(), center.y(), center.z(), axis.x(), axis.y(), axis.z(),
      fine->visible_length_m, fine->visible_width_m,
      1000.0 * tcp_ground_clearance_);
    const std::vector<std::string> target_ids{"zucchini", "observe_target"};
    const auto target_objects = runtime_.planningScene().getObjects(target_ids);
    const auto pregrasp_distances = descending(pregrasp_max_, pregrasp_min_, pregrasp_step_);

    geometry_msgs::msg::Pose grasp, pregrasp;
    moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
    Eigen::Matrix3d selected_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d selected_closing = up.cross(axis).normalized();
    double selected_pregrasp = 0.0, selected_grasp = 0.0;
    double selected_yaw = 0.0, selected_tilt = 0.0;
    bool found = false;
    auto& move_group = runtime_.moveGroup();
    for (const double pregrasp_distance : pregrasp_distances) {
      for (const double direction_sign : {1.0, -1.0}) {
        const Eigen::Vector3d closing = direction_sign * up.cross(axis).normalized();
        const Eigen::Vector3d z = -up;
        // tcp_link inherits Link6 axes. The line joining the two fingertips
        // (and therefore the gripper closing axis) is TCP +Y, not TCP +X.
        // Keep TCP +Z downward and assign the desired closing direction to +Y.
        const Eigen::Vector3d y = closing;
        const Eigen::Vector3d x = y.cross(z).normalized();
        Eigen::Matrix3d vertical;
        vertical.col(0) = x; vertical.col(1) = y; vertical.col(2) = z;
        for (const double tilt_deg : {2.0, -2.0, 4.0, -4.0}) {
          Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
          transform.linear() = vertical *
            Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
          transform.translation() = grasp_point + pregrasp_distance * up;
          pregrasp = poseMessage(transform);
          move_group.setEndEffectorLink(runtime_.tcpFrame());
          move_group.setStartStateToCurrentState();
          if (!move_group.setJointValueTarget(pregrasp, runtime_.tcpFrame())) continue;
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) continue;
          if (!runtime_.removeTargetCollision(target_ids)) {
            failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
              "PLAN_PREGRASP", "target collision removal did not reach the planning scene"};
            return false;
          }
          transform.translation() = grasp_point;
          moveit_msgs::msg::RobotTrajectory descent;
          if (runtime_.computeCartesianFromPlanEnd(
              plan, poseMessage(transform), true, descent) >= runtime_.minimumCartesianFraction())
          {
            grasp = poseMessage(transform); selected_plan = plan;
            selected_rotation = transform.rotation(); selected_closing = closing;
            selected_pregrasp = pregrasp_distance;
            selected_grasp = tcp_ground_clearance_;
            selected_yaw = std::atan2(closing.y(), closing.x()) * 180.0 / M_PI;
            selected_tilt = tilt_deg; found = true;
          }
          if (!runtime_.restoreTargetCollision(target_objects)) {
            failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
              "PLAN_PREGRASP", "target collision restoration did not reach the planning scene"};
            return false;
          }
          if (found) break;
        }
        if (found) break;
      }
      if (found) break;
    }
    if (!found) {
      failure = {action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
        "PLAN_PREGRASP", "no zucchini pregrasp has a feasible downstream grasp; reposition Go2"};
      return false;
    }

    output.class_name = class_name_; output.estimated_center = fine->center;
    output.grasp_pose = grasp; output.pregrasp_pose = pregrasp;
    output.pregrasp_plan = std::move(selected_plan); output.lift_direction = up;
    output.pregrasp_distance_m = selected_pregrasp;
    output.grasp_distance_m = selected_grasp;
    output.grasp_yaw_degrees = selected_yaw;
    output.approach_tilt_degrees = selected_tilt;
    output.gripper_open_m = gripper_open_; output.gripper_closed_m = gripper_closed_;
    output.gripper_held_threshold_m = gripper_held_threshold_;
    output.grasp_settle_s = grasp_settle_; output.lift_distance_m = lift_distance_;
    auto state = std::make_shared<ZucchiniState>();
    state->grasp_point = grasp_point; state->grasp_rotation = selected_rotation;
    state->object_rotation.col(0) = axis;
    state->object_rotation.col(1) = selected_closing;
    state->object_rotation.col(2) = up;
    state->target_collision_ids = target_ids;
    output.strategy_state = std::move(state);
    publishMarkers(*coarse, *fine, output, axis, selected_closing);
    RCLCPP_INFO(node_->get_logger(),
      "Zucchini strategy selected pregrasp=%+.0f mm tcp_ground_clearance=%.0f mm "
      "yaw=%.1f deg tilt=%.1f deg",
      1000.0 * selected_pregrasp, 1000.0 * selected_grasp, selected_yaw, selected_tilt);
    return true;
  }

  bool confirmDescent(PreparedPick& plan, StrategyFailure& failure) override
  {
    const auto state = std::dynamic_pointer_cast<ZucchiniState>(plan.strategy_state);
    if (!state) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "zucchini strategy state is missing"};
      return false;
    }
    if (!runtime_.removeTargetCollision(state->target_collision_ids)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "target collision removal did not reach the planning scene"};
      return false;
    }
    auto& move_group = runtime_.moveGroup();
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = state->grasp_rotation;
    transform.translation() = state->grasp_point;
    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory descent;
    if (move_group.computeCartesianPath(
        {poseMessage(transform)}, runtime_.cartesianStep(), 0.0, descent, true) >=
      runtime_.minimumCartesianFraction())
    {
      plan.grasp_pose = poseMessage(transform);
      plan.grasp_distance_m = tcp_ground_clearance_;
      plan.descent_trajectory = std::move(descent);
      return true;
    }
    failure = {action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
      "DESCEND", "ground-referenced zucchini grasp is unreachable from pregrasp; reposition Go2"};
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
  double pregrasp_max_{}, pregrasp_min_{}, pregrasp_step_{};
  double tcp_ground_clearance_{};
  double gripper_open_{}, gripper_closed_{}, gripper_held_threshold_{};
  double grasp_settle_{}, lift_distance_{};
};

std::unique_ptr<PickStrategy> makeZucchiniPickStrategy(
  const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
{
  return std::make_unique<ZucchiniPickStrategy>(node, runtime);
}

}  // namespace d1_manipulation
