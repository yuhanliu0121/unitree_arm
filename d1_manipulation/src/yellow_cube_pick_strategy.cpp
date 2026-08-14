#include "d1_manipulation/pick_strategy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <utility>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

struct YellowCubeState final : PickStrategyState
{
  Eigen::Vector3d top_center{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d grasp_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d object_rotation{Eigen::Matrix3d::Identity()};
  std::vector<double> grasp_distances;
  std::vector<std::string> target_collision_ids;
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
    if (cube_size_ <= 0.0 || pregrasp_min_ < 0.0 || pregrasp_max_ < pregrasp_min_ ||
      pregrasp_step_ <= 0.0 || grasp_min_ > grasp_max_ || grasp_max_ >= 0.0 ||
      grasp_step_ <= 0.0 || gripper_closed_ < 0.0 ||
      gripper_held_threshold_ <= gripper_closed_ || gripper_held_threshold_ > gripper_open_)
    {
      throw std::invalid_argument("invalid signed cube pregrasp/grasp search parameters");
    }
    estimate_client_ = node_->create_client<srv::EstimateCube>(estimate_name_);
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm/debug/cube_grasp_markers", rclcpp::QoS(1).transient_local().reliable());
  }

  const std::string& className() const override { return class_name_; }

  bool prepare(const geometry_msgs::msg::PointStamped& hint, PreparedPick& output,
    StrategyFailure& failure) override
  {
    const auto coarse = estimate(srv::EstimateCube::Request::COARSE, hint);
    if (!coarse || !coarse->success) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", coarse ? coarse->detail : "cube coarse estimation unavailable"};
      return false;
    }
    Eigen::Vector3d up(
      coarse->ground_normal.x, coarse->ground_normal.y, coarse->ground_normal.z);
    Eigen::Vector3d coarse_center(
      coarse->center.point.x, coarse->center.point.y, coarse->center.point.z);
    if (!runtime_.applyEstimatedGround(up, coarse->ground_offset)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply perception-fitted ground to MoveIt"};
      return false;
    }
    if (!runtime_.moveCameraTopDown(coarse_center, up.normalized())) {
      failure = {action::PickObject::Result::FAILURE_THEORETICALLY_INFEASIBLE,
        "MOVE_TOP_OBSERVE", "top observation pose is not plannable"};
      return false;
    }
    const auto fine = estimate(
      srv::EstimateCube::Request::FINE, coarse->center, up.normalized(), coarse->ground_offset);
    if (!fine || !fine->success) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", fine ? fine->detail : "cube fine estimation unavailable"};
      return false;
    }

    Eigen::Vector3d center(fine->center.point.x, fine->center.point.y, fine->center.point.z);
    Eigen::Vector3d edge(
      fine->edge_direction.x, fine->edge_direction.y, fine->edge_direction.z);
    up.normalize(); edge = (edge - edge.dot(up) * up).normalized();
    const Eigen::Vector3d top_center = center + 0.5 * cube_size_ * up;
    const std::vector<std::string> target_ids{"yellow_cube", "observe_target"};
    const auto target_objects = runtime_.planningScene().getObjects(target_ids);
    const auto pregrasp_distances = descending(pregrasp_max_, pregrasp_min_, pregrasp_step_);
    const auto grasp_distances = ascending(grasp_min_, grasp_max_, grasp_step_);

    geometry_msgs::msg::Pose grasp, pregrasp;
    moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
    Eigen::Matrix3d selected_rotation = Eigen::Matrix3d::Identity();
    double selected_pregrasp = 0.0, selected_grasp = 0.0;
    double selected_yaw = 0.0, selected_tilt = 0.0;
    bool found = false;
    auto& move_group = runtime_.moveGroup();
    for (const double pregrasp_distance : pregrasp_distances) {
      for (const int quarter_turn : {0, 1, -1, 2}) {
        const Eigen::Vector3d x = Eigen::AngleAxisd(quarter_turn * M_PI_2, up) * edge;
        const Eigen::Vector3d z = -up;
        const Eigen::Vector3d y = z.cross(x).normalized();
        Eigen::Matrix3d vertical;
        vertical.col(0) = x; vertical.col(1) = y; vertical.col(2) = z;
        for (const double tilt_deg : {2.0, -2.0, 4.0, -4.0}) {
          Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
          transform.linear() = vertical *
            Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitX());
          transform.translation() = top_center + pregrasp_distance * up;
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
          for (const double grasp_distance : grasp_distances) {
            transform.translation() = top_center + grasp_distance * up;
            moveit_msgs::msg::RobotTrajectory descent;
            if (runtime_.computeCartesianFromPlanEnd(
                plan, poseMessage(transform), true, descent) < runtime_.minimumCartesianFraction())
            {
              continue;
            }
            grasp = poseMessage(transform);
            selected_plan = plan; selected_rotation = transform.rotation();
            selected_pregrasp = pregrasp_distance; selected_grasp = grasp_distance;
            selected_yaw = std::atan2(x.y(), x.x()) * 180.0 / M_PI;
            selected_tilt = tilt_deg; found = true; break;
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
        "PLAN_PREGRASP", "no cube pregrasp has a feasible downstream grasp; reposition Go2"};
      return false;
    }

    output.class_name = class_name_;
    output.estimated_center = fine->center;
    output.grasp_pose = grasp; output.pregrasp_pose = pregrasp;
    output.pregrasp_plan = std::move(selected_plan);
    output.lift_direction = up;
    output.pregrasp_distance_m = selected_pregrasp;
    output.grasp_distance_m = selected_grasp;
    output.grasp_yaw_degrees = selected_yaw;
    output.approach_tilt_degrees = selected_tilt;
    output.gripper_open_m = gripper_open_;
    output.gripper_closed_m = gripper_closed_;
    output.gripper_held_threshold_m = gripper_held_threshold_;
    output.grasp_settle_s = grasp_settle_;
    output.lift_distance_m = lift_distance_;
    auto state = std::make_shared<YellowCubeState>();
    state->top_center = top_center;
    state->grasp_rotation = selected_rotation;
    state->object_rotation.col(0) = edge;
    state->object_rotation.col(2) = up;
    state->object_rotation.col(1) = up.cross(edge).normalized();
    state->grasp_distances = grasp_distances;
    state->target_collision_ids = target_ids;
    output.strategy_state = std::move(state);
    publishMarkers(*fine, output);
    RCLCPP_INFO(node_->get_logger(),
      "Cube strategy selected pregrasp=%+.0f mm grasp=%+.0f mm yaw=%.1f deg tilt=%.1f deg",
      1000.0 * selected_pregrasp, 1000.0 * selected_grasp, selected_yaw, selected_tilt);
    return true;
  }

  bool confirmDescent(PreparedPick& plan, StrategyFailure& failure) override
  {
    const auto state = std::dynamic_pointer_cast<YellowCubeState>(plan.strategy_state);
    if (!state) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "yellow-cube strategy state is missing"};
      return false;
    }
    if (!runtime_.removeTargetCollision(state->target_collision_ids)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "target collision removal did not reach the planning scene"};
      return false;
    }
    auto& move_group = runtime_.moveGroup();
    for (const double distance : state->grasp_distances) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.linear() = state->grasp_rotation;
      transform.translation() = state->top_center + distance * plan.lift_direction;
      const auto grasp = poseMessage(transform);
      move_group.setStartStateToCurrentState();
      moveit_msgs::msg::RobotTrajectory descent;
      const double fraction = move_group.computeCartesianPath(
        {grasp}, runtime_.cartesianStep(), 0.0, descent, true);
      if (fraction < runtime_.minimumCartesianFraction()) continue;
      plan.grasp_pose = grasp;
      plan.grasp_distance_m = distance;
      plan.descent_trajectory = std::move(descent);
      return true;
    }
    failure = {action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
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

  void publishMarkers(const srv::EstimateCube::Response& estimate, const PreparedPick& plan)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    visualization_msgs::msg::Marker plane;
    plane.header.frame_id = runtime_.planningFrame(); plane.ns = "ground_plane"; plane.id = 0;
    plane.type = visualization_msgs::msg::Marker::CUBE;
    plane.action = visualization_msgs::msg::Marker::ADD;
    Eigen::Vector3d normal(
      estimate.ground_normal.x, estimate.ground_normal.y, estimate.ground_normal.z);
    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), normal);
    plane.pose.orientation.x = q.x(); plane.pose.orientation.y = q.y();
    plane.pose.orientation.z = q.z(); plane.pose.orientation.w = q.w();
    const Eigen::Vector3d on_plane = -estimate.ground_offset * normal;
    plane.pose.position.x = on_plane.x(); plane.pose.position.y = on_plane.y();
    plane.pose.position.z = on_plane.z();
    plane.scale.x = 0.8; plane.scale.y = 0.6; plane.scale.z = 0.003;
    plane.color = color(0.3F, 0.7F, 1.0F, 0.25F);
    array.markers.push_back(plane);
    int id = 1;
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
    for (const auto& point : estimate.top_polygon.polygon.points) {
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  const std::string class_name_{"yellow_cube"};
  std::string estimate_name_;
  double cube_size_{}, pregrasp_max_{}, pregrasp_min_{}, pregrasp_step_{};
  double grasp_min_{}, grasp_max_{}, grasp_step_{};
  double gripper_open_{}, gripper_closed_{}, gripper_held_threshold_{};
  double grasp_settle_{}, lift_distance_{};
};

std::unique_ptr<PickStrategy> makeYellowCubePickStrategy(
  const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
{
  return std::make_unique<YellowCubePickStrategy>(node, runtime);
}

}  // namespace d1_manipulation
