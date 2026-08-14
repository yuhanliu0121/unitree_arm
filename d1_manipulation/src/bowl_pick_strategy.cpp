#include "d1_manipulation/pick_strategy.hpp"

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
#include "d1_manipulation/srv/estimate_bowl.hpp"

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

std::vector<double> orderedAzimuths()
{
  std::vector<double> values{0.0};
  for (int angle = 15; angle <= 165; angle += 15) {
    values.push_back(-static_cast<double>(angle));
    values.push_back(static_cast<double>(angle));
  }
  values.push_back(180.0);
  return values;
}

struct BowlState final : PickStrategyState
{
  Eigen::Vector3d bottom_center{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d world_from_model{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d grasp_transform{Eigen::Isometry3d::Identity()};
  std::vector<std::string> target_collision_ids;
};
}  // namespace

class BowlPickStrategy final : public PickStrategy
{
public:
  BowlPickStrategy(const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
  : node_(node), runtime_(runtime)
  {
    estimate_name_ = parameterOrDeclare(
      node_, "bowl_estimation_service_name", std::string("/arm/perception/estimate_bowl"));
    pregrasp_max_ = parameterOrDeclare(node_, "bowl_pregrasp_distance_max_m", 0.080);
    pregrasp_min_ = parameterOrDeclare(node_, "bowl_pregrasp_distance_min_m", 0.015);
    pregrasp_step_ = parameterOrDeclare(node_, "bowl_pregrasp_distance_step_m", 0.005);
    radius_ = parameterOrDeclare(node_, "bowl_radius_m", 0.058);
    height_ = parameterOrDeclare(node_, "bowl_height_m", 0.05001143);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_closed_ = parameterOrDeclare(node_, "gripper_closed_m", 0.0);
    grasp_settle_ = parameterOrDeclare(node_, "grasp_settle_s", 0.5);
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    if (pregrasp_min_ < 0.0 || pregrasp_max_ < pregrasp_min_ ||
      pregrasp_step_ <= 0.0 || radius_ <= 0.0 || height_ <= 0.0)
    {
      throw std::invalid_argument("invalid bowl geometry or pregrasp search parameters");
    }
    estimate_client_ = node_->create_client<srv::EstimateBowl>(estimate_name_);
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm/debug/bowl_grasp_markers", rclcpp::QoS(1).transient_local().reliable());
  }

  const std::string& className() const override { return class_name_; }

  bool prepare(const geometry_msgs::msg::PointStamped& hint, PreparedPick& output,
    StrategyFailure& failure) override
  {
    const auto coarse = estimate(srv::EstimateBowl::Request::COARSE, hint);
    if (!coarse || !coarse->success) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", coarse ? coarse->detail : "bowl coarse estimation unavailable"};
      return false;
    }
    Eigen::Vector3d up(
      coarse->ground_normal.x, coarse->ground_normal.y, coarse->ground_normal.z);
    up.normalize();
    Eigen::Vector3d coarse_bottom(
      coarse->bottom_center.point.x, coarse->bottom_center.point.y,
      coarse->bottom_center.point.z);
    if (!runtime_.applyEstimatedGround(up, coarse->ground_offset)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "ESTIMATE_POSE", "failed to apply perception-fitted ground to MoveIt"};
      return false;
    }
    if (!runtime_.moveCameraTopDown(coarse_bottom, up)) {
      failure = {action::PickObject::Result::FAILURE_THEORETICALLY_INFEASIBLE,
        "MOVE_TOP_OBSERVE", "top observation pose is not plannable"};
      return false;
    }
    const auto fine = estimate(
      srv::EstimateBowl::Request::FINE, coarse->bottom_center, up, coarse->ground_offset);
    if (!fine || !fine->success) {
      failure = {action::PickObject::Result::FAILURE_INCOMPLETE_INFORMATION,
        "ESTIMATE_POSE", fine ? fine->detail : "bowl fine estimation unavailable"};
      return false;
    }
    const Eigen::Vector3d bottom(
      fine->bottom_center.point.x, fine->bottom_center.point.y,
      fine->bottom_center.point.z);

    // Runtime bowl-model frame: origin at the perceived bottom centre, +Z up,
    // +Y from the bowl toward base_link, and +X = +Y cross +Z.
    Eigen::Vector3d y = -bottom;
    y -= y.dot(up) * up;
    if (y.norm() < 1e-6) {
      y = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(up) * up;
    }
    y.normalize();
    const Eigen::Vector3d x = y.cross(up).normalized();
    Eigen::Isometry3d world_from_model = Eigen::Isometry3d::Identity();
    world_from_model.translation() = bottom;
    world_from_model.linear().col(0) = x;
    world_from_model.linear().col(1) = y;
    world_from_model.linear().col(2) = up;

    // Blender-annotated bowl_model -> tcp_link transform.  The annotation was
    // made with one finger outside and one finger inside at 60 mm jaw opening.
    Eigen::Isometry3d model_from_tcp = Eigen::Isometry3d::Identity();
    model_from_tcp.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    model_from_tcp.translation() = Eigen::Vector3d(0.0, 0.040684673935, 0.004748903215);

    const std::vector<std::string> target_ids{"bowl", "observe_target"};
    const auto target_objects = runtime_.planningScene().getObjects(target_ids);
    auto& move_group = runtime_.moveGroup();
    bool found = false;
    double selected_pregrasp = 0.0, selected_azimuth = 0.0, selected_flip = 0.0;
    geometry_msgs::msg::Pose grasp, pregrasp;
    Eigen::Isometry3d selected_grasp = Eigen::Isometry3d::Identity();
    moveit::planning_interface::MoveGroupInterface::Plan selected_plan;

    for (const double pregrasp_distance : descending()) {
      for (const double azimuth_deg : orderedAzimuths()) {
        for (const double flip_deg : {0.0, 180.0}) {
          const Eigen::Isometry3d grasp_tf =
            world_from_model *
            Eigen::AngleAxisd(azimuth_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
            model_from_tcp *
            Eigen::AngleAxisd(flip_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ());
          Eigen::Isometry3d pregrasp_tf = grasp_tf;
          pregrasp_tf.translation() += pregrasp_distance * up;
          pregrasp = poseMessage(pregrasp_tf);
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
          moveit_msgs::msg::RobotTrajectory descent;
          if (runtime_.computeCartesianFromPlanEnd(
              plan, poseMessage(grasp_tf), true, descent) >= runtime_.minimumCartesianFraction())
          {
            grasp = poseMessage(grasp_tf); selected_grasp = grasp_tf;
            selected_plan = plan; selected_pregrasp = pregrasp_distance;
            selected_azimuth = azimuth_deg; selected_flip = flip_deg; found = true;
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
        "PLAN_PREGRASP", "no bowl-rim candidate has a feasible approach; reposition Go2"};
      return false;
    }

    output.class_name = class_name_; output.estimated_center = fine->bottom_center;
    output.grasp_pose = grasp; output.pregrasp_pose = pregrasp;
    output.pregrasp_plan = std::move(selected_plan); output.lift_direction = up;
    output.pregrasp_distance_m = selected_pregrasp;
    output.grasp_distance_m = 0.0;
    output.grasp_yaw_degrees = selected_azimuth;
    output.approach_tilt_degrees = 0.0;
    output.gripper_open_m = gripper_open_; output.gripper_closed_m = gripper_closed_;
    output.grasp_settle_s = grasp_settle_; output.lift_distance_m = lift_distance_;
    auto state = std::make_shared<BowlState>();
    state->bottom_center = bottom; state->world_from_model = world_from_model;
    state->grasp_transform = selected_grasp;
    state->target_collision_ids = target_ids; output.strategy_state = std::move(state);
    publishMarkers(bottom, up, output);
    RCLCPP_INFO(node_->get_logger(),
      "Bowl strategy selected pregrasp=%.0f mm rim_azimuth=%.1f deg finger_flip=%.0f deg",
      1000.0 * selected_pregrasp, selected_azimuth, selected_flip);
    return true;
  }

  bool confirmDescent(PreparedPick& plan, StrategyFailure& failure) override
  {
    const auto state = std::dynamic_pointer_cast<BowlState>(plan.strategy_state);
    if (!state) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "bowl strategy state is missing"};
      return false;
    }
    if (!runtime_.removeTargetCollision(state->target_collision_ids)) {
      failure = {action::PickObject::Result::FAILURE_EXECUTION_ERROR,
        "DESCEND", "target collision removal did not reach the planning scene"};
      return false;
    }
    auto& move_group = runtime_.moveGroup();
    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory descent;
    if (move_group.computeCartesianPath(
        {poseMessage(state->grasp_transform)}, runtime_.cartesianStep(), 0.0,
        descent, true) >= runtime_.minimumCartesianFraction())
    {
      plan.grasp_pose = poseMessage(state->grasp_transform);
      plan.descent_trajectory = std::move(descent);
      return true;
    }
    failure = {action::PickObject::Result::FAILURE_REPOSITION_REQUIRED,
      "DESCEND", "bowl-rim grasp is unreachable from actual pregrasp; reposition Go2"};
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject makeAttachedObject(
    const PreparedPick& plan, geometry_msgs::msg::PointStamped& expected) override
  {
    const auto state = std::dynamic_pointer_cast<BowlState>(plan.strategy_state);
    if (!state) throw std::runtime_error("bowl strategy state is missing");
    // Both transforms below were computed for the same pre-lift grasp state.
    // Their relative transform is therefore invariant while the grasped bowl
    // follows TCP.  Do not combine the stale, pre-lift world bowl pose with
    // the current, post-lift TCP pose: that would offset the attached object
    // by the complete lift displacement.
    const Eigen::Isometry3d tcp_from_model =
      state->grasp_transform.inverse() * state->world_from_model;
    const Eigen::Vector3d tcp_bottom = tcp_from_model.translation();
    const Eigen::Vector3d tcp_up = tcp_from_model.linear().col(2);

    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = runtime_.tcpFrame();
    attached.touch_links = {
      runtime_.tcpFrame(), runtime_.link6Frame(), "left_finger", "right_finger"};
    attached.object.header.frame_id = runtime_.tcpFrame();
    attached.object.id = "held/bowl";
    auto append_primitive = [&attached, &tcp_from_model](
      shape_msgs::msg::SolidPrimitive primitive,
      const Eigen::Isometry3d& model_from_primitive)
      {
        attached.object.primitives.push_back(std::move(primitive));
        attached.object.primitive_poses.push_back(
          poseMessage(tcp_from_model * model_from_primitive));
      };

    // Keep this collision decomposition identical to MuJoCo's physical bowl:
    // one bottom cylinder plus twelve thin, sloped wall boxes.  In particular,
    // do not replace the hollow bowl with a solid outer cylinder.
    constexpr double center_x = -0.00023;
    constexpr double center_y = -0.00005;
    shape_msgs::msg::SolidPrimitive bottom;
    bottom.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    bottom.dimensions = {0.008, 0.0405};  // height, radius
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
      mid_radius * std::tan(M_PI / static_cast<double>(segment_count)) + 0.0008;
    for (int index = 0; index < segment_count; ++index) {
      const double theta = 2.0 * M_PI * index / segment_count;
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
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    const Eigen::Vector3d centre = tcp_bottom + 0.5 * height_ * tcp_up;
    expected.header.frame_id = runtime_.tcpFrame(); expected.header.stamp = node_->now();
    expected.point.x = centre.x(); expected.point.y = centre.y(); expected.point.z = centre.z();
    return attached;
  }

private:
  srv::EstimateBowl::Response::SharedPtr estimate(uint8_t stage,
    const geometry_msgs::msg::PointStamped& hint,
    const Eigen::Vector3d& normal = Eigen::Vector3d::Zero(), double offset = 0.0)
  {
    if (!estimate_client_->wait_for_service(5s)) return {};
    auto request = std::make_shared<srv::EstimateBowl::Request>();
    request->stage = stage; request->target_hint = hint;
    request->ground_normal.x = normal.x(); request->ground_normal.y = normal.y();
    request->ground_normal.z = normal.z(); request->ground_offset = offset;
    auto future = estimate_client_->async_send_request(request);
    if (future.wait_for(15s) != std::future_status::ready) return {};
    return future.get();
  }

  std::vector<double> descending() const
  {
    std::vector<double> values;
    for (double value = pregrasp_max_; value >= pregrasp_min_ - 1e-9;
      value -= pregrasp_step_) values.push_back(value);
    return values;
  }

  void publishMarkers(const Eigen::Vector3d& bottom, const Eigen::Vector3d& up,
    const PreparedPick& plan)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear; clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = runtime_.planningFrame(); sphere.ns = "bowl_bottom"; sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = bottom.x(); sphere.pose.position.y = bottom.y();
    sphere.pose.position.z = bottom.z(); sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.025;
    sphere.color = color(1, 0, 1); array.markers.push_back(sphere);
    int id = 1;
    for (const auto& item : {std::make_pair("grasp", plan.grasp_pose),
      std::make_pair("pregrasp", plan.pregrasp_pose)})
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = runtime_.planningFrame(); arrow.ns = item.first; arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD; arrow.pose = item.second;
      arrow.scale.x = 0.10; arrow.scale.y = 0.015; arrow.scale.z = 0.02;
      arrow.color = item.first == std::string("grasp") ? color(1, 1, 0) : color(0, 1, 1);
      array.markers.push_back(arrow);
    }
    (void)up;
    marker_publisher_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  PickStrategyRuntime& runtime_;
  rclcpp::Client<srv::EstimateBowl>::SharedPtr estimate_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  const std::string class_name_{"bowl"};
  std::string estimate_name_;
  double pregrasp_max_{}, pregrasp_min_{}, pregrasp_step_{};
  double radius_{}, height_{};
  double gripper_open_{}, gripper_closed_{}, grasp_settle_{}, lift_distance_{};
};

std::unique_ptr<PickStrategy> makeBowlPickStrategy(
  const rclcpp::Node::SharedPtr& node, PickStrategyRuntime& runtime)
{
  return std::make_unique<BowlPickStrategy>(node, runtime);
}

}  // namespace d1_manipulation
