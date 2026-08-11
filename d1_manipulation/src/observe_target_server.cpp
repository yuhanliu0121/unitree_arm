#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "d1_manipulation/action/observe_target.hpp"
#include "d1_manipulation/observation_geometry.hpp"

using namespace std::chrono_literals;

namespace d1_manipulation
{
namespace
{
constexpr char kSceneMagic[] = "D1SCENE";
constexpr int kSceneVersion = 1;
constexpr char kGo2PlatformName[] = "go2_platform";

template<typename T>
T parameterOrDeclare(
  const rclcpp::Node::SharedPtr& node,
  const std::string& name,
  const T& default_value)
{
  if (node->has_parameter(name)) {
    T value;
    if (node->get_parameter(name, value)) {
      return value;
    }
  }
  return node->declare_parameter<T>(name, default_value);
}

struct ScenePose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class UdpSceneSnapshot
{
public:
  explicit UdpSceneSnapshot(int port)
  {
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
      throw std::runtime_error("failed to create scene-state UDP socket");
    }
    int reuse = 1;
    ::setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      ::close(socket_);
      socket_ = -1;
      throw std::runtime_error("failed to bind scene-state UDP port " + std::to_string(port));
    }
  }

  ~UdpSceneSnapshot()
  {
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  std::map<std::string, ScenePose> receive(std::chrono::milliseconds timeout)
  {
    pollfd descriptor{socket_, POLLIN, 0};
    if (::poll(&descriptor, 1, static_cast<int>(timeout.count())) <= 0) {
      return {};
    }
    char buffer[4096];
    const auto size = ::recv(socket_, buffer, sizeof(buffer) - 1, 0);
    if (size <= 0) {
      return {};
    }
    buffer[size] = '\0';
    auto result = parse(buffer);
    while (true) {
      const auto queued = ::recv(socket_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
      if (queued <= 0) {
        break;
      }
      buffer[queued] = '\0';
      auto newer = parse(buffer);
      if (!newer.empty()) {
        result = std::move(newer);
      }
    }
    return result;
  }

private:
  static std::map<std::string, ScenePose> parse(const std::string& payload)
  {
    std::istringstream stream(payload);
    std::string magic;
    int version = 0;
    double simulation_time = 0.0;
    if (!(stream >> magic >> version >> simulation_time) || magic != kSceneMagic ||
      version != kSceneVersion)
    {
      return {};
    }
    std::map<std::string, ScenePose> poses;
    std::string name;
    while (stream >> name) {
      ScenePose pose;
      double qw = 1.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      if (!(stream >> pose.position.x() >> pose.position.y() >> pose.position.z() >>
        qw >> qx >> qy >> qz))
      {
        return {};
      }
      pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
      poses[name] = pose;
    }
    return poses;
  }

  int socket_{-1};
};

geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose message;
  message.position.x = transform.translation().x();
  message.position.y = transform.translation().y();
  message.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();
  return message;
}

geometry_msgs::msg::Pose offsetPose(
  const ScenePose& body,
  const Eigen::Vector3d& local_offset)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = body.orientation.toRotationMatrix();
  transform.translation() = body.position + body.orientation * local_offset;
  return poseMessage(transform);
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const std::string& frame,
  const geometry_msgs::msg::Pose& pose,
  double x,
  double y,
  double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {x, y, z};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

moveit_msgs::msg::CollisionObject makeCylinder(
  const std::string& id,
  const std::string& frame,
  const geometry_msgs::msg::Pose& pose,
  double height,
  double radius)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  shape.dimensions = {height, radius};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA value;
  value.r = r;
  value.g = g;
  value.b = b;
  value.a = a;
  return value;
}
}  // namespace

class ObserveTargetServer
{
public:
  using ObserveTarget = d1_manipulation::action::ObserveTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ObserveTarget>;

  explicit ObserveTargetServer(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node, parameterOrDeclare(node, "arm_group", std::string("arm"))),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    action_name_ = parameterOrDeclare(node_, "action_name", std::string("/arm/debug/observe_target"));
    planning_frame_ = parameterOrDeclare(node_, "planning_frame", std::string("base_link"));
    gravity_frame_ = parameterOrDeclare(node_, "gravity_frame", std::string("world"));
    link6_frame_ = parameterOrDeclare(node_, "link6_frame", std::string("Link6"));
    camera_frame_ = parameterOrDeclare(
      node_, "color_optical_frame", std::string("wrist_camera_color_optical_frame"));
    camera_info_topic_ = parameterOrDeclare(
      node_, "color_camera_info_topic", std::string("/wrist_camera/color/camera_info"));
    beta_degrees_ = parameterOrDeclare(
      node_, "beta_degrees", std::vector<double>{0.0, -5.0, 5.0, -10.0, 10.0, -15.0, 15.0});
    alpha_degrees_ = parameterOrDeclare(
      node_, "alpha_degrees", std::vector<double>{45.0, 40.0, 50.0, 35.0, 55.0, 30.0, 60.0, 65.0, 70.0});
    distances_m_ = parameterOrDeclare(
      node_, "distances_m", std::vector<double>{0.35, 0.30, 0.40, 0.45, 0.50, 0.55, 0.60});
    tf_timeout_s_ = parameterOrDeclare(node_, "tf_timeout_s", 1.0);
    camera_settle_s_ = parameterOrDeclare(node_, "camera_settle_s", 0.5);
    max_target_pixel_error_ = parameterOrDeclare(node_, "max_target_pixel_error", 30.0);
    stowed_tolerance_rad_ = parameterOrDeclare(node_, "stowed_tolerance_rad", 0.08);
    stowed_ = parameterOrDeclare(
      node_, "stowed_joint_positions", std::vector<double>{0.0, -1.5, 1.5, 0.0, 0.0, 0.0});
    use_simulation_scene_truth_ = parameterOrDeclare(node_, "use_simulation_scene_truth", true);
    scene_state_port_ = parameterOrDeclare(node_, "scene_state_port", 15002);
    ground_surface_z_ = parameterOrDeclare(node_, "ground_surface_z_m", -0.225248769402);

    move_group_.setEndEffectorLink(link6_frame_);
    move_group_.setPoseReferenceFrame(planning_frame_);
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPlanningTime(parameterOrDeclare(node_, "planning_time_s", 1.5));
    move_group_.setNumPlanningAttempts(parameterOrDeclare(node_, "planning_attempts", 2));
    move_group_.setMaxVelocityScalingFactor(parameterOrDeclare(node_, "velocity_scaling", 0.10));
    move_group_.setMaxAccelerationScalingFactor(
      parameterOrDeclare(node_, "acceleration_scaling", 0.10));
    move_group_.setGoalPositionTolerance(parameterOrDeclare(node_, "position_tolerance_m", 0.005));
    move_group_.setGoalOrientationTolerance(
      parameterOrDeclare(node_, "orientation_tolerance_rad", 0.03));

    auto marker_qos = rclcpp::QoS(1).transient_local().reliable();
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/arm/debug/observation_markers", marker_qos);
    camera_info_subscription_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        latest_camera_info_ = std::move(message);
      });

    action_server_ = rclcpp_action::create_server<ObserveTarget>(
      node_, action_name_,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ObserveTarget::Goal> goal) {
        if (busy_.exchange(true)) {
          RCLCPP_WARN(node_->get_logger(), "Rejecting ObserveTarget goal: server is busy");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->target.header.frame_id.empty()) {
          busy_.store(false);
          RCLCPP_WARN(node_->get_logger(), "Rejecting ObserveTarget goal with empty frame_id");
          return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_requested_.store(false);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        cancel_requested_.store(true);
        move_group_.stop();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
      });

    RCLCPP_INFO(
      node_->get_logger(), "ObserveTarget action server ready: %s (%zu ordered candidates)",
      action_name_.c_str(), beta_degrees_.size() * alpha_degrees_.size() * distances_m_.size());
  }

private:
  geometry_msgs::msg::TransformStamped lookupLatestTransform(
    const std::string& target_frame,
    const std::string& source_frame)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(tf_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        return tf_buffer_.lookupTransform(
          target_frame, source_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException&) {
        std::this_thread::sleep_for(20ms);
      }
    }
    return tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
  }

  void publishFeedback(
    const std::shared_ptr<GoalHandle>& goal_handle,
    const std::string& state,
    std::size_t index,
    std::size_t count,
    const ObservationCandidate* candidate,
    const std::string& detail)
  {
    auto feedback = std::make_shared<ObserveTarget::Feedback>();
    feedback->current_state = state;
    feedback->candidate_index = static_cast<uint32_t>(index);
    feedback->candidate_count = static_cast<uint32_t>(count);
    if (candidate != nullptr) {
      feedback->beta_deg = candidate->beta_deg;
      feedback->alpha_deg = candidate->alpha_deg;
      feedback->distance_m = candidate->distance_m;
    }
    feedback->detail = detail;
    goal_handle->publish_feedback(feedback);
  }

  void finishFailure(
    const std::shared_ptr<GoalHandle>& goal_handle,
    uint8_t category,
    const std::string& state,
    const std::string& detail,
    bool canceled = false)
  {
    auto result = std::make_shared<ObserveTarget::Result>();
    result->success = false;
    result->failure_category = category;
    result->failed_state = state;
    result->detail = detail;
    result->returned_to_stowed = isAtStowed();
    if (canceled) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    busy_.store(false);
  }

  Eigen::Vector3d targetInPlanningFrame(const geometry_msgs::msg::PointStamped& target)
  {
    Eigen::Vector3d point(target.point.x, target.point.y, target.point.z);
    if (!point.allFinite()) {
      throw std::runtime_error("target coordinates are not finite");
    }
    if (target.header.frame_id == planning_frame_) {
      return point;
    }
    geometry_msgs::msg::TransformStamped transform;
    const auto timeout = tf2::durationFromSec(tf_timeout_s_);
    if (target.header.stamp.sec == 0 && target.header.stamp.nanosec == 0) {
      transform = lookupLatestTransform(planning_frame_, target.header.frame_id);
    } else {
      transform = tf_buffer_.lookupTransform(
        planning_frame_, target.header.frame_id, rclcpp::Time(target.header.stamp), timeout);
    }
    return tf2::transformToEigen(transform) * point;
  }

  Eigen::Vector3d upInPlanningFrame()
  {
    const auto transform = lookupLatestTransform(planning_frame_, gravity_frame_);
    return (tf2::transformToEigen(transform).rotation() * Eigen::Vector3d::UnitZ()).normalized();
  }

  Eigen::Isometry3d link6FromCamera()
  {
    const auto transform = lookupLatestTransform(link6_frame_, camera_frame_);
    return tf2::transformToEigen(transform);
  }

  sensor_msgs::msg::CameraInfo::ConstSharedPtr waitForCameraInfo()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(tf_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        if (latest_camera_info_ != nullptr) {
          return latest_camera_info_;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("wrist color CameraInfo is unavailable");
  }

  double targetPixelError(
    const Eigen::Vector3d& target,
    const sensor_msgs::msg::CameraInfo& info)
  {
    const auto transform = lookupLatestTransform(planning_frame_, camera_frame_);
    const Eigen::Vector3d point = tf2::transformToEigen(transform).inverse() * target;
    if (point.z() <= 0.0) {
      throw std::runtime_error("target is behind the executed RGB optical frame");
    }
    double x = point.x() / point.z();
    double y = point.y() / point.z();
    if (info.distortion_model == "plumb_bob" && info.d.size() >= 5) {
      const double radius2 = x * x + y * y;
      const double radial = 1.0 + info.d[0] * radius2 +
        info.d[1] * radius2 * radius2 + info.d[4] * radius2 * radius2 * radius2;
      const double distorted_x = x * radial + 2.0 * info.d[2] * x * y +
        info.d[3] * (radius2 + 2.0 * x * x);
      const double distorted_y = y * radial + info.d[2] * (radius2 + 2.0 * y * y) +
        2.0 * info.d[3] * x * y;
      x = distorted_x;
      y = distorted_y;
    }
    const double pixel_x = info.k[0] * x + info.k[2];
    const double pixel_y = info.k[4] * y + info.k[5];
    return std::hypot(pixel_x - info.k[2], pixel_y - info.k[5]);
  }

  void applyPlanningScene(const Eigen::Vector3d& target)
  {
    geometry_msgs::msg::Pose identity;
    identity.orientation.w = 1.0;
    geometry_msgs::msg::Pose ground = identity;
    ground.position.x = 0.3;
    ground.position.z = ground_surface_z_ - 0.01;
    std::vector<moveit_msgs::msg::CollisionObject> objects{
      makeBox("ground", planning_frame_, ground, 2.0, 2.0, 0.02)};

    bool received_truth = false;
    if (use_simulation_scene_truth_) {
      UdpSceneSnapshot receiver(scene_state_port_);
      const auto poses = receiver.receive(1s);
      const auto cube = poses.find("yellow_cube");
      if (cube != poses.end()) {
        objects.push_back(makeBox(
          "yellow_cube", planning_frame_,
          offsetPose(cube->second, {-0.000787, -0.000889, 0.025}),
          0.05, 0.05, 0.05));
        received_truth = true;
      }
      const auto bowl = poses.find("bowl");
      if (bowl != poses.end()) {
        objects.push_back(makeCylinder(
          "bowl", planning_frame_, offsetPose(bowl->second, {-0.00023, -0.00005, 0.025}),
          0.05, 0.06));
      }
      const auto zucchini = poses.find("zucchini");
      if (zucchini != poses.end()) {
        objects.push_back(makeBox(
          "zucchini", planning_frame_,
          offsetPose(zucchini->second, {0.00047, -0.00478, 0.01656}),
          0.04, 0.15, 0.034));
      }
    }
    if (!received_truth) {
      geometry_msgs::msg::Pose target_pose = identity;
      target_pose.position.x = target.x();
      target_pose.position.y = target.y();
      target_pose.position.z = target.z();
      objects.push_back(makeBox("observe_target", planning_frame_, target_pose, 0.05, 0.05, 0.05));
    }

    std::vector<moveit_msgs::msg::ObjectColor> colors;
    for (const auto& object : objects) {
      moveit_msgs::msg::ObjectColor object_color;
      object_color.id = object.id;
      object_color.color = color(1.0F, 0.45F, 0.05F, object.id == "ground" ? 1.0F : 0.55F);
      colors.push_back(object_color);
    }
    if (!planning_scene_.applyCollisionObjects(objects, colors)) {
      throw std::runtime_error("failed to apply observation planning scene");
    }

    geometry_msgs::msg::Pose go2_pose = identity;
    go2_pose.position.x = -0.044763;
    go2_pose.position.z = -0.096688269402;
    moveit_msgs::msg::AttachedCollisionObject go2;
    go2.link_name = planning_frame_;
    go2.object = makeBox(
      kGo2PlatformName, planning_frame_, go2_pose, 0.753442, 0.338254, 0.255121);
    go2.touch_links = {planning_frame_};
    if (!planning_scene_.applyAttachedCollisionObject(go2)) {
      throw std::runtime_error("failed to attach Go2 planning collision box");
    }
    std::this_thread::sleep_for(300ms);
  }

  bool isAtStowed()
  {
    const auto current = move_group_.getCurrentJointValues();
    if (current.size() != stowed_.size()) {
      return false;
    }
    for (std::size_t index = 0; index < current.size(); ++index) {
      if (std::abs(current[index] - stowed_[index]) > stowed_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  bool returnToStowed()
  {
    if (isAtStowed()) {
      return true;
    }
    move_group_.setStartStateToCurrentState();
    if (!move_group_.setJointValueTarget(stowed_)) {
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS && isAtStowed();
  }

  visualization_msgs::msg::Marker arrowMarker(
    int id,
    const std::string& name,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std_msgs::msg::ColorRGBA& marker_color)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = node_->now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point point;
    point.x = start.x(); point.y = start.y(); point.z = start.z();
    marker.points.push_back(point);
    point.x = end.x(); point.y = end.y(); point.z = end.z();
    marker.points.push_back(point);
    marker.scale.x = 0.008;
    marker.scale.y = 0.016;
    marker.scale.z = 0.020;
    marker.color = marker_color;
    return marker;
  }

  void publishMarkers(
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& up,
    const ObservationCandidate& candidate)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = planning_frame_;
    target_marker.header.stamp = node_->now();
    target_marker.ns = "target";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose.position.x = target.x();
    target_marker.pose.position.y = target.y();
    target_marker.pose.position.z = target.z();
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 0.035;
    target_marker.color = color(1.0F, 1.0F, 0.0F, 0.9F);
    array.markers.push_back(target_marker);
    array.markers.push_back(arrowMarker(1, "up", target, target + 0.15 * up, color(0, 0.8F, 0, 1)));

    const Eigen::Vector3d camera = candidate.camera_position;
    const double axis_length = 0.08;
    array.markers.push_back(arrowMarker(
      2, "camera_x", camera,
      camera + axis_length * candidate.planning_from_camera.col(0), color(1, 0, 0, 1)));
    array.markers.push_back(arrowMarker(
      3, "camera_y", camera,
      camera + axis_length * candidate.planning_from_camera.col(1), color(0, 1, 0, 1)));
    array.markers.push_back(arrowMarker(
      4, "camera_z", camera,
      camera + axis_length * candidate.planning_from_camera.col(2), color(0, 0.4F, 1, 1)));
    array.markers.push_back(arrowMarker(5, "optical_ray", camera, target, color(0, 1, 1, 0.9F)));
    marker_publisher_->publish(array);
  }

  void execute(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    try {
      publishFeedback(goal_handle, "CHECK_PRECONDITIONS", 0, 0, nullptr, "Resolving target, gravity and camera TF");
      const Eigen::Vector3d target = targetInPlanningFrame(goal->target);
      const Eigen::Vector3d up = upInPlanningFrame();
      const Eigen::Isometry3d link6_from_camera = link6FromCamera();
      const auto camera_info = waitForCameraInfo();
      applyPlanningScene(target);
      const auto candidates = generateObservationCandidates(
        target, up, beta_degrees_, alpha_degrees_, distances_m_);

      RCLCPP_INFO(
        node_->get_logger(),
        "ObserveTarget: target=(%.3f, %.3f, %.3f), up=(%.3f, %.3f, %.3f), candidates=%zu",
        target.x(), target.y(), target.z(), up.x(), up.y(), up.z(), candidates.size());

      for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (cancel_requested_.load() || goal_handle->is_canceling()) {
          publishFeedback(goal_handle, "RETURNING_STOWED", index, candidates.size(), nullptr, "Canceled; returning to STOWED");
          returnToStowed();
          finishFailure(
            goal_handle, ObserveTarget::Result::FAILURE_EXECUTION_ERROR,
            "PLAN_OBSERVE", "ObserveTarget was canceled", true);
          return;
        }
        const auto& candidate = candidates[index];
        publishFeedback(
          goal_handle, "PLAN_OBSERVE", index + 1, candidates.size(), &candidate,
          "Trying ordered observation candidate");

        Eigen::Isometry3d planning_from_camera = Eigen::Isometry3d::Identity();
        planning_from_camera.linear() = candidate.planning_from_camera;
        planning_from_camera.translation() = candidate.camera_position;
        const Eigen::Isometry3d planning_from_link6 =
          planning_from_camera * link6_from_camera.inverse();
        const auto link6_pose = poseMessage(planning_from_link6);

        move_group_.setStartStateToCurrentState();
        if (!move_group_.setJointValueTarget(link6_pose, link6_frame_)) {
          continue;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          continue;
        }

        publishMarkers(target, up, candidate);
        publishFeedback(
          goal_handle, "MOVE_OBSERVE", index + 1, candidates.size(), &candidate,
          "Executing first fully planned candidate");
        if (move_group_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          publishFeedback(
            goal_handle, "RETURNING_STOWED", index + 1, candidates.size(), &candidate,
            "Execution failed; returning to STOWED");
          returnToStowed();
          finishFailure(
            goal_handle, ObserveTarget::Result::FAILURE_EXECUTION_ERROR,
            "MOVE_OBSERVE", "Observation trajectory execution failed");
          return;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(camera_settle_s_));
        const double pixel_error = targetPixelError(target, *camera_info);
        if (pixel_error > max_target_pixel_error_) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Executed observation target is %.1f px from image centre (limit %.1f px)",
            pixel_error, max_target_pixel_error_);
          publishFeedback(
            goal_handle, "RETURNING_STOWED", index + 1, candidates.size(), &candidate,
            "Executed camera alignment failed; returning to STOWED");
          returnToStowed();
          finishFailure(
            goal_handle, ObserveTarget::Result::FAILURE_EXECUTION_ERROR,
            "OBSERVING", "Executed target projection is outside the centre tolerance");
          return;
        }
        auto result = std::make_shared<ObserveTarget::Result>();
        result->success = true;
        result->failure_category = ObserveTarget::Result::FAILURE_NONE;
        result->detail = "First feasible ordered observation candidate executed";
        result->returned_to_stowed = false;
        result->selected_beta_deg = candidate.beta_deg;
        result->selected_alpha_deg = candidate.alpha_deg;
        result->selected_distance_m = candidate.distance_m;
        result->selected_camera_pose.header.frame_id = planning_frame_;
        result->selected_camera_pose.header.stamp = node_->now();
        result->selected_camera_pose.pose = poseMessage(planning_from_camera);
        result->selected_link6_pose.header = result->selected_camera_pose.header;
        result->selected_link6_pose.pose = link6_pose;
        publishFeedback(
          goal_handle, "OBSERVING", index + 1, candidates.size(), &candidate,
          "Observation pose reached and camera settled");
        goal_handle->succeed(result);
        busy_.store(false);
        RCLCPP_INFO(
          node_->get_logger(),
          "OBSERVE SUCCEEDED: candidate=%zu beta=%.1f alpha=%.1f distance=%.2f pixel_error=%.1f",
          index + 1, candidate.beta_deg, candidate.alpha_deg, candidate.distance_m, pixel_error);
        return;
      }

      finishFailure(
        goal_handle, ObserveTarget::Result::FAILURE_THEORETICALLY_INFEASIBLE,
        "PLAN_OBSERVE", "All ordered observation candidates failed IK or motion planning");
    } catch (const tf2::TransformException& exception) {
      finishFailure(
        goal_handle, ObserveTarget::Result::FAILURE_INCOMPLETE_INFORMATION,
        "CHECK_PRECONDITIONS", std::string("TF unavailable: ") + exception.what());
    } catch (const std::invalid_argument& exception) {
      finishFailure(
        goal_handle, ObserveTarget::Result::FAILURE_INCOMPLETE_INFORMATION,
        "CHECK_PRECONDITIONS", exception.what());
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(node_->get_logger(), "ObserveTarget internal error: %s", exception.what());
      finishFailure(
        goal_handle, ObserveTarget::Result::FAILURE_EXECUTION_ERROR,
        "INTERNAL", exception.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<ObserveTarget>::SharedPtr action_server_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_requested_{false};
  std::string action_name_;
  std::string planning_frame_;
  std::string gravity_frame_;
  std::string link6_frame_;
  std::string camera_frame_;
  std::string camera_info_topic_;
  std::vector<double> beta_degrees_;
  std::vector<double> alpha_degrees_;
  std::vector<double> distances_m_;
  std::vector<double> stowed_;
  double tf_timeout_s_{};
  double camera_settle_s_{};
  double max_target_pixel_error_{};
  double stowed_tolerance_rad_{};
  bool use_simulation_scene_truth_{};
  int scene_state_port_{};
  double ground_surface_z_{};
};
}  // namespace d1_manipulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "d1_observe_target",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<d1_manipulation::ObserveTargetServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  server.reset();
  rclcpp::shutdown();
  return 0;
}
