#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace d1_manipulation
{
namespace
{
moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const std::string& frame,
  const geometry_msgs::msg::Pose& pose,
  const std::vector<double>& dimensions)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  for (const double dimension : dimensions) {
    shape.dimensions.push_back(dimension);
  }
  object.primitives.push_back(std::move(shape));
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}
}  // namespace

class BasePlanningSceneInitializer
{
public:
  explicit BasePlanningSceneInitializer(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    planning_frame_ = node_->declare_parameter<std::string>("planning_frame", "base_link");
    ground_id_ = node_->declare_parameter<std::string>("ground_collision_id", "ground");
    platform_id_ = node_->declare_parameter<std::string>(
      "go2_platform_collision_id", "go2_platform");
    ground_surface_z_ = node_->declare_parameter<double>(
      "ground_surface_z_m", -0.225248769402);
    ground_centre_x_ = node_->declare_parameter<double>("ground_centre_x_m", 0.3);
    ground_dimensions_ = node_->declare_parameter<std::vector<double>>(
      "ground_dimensions_m", {2.0, 2.0, 0.02});
    platform_position_ = node_->declare_parameter<std::vector<double>>(
      "go2_platform_position_m", {-0.044763, 0.0, -0.096688269402});
    platform_dimensions_ = node_->declare_parameter<std::vector<double>>(
      "go2_platform_dimensions_m", {0.753442, 0.338254, 0.255121});
    if (planning_frame_.empty() || ground_id_.empty() || platform_id_.empty() ||
      ground_dimensions_.size() != 3 || platform_position_.size() != 3 ||
      platform_dimensions_.size() != 3 ||
      std::any_of(
        ground_dimensions_.begin(), ground_dimensions_.end(), [](double value) {return value <= 0.0;}) ||
      std::any_of(
        platform_dimensions_.begin(), platform_dimensions_.end(), [](double value) {return value <= 0.0;}))
    {
      throw std::invalid_argument("invalid base planning-scene geometry parameters");
    }

    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/arm/planning_scene_ready", rclcpp::QoS(1).reliable().transient_local());
    timer_ = node_->create_wall_timer(500ms, [this]() {ensureScene();});
    publishReady(false);
  }

private:
  void publishReady(bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_publisher_->publish(message);
  }

  bool scenePresent()
  {
    const auto known = planning_scene_.getKnownObjectNames();
    const bool ground_present =
      std::find(known.begin(), known.end(), ground_id_) != known.end();
    const bool platform_present =
      planning_scene_.getAttachedObjects({platform_id_}).size() == 1;
    return ground_present && platform_present;
  }

  void ensureScene()
  {
    if (scenePresent()) {
      if (!ready_) {
        ready_ = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "Base planning scene ready: ground=%s platform=%s",
          ground_id_.c_str(), platform_id_.c_str());
      }
      publishReady(true);
      return;
    }

    ready_ = false;
    publishReady(false);
    geometry_msgs::msg::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = ground_centre_x_;
    ground_pose.position.z = ground_surface_z_ - 0.5 * ground_dimensions_[2];
    const auto ground = makeBox(
      ground_id_, planning_frame_, ground_pose, ground_dimensions_);
    if (!planning_scene_.applyCollisionObject(ground)) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to apply base planning-scene ground");
      return;
    }

    geometry_msgs::msg::Pose platform_pose;
    platform_pose.orientation.w = 1.0;
    platform_pose.position.x = platform_position_[0];
    platform_pose.position.y = platform_position_[1];
    platform_pose.position.z = platform_position_[2];
    moveit_msgs::msg::AttachedCollisionObject platform;
    platform.link_name = planning_frame_;
    platform.object = makeBox(
      platform_id_, planning_frame_, platform_pose, platform_dimensions_);
    platform.touch_links = {planning_frame_};
    if (!planning_scene_.applyAttachedCollisionObject(platform)) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to attach Go2 platform planning collision box");
    }
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string planning_frame_;
  std::string ground_id_;
  std::string platform_id_;
  std::vector<double> ground_dimensions_;
  std::vector<double> platform_position_;
  std::vector<double> platform_dimensions_;
  double ground_surface_z_{};
  double ground_centre_x_{};
  bool ready_{false};
};
}  // namespace d1_manipulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("d1_base_planning_scene");
  auto initializer = std::make_shared<d1_manipulation::BasePlanningSceneInitializer>(node);
  rclcpp::spin(node);
  initializer.reset();
  rclcpp::shutdown();
  return 0;
}
