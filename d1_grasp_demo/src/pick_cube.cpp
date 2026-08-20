#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr char kSceneMagic[] = "D1SCENE";
constexpr int kSceneVersion = 1;
constexpr char kPlanningFrame[] = "base_link";
constexpr char kArmGroup[] = "arm";
constexpr char kTcpLink[] = "tcp_link";
constexpr char kCubeName[] = "yellow_cube";
constexpr char kGo2PlatformName[] = "go2_platform";
constexpr double kPi = 3.14159265358979323846;

struct ScenePose
{
  geometry_msgs::msg::Pose pose;
  double simulation_time{};
};

geometry_msgs::msg::Pose offsetPose(
  const geometry_msgs::msg::Pose& body_pose,
  double local_x,
  double local_y,
  double local_z)
{
  const auto& q = body_pose.orientation;
  // Rotate the local offset by the body quaternion, then translate it into the
  // planning frame. Object truth intentionally reports the MuJoCo body origin.
  const double tx = 2.0 * (q.y * local_z - q.z * local_y);
  const double ty = 2.0 * (q.z * local_x - q.x * local_z);
  const double tz = 2.0 * (q.x * local_y - q.y * local_x);
  geometry_msgs::msg::Pose result = body_pose;
  result.position.x += local_x + q.w * tx + q.y * tz - q.z * ty;
  result.position.y += local_y + q.w * ty + q.z * tx - q.x * tz;
  result.position.z += local_z + q.w * tz + q.x * ty - q.y * tx;
  return result;
}

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

class UdpSceneReceiver
{
public:
  explicit UdpSceneReceiver(int port)
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

  ~UdpSceneReceiver()
  {
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  bool receiveLatest(
    const std::string& object_name,
    std::chrono::milliseconds timeout,
    ScenePose& result)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool found = false;
    do {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      pollfd descriptor{socket_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(std::max<int64_t>(0, remaining.count())));
      if (ready <= 0) {
        break;
      }
      char buffer[4096];
      const auto size = ::recv(socket_, buffer, sizeof(buffer) - 1, 0);
      if (size <= 0) {
        continue;
      }
      buffer[size] = '\0';
      const auto packet = parsePacket(buffer);
      const auto match = packet.find(object_name);
      if (match != packet.end()) {
        result = match->second;
        found = true;
      }

      // Drain all queued packets without delaying so the returned pose is the
      // newest one, not a sample accumulated during trajectory execution.
      while (true) {
        const auto queued_size = ::recv(socket_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
        if (queued_size <= 0) {
          break;
        }
        buffer[queued_size] = '\0';
        const auto queued = parsePacket(buffer);
        const auto queued_match = queued.find(object_name);
        if (queued_match != queued.end()) {
          result = queued_match->second;
          found = true;
        }
      }
      return found;
    } while (std::chrono::steady_clock::now() < deadline);
    return found;
  }

private:
  static std::map<std::string, ScenePose> parsePacket(const std::string& payload)
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
      ScenePose value;
      value.simulation_time = simulation_time;
      double qw = 1.0;
      if (!(stream >> value.pose.position.x >> value.pose.position.y >> value.pose.position.z >> qw >>
        value.pose.orientation.x >> value.pose.orientation.y >> value.pose.orientation.z))
      {
        return {};
      }
      value.pose.orientation.w = qw;
      poses[name] = value;
    }
    return poses;
  }

  int socket_{-1};
};

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const geometry_msgs::msg::Pose& pose,
  double x,
  double y,
  double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = kPlanningFrame;
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
  const geometry_msgs::msg::Pose& pose,
  double height,
  double radius)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = kPlanningFrame;
  object.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  shape.dimensions.resize(2);
  shape.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
  shape.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std_msgs::msg::ColorRGBA moveItCollisionColor(float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0F;
  color.g = 0.45F;
  color.b = 0.05F;
  color.a = alpha;
  return color;
}

class PickCubeDemo
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit PickCubeDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node, kArmGroup),
    scene_receiver_(parameterOrDeclare<int>(node, "scene_state_port", 15002)),
    gripper_client_(rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd"))
  {
    pregrasp_clearance_ = parameterOrDeclare(node_, "pregrasp_clearance_m", 0.12);
    lift_distance_ = parameterOrDeclare(node_, "lift_distance_m", 0.10);
    cartesian_step_ = parameterOrDeclare(node_, "cartesian_step_m", 0.005);
    minimum_fraction_ = parameterOrDeclare(node_, "minimum_cartesian_fraction", 0.999);
    gripper_open_ = parameterOrDeclare(node_, "gripper_open_m", 0.03);
    gripper_closed_ = parameterOrDeclare(node_, "gripper_closed_m", 0.0);
    grasp_settle_ = parameterOrDeclare(node_, "grasp_settle_s", 0.5);
    hold_time_ = parameterOrDeclare(node_, "hold_s", 2.0);
    velocity_scaling_ = parameterOrDeclare(node_, "velocity_scaling", 0.10);
    acceleration_scaling_ = parameterOrDeclare(node_, "acceleration_scaling", 0.10);
    ground_surface_z_ = parameterOrDeclare(
      node_, "ground_surface_z_m", -0.225248769402);

    move_group_.setEndEffectorLink(kTcpLink);
    move_group_.setPoseReferenceFrame(kPlanningFrame);
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPlanningTime(8.0);
    move_group_.setNumPlanningAttempts(8);
    move_group_.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_.setGoalPositionTolerance(0.005);
    move_group_.setGoalOrientationTolerance(0.03);
  }

  bool run()
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for MuJoCo scene truth on loopback UDP");
    ScenePose cube;
    if (!scene_receiver_.receiveLatest(kCubeName, 5s, cube)) {
      RCLCPP_ERROR(node_->get_logger(), "No yellow_cube scene state received; is d1-mujoco-sim running?");
      return false;
    }
    const double initial_cube_z = cube.pose.position.z;
    const auto cube_center = offsetPose(
      cube.pose, -0.000787, -0.000889, 0.025);
    RCLCPP_INFO(
      node_->get_logger(), "Cube centre: x=%.3f y=%.3f z=%.3f",
      cube_center.position.x, cube_center.position.y, cube_center.position.z);

    addPlanningScene(cube_center);
    if (!commandGripper(gripper_open_)) {
      return false;
    }

    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.position = cube_center.position;
    // tcp_link +Z points out through the fingers. Rotate it toward base -Z;
    // then align the horizontal finger frame with a measured cube edge. This
    // matters because a 50 mm cube is wider than the jaw when approached near
    // its 70.7 mm face diagonal.
    const auto& q = cube.pose.orientation;
    const double cube_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    moveit::planning_interface::MoveGroupInterface::Plan approach;
    bool approach_found = false;
    // A square has four equivalent edge-aligned yaws. D1's asymmetric joint
    // limits make some wrist-yaw branches unreachable even though the grasp is
    // geometrically equivalent, so explicitly choose a branch with valid IK.
    for (const double yaw_offset : {0.0, kPi / 2.0, -kPi / 2.0, kPi}) {
      const double grasp_yaw = cube_yaw + yaw_offset;
      grasp_pose.orientation.x = std::cos(grasp_yaw / 2.0);
      grasp_pose.orientation.y = std::sin(grasp_yaw / 2.0);
      grasp_pose.orientation.z = 0.0;
      grasp_pose.orientation.w = 0.0;
      geometry_msgs::msg::Pose pregrasp_pose = grasp_pose;
      pregrasp_pose.position.z += pregrasp_clearance_;

      move_group_.setStartStateToCurrentState();
      if (!move_group_.setJointValueTarget(pregrasp_pose, kTcpLink)) {
        continue;
      }
      RCLCPP_INFO(
        node_->get_logger(), "Planning pregrasp with cube-symmetric yaw %.1f deg",
        grasp_yaw * 180.0 / kPi);
      if (move_group_.plan(approach) == moveit::core::MoveItErrorCode::SUCCESS) {
        approach_found = true;
        break;
      }
    }
    if (!approach_found) {
      RCLCPP_ERROR(node_->get_logger(), "No collision-free plan to the pregrasp pose");
      return false;
    }
    if (move_group_.execute(approach) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Pregrasp trajectory execution failed");
      return false;
    }

    // The target must no longer be a world obstacle during intentional finger
    // contact. It is added back as an attached object after physical closure.
    planning_scene_.removeCollisionObjects({kCubeName});
    std::this_thread::sleep_for(300ms);

    RCLCPP_INFO(node_->get_logger(), "Descending vertically to the grasp pose");
    if (!executeCartesian(grasp_pose)) {
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Closing gripper and waiting for contact to settle");
    if (!commandGripper(gripper_closed_)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(grasp_settle_));

    ScenePose grasped_cube = cube;
    scene_receiver_.receiveLatest(kCubeName, 1s, grasped_cube);
    const auto grasped_cube_center = offsetPose(
      grasped_cube.pose, -0.000787, -0.000889, 0.025);

    ScenePose simulated_tcp;
    ScenePose left_finger;
    ScenePose right_finger;
    ScenePose contacts;
    ScenePose gripper_state;
    const bool have_tcp = scene_receiver_.receiveLatest("debug_tcp_link", 1s, simulated_tcp);
    const bool have_left = scene_receiver_.receiveLatest("debug_left_finger", 1s, left_finger);
    const bool have_right = scene_receiver_.receiveLatest("debug_right_finger", 1s, right_finger);
    const bool have_contacts = scene_receiver_.receiveLatest("debug_cube_contacts", 1s, contacts);
    const bool have_gripper_state = scene_receiver_.receiveLatest(
      "debug_gripper_state", 1s, gripper_state);
    if (have_tcp) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Physical alignment after close: cube=(%.3f, %.3f, %.3f), MuJoCo TCP=(%.3f, %.3f, %.3f)",
        grasped_cube_center.position.x, grasped_cube_center.position.y,
        grasped_cube_center.position.z, simulated_tcp.pose.position.x,
        simulated_tcp.pose.position.y, simulated_tcp.pose.position.z);
    }
    if (have_left && have_right) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Finger collision centres: left=(%.3f, %.3f, %.3f), right=(%.3f, %.3f, %.3f)",
        left_finger.pose.position.x, left_finger.pose.position.y,
        left_finger.pose.position.z, right_finger.pose.position.x,
        right_finger.pose.position.y, right_finger.pose.position.z);
    }
    if (have_contacts) {
      const int left_count = static_cast<int>(std::lround(contacts.pose.position.x));
      const int right_count = static_cast<int>(std::lround(contacts.pose.position.y));
      RCLCPP_INFO(
        node_->get_logger(), "Cube contact count after close: left=%d right=%d",
        left_count, right_count);
      if (left_count == 0 || right_count == 0) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Physical grasp rejected before lift: the cube is not held by both fingers");
        commandGripper(gripper_open_);
        return false;
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "MuJoCo contact diagnostics unavailable; continuing with final lift verification");
    }
    if (have_gripper_state) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Physical gripper after close: Joint6=%.5f m, normal force left=%.3f N right=%.3f N",
        gripper_state.pose.position.x, gripper_state.pose.position.y,
        gripper_state.pose.position.z);
    }

    planning_scene_.applyCollisionObject(
      makeBox(kCubeName, grasped_cube_center, 0.05, 0.05, 0.05),
      moveItCollisionColor(0.55F));
    if (!move_group_.attachObject(kCubeName, kTcpLink, {"left_finger", "right_finger", "Link6"})) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to attach cube in the MoveIt planning scene");
      return false;
    }
    std::this_thread::sleep_for(300ms);

    auto lift_pose = move_group_.getCurrentPose(kTcpLink).pose;
    lift_pose.position.z += lift_distance_;
    RCLCPP_INFO(node_->get_logger(), "Lifting TCP vertically by %.3f m", lift_distance_);
    if (!executeCartesian(lift_pose)) {
      move_group_.detachObject(kCubeName);
      commandGripper(gripper_open_);
      return false;
    }

    ScenePose final_cube;
    RCLCPP_INFO(node_->get_logger(), "Holding for %.1f s before physical success check", hold_time_);
    const auto hold_started = std::chrono::steady_clock::now();
    while (true) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - hold_started).count();
      if (elapsed >= hold_time_) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(
        std::min(1.0, hold_time_ - elapsed)));
      ScenePose contacts_during_hold;
      ScenePose gripper_during_hold;
      if (!scene_receiver_.receiveLatest(kCubeName, 500ms, final_cube)) {
        RCLCPP_ERROR(node_->get_logger(), "Lost MuJoCo scene state during lift verification");
        return false;
      }
      const bool have_hold_contacts = scene_receiver_.receiveLatest(
        "debug_cube_contacts", 200ms, contacts_during_hold);
      const bool have_hold_gripper = scene_receiver_.receiveLatest(
        "debug_gripper_state", 200ms, gripper_during_hold);
      RCLCPP_INFO(
        node_->get_logger(),
        "Hold %.1f s: rise=%.3f m contacts=%.0f/%.0f Joint6=%.5f m normal=%.3f/%.3f N",
        std::min(hold_time_, elapsed + 1.0),
        final_cube.pose.position.z - initial_cube_z,
        have_hold_contacts ? contacts_during_hold.pose.position.x : -1.0,
        have_hold_contacts ? contacts_during_hold.pose.position.y : -1.0,
        have_hold_gripper ? gripper_during_hold.pose.position.x : -1.0,
        have_hold_gripper ? gripper_during_hold.pose.position.y : -1.0,
        have_hold_gripper ? gripper_during_hold.pose.position.z : -1.0);
    }
    const double rise = final_cube.pose.position.z - initial_cube_z;
    if (rise < 0.05) {
      RCLCPP_ERROR(
        node_->get_logger(), "GRASP FAILED: cube rose only %.3f m and did not remain lifted", rise);
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "GRASP SUCCEEDED: cube remained %.3f m above its initial height", rise);
    return true;
  }

private:
  void addPlanningScene(const geometry_msgs::msg::Pose& cube_center)
  {
    geometry_msgs::msg::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.3;
    ground_pose.position.z = ground_surface_z_ - 0.01;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(makeBox("ground", ground_pose, 2.0, 2.0, 0.02));
    objects.push_back(makeBox(kCubeName, cube_center, 0.05, 0.05, 0.05));

    // Use the same local simulation-truth stream for non-target obstacles.
    geometry_msgs::msg::Pose bowl_pose;
    bowl_pose.orientation.w = 1.0;
    bowl_pose.position.x = 0.068;
    bowl_pose.position.y = 0.213;
    bowl_pose.position.z = 0.025;
    ScenePose bowl;
    if (scene_receiver_.receiveLatest("bowl", 500ms, bowl)) {
      bowl_pose = offsetPose(bowl.pose, -0.00023, -0.00005, 0.025);
    }
    objects.push_back(makeCylinder("bowl", bowl_pose, 0.05, 0.06));

    geometry_msgs::msg::Pose zucchini_pose;
    zucchini_pose.orientation.w = 1.0;
    zucchini_pose.position.x = 0.361;
    zucchini_pose.position.y = -0.148;
    zucchini_pose.position.z = 0.017;
    ScenePose zucchini;
    if (scene_receiver_.receiveLatest("zucchini", 500ms, zucchini)) {
      zucchini_pose = offsetPose(
        zucchini.pose, 0.00047, -0.00478, 0.01656);
    }
    objects.push_back(makeBox("zucchini", zucchini_pose, 0.04, 0.15, 0.034));

    std::vector<moveit_msgs::msg::ObjectColor> colors;
    colors.reserve(objects.size());
    for (const auto& object : objects) {
      moveit_msgs::msg::ObjectColor color;
      color.id = object.id;
      color.color = moveItCollisionColor(object.id == "ground" ? 1.0F : 0.55F);
      colors.push_back(color);
    }
    if (!planning_scene_.applyCollisionObjects(objects, colors)) {
      throw std::runtime_error("failed to apply MoveIt planning scene objects");
    }

    // Mirror MuJoCo's single whole-mesh AABB in the planning model. The box
    // is rigidly attached to base_link so it follows later GT XY navigation.
    // Its unavoidable bolted-mount overlap is allowed only for base_link;
    // every moving arm link must still avoid the Go2 platform.
    geometry_msgs::msg::Pose go2_pose;
    go2_pose.orientation.w = 1.0;
    go2_pose.position.x = -0.044763;
    go2_pose.position.z = -0.096688269402;
    moveit_msgs::msg::AttachedCollisionObject go2_platform;
    go2_platform.link_name = kPlanningFrame;
    go2_platform.object = makeBox(
      kGo2PlatformName, go2_pose, 0.753442, 0.338254, 0.255121);
    go2_platform.touch_links = {kPlanningFrame};
    if (!planning_scene_.applyAttachedCollisionObject(go2_platform)) {
      throw std::runtime_error("failed to attach the Go2 planning collision box");
    }
    std::this_thread::sleep_for(500ms);
  }

  bool commandGripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(node_->get_logger(), "gripper_controller action is unavailable");
      return false;
    }
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = 0.0;
    const auto handle_future = gripper_client_->async_send_goal(goal);
    if (handle_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timed out sending gripper command");
      return false;
    }
    const auto handle = handle_future.get();
    if (!handle) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper command was rejected");
      return false;
    }
    const auto result_future = gripper_client_->async_get_result(handle);
    if (result_future.wait_for(8s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper motion");
      return false;
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper action did not succeed");
      return false;
    }
    return true;
  }

  bool executeCartesian(const geometry_msgs::msg::Pose& target)
  {
    move_group_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    const double fraction = move_group_.computeCartesianPath(
      {target}, cartesian_step_, 0.0, trajectory_message, true);
    if (fraction < minimum_fraction_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Cartesian path fraction %.3f is below %.3f; refusing execution",
        fraction, minimum_fraction_);
      return false;
    }

    auto current_state = move_group_.getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), "Current robot state is unavailable for time parameterization");
      return false;
    }
    robot_trajectory::RobotTrajectory trajectory(move_group_.getRobotModel(), kArmGroup);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.15, 0.15)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to time-parameterize Cartesian trajectory");
      return false;
    }
    trajectory.getRobotTrajectoryMsg(trajectory_message);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_message;
    return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  UdpSceneReceiver scene_receiver_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  double pregrasp_clearance_{};
  double lift_distance_{};
  double cartesian_step_{};
  double minimum_fraction_{};
  double gripper_open_{};
  double gripper_closed_{};
  double grasp_settle_{};
  double hold_time_{};
  double velocity_scaling_{};
  double acceleration_scaling_{};
  double ground_surface_z_{};
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "d1_pick_cube",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 1;
  try {
    PickCubeDemo demo(node);
    exit_code = demo.run() ? 0 : 1;
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(node->get_logger(), "Pick demo aborted: %s", exception.what());
  }

  rclcpp::shutdown();
  spinner.join();
  return exit_code;
}
