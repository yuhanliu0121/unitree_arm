#pragma once

#include <rclcpp/logger.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace d1_manipulation
{

struct TrajectoryQuality
{
  double duration_s{0.0};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double max_waypoint_jerk{0.0};
};

TrajectoryQuality trajectoryQuality(const robot_trajectory::RobotTrajectory& trajectory);

bool retimeAndSmoothTrajectory(
  robot_trajectory::RobotTrajectory& trajectory,
  double velocity_scaling,
  double acceleration_scaling,
  const rclcpp::Logger& logger,
  const char* label);

}  // namespace d1_manipulation
