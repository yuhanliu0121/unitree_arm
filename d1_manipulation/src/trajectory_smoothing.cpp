#include "d1_manipulation/trajectory_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>

namespace d1_manipulation
{

TrajectoryQuality trajectoryQuality(const robot_trajectory::RobotTrajectory& trajectory)
{
  TrajectoryQuality quality;
  const auto* group = trajectory.getGroup();
  if (!group) return quality;
  const auto& indices = group->getVariableIndexList();
  std::vector<double> previous_acceleration(indices.size(), 0.0);
  bool have_previous = false;
  for (std::size_t waypoint = 0; waypoint < trajectory.getWayPointCount(); ++waypoint) {
    const auto& state = trajectory.getWayPoint(waypoint);
    const double dt = trajectory.getWayPointDurationFromPrevious(waypoint);
    quality.duration_s += dt;
    for (std::size_t joint = 0; joint < indices.size(); ++joint) {
      const double velocity = state.getVariableVelocity(indices[joint]);
      const double acceleration = state.getVariableAcceleration(indices[joint]);
      quality.max_velocity = std::max(quality.max_velocity, std::abs(velocity));
      quality.max_acceleration = std::max(quality.max_acceleration, std::abs(acceleration));
      if (have_previous && dt > 1e-9) {
        quality.max_waypoint_jerk = std::max(
          quality.max_waypoint_jerk,
          std::abs(acceleration - previous_acceleration[joint]) / dt);
      }
      previous_acceleration[joint] = acceleration;
    }
    have_previous = true;
  }
  return quality;
}

namespace
{

void logQuality(
  const rclcpp::Logger& logger, const char* label, const char* stage,
  const TrajectoryQuality& quality)
{
  RCLCPP_INFO(
    logger,
    "Trajectory quality %s/%s: duration=%.3fs max|v|=%.3frad/s "
    "max|a|=%.3frad/s^2 max waypoint jerk=%.3frad/s^3",
    label, stage, quality.duration_s, quality.max_velocity, quality.max_acceleration,
    quality.max_waypoint_jerk);
}

}  // namespace

bool retimeAndSmoothTrajectory(
  robot_trajectory::RobotTrajectory& trajectory,
  double velocity_scaling,
  double acceleration_scaling,
  const rclcpp::Logger& logger,
  const char* label)
{
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling)) {
    RCLCPP_ERROR(logger, "IPTP time parameterization failed for %s", label);
    return false;
  }
  const auto before = trajectoryQuality(trajectory);
  if (!trajectory_processing::RuckigSmoothing::applySmoothing(trajectory)) {
    RCLCPP_ERROR(logger, "Ruckig smoothing failed for %s; refusing trajectory execution", label);
    return false;
  }
  logQuality(logger, label, "IPTP", before);
  logQuality(logger, label, "Ruckig", trajectoryQuality(trajectory));
  return true;
}

}  // namespace d1_manipulation
