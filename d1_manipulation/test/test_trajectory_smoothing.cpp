#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include "d1_manipulation/trajectory_smoothing.hpp"

namespace
{

moveit::core::RobotModelPtr makeModel(double max_jerk)
{
  std::ostringstream urdf;
  urdf << "<robot name='test'><link name='base'/>";
  for (int index = 0; index < 6; ++index) {
    urdf << "<link name='link" << index << "'/>"
         << "<joint name='Joint" << index << "' type='revolute'>"
         << "<parent link='" << (index == 0 ? "base" : "link" + std::to_string(index - 1))
         << "'/><child link='link" << index << "'/><axis xyz='0 0 1'/>"
         << "<limit lower='-3.14' upper='3.14' effort='10' velocity='2'/></joint>";
  }
  urdf << "</robot>";
  auto urdf_model = urdf::parseURDF(urdf.str());
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!urdf_model || !srdf_model->initString(
      *urdf_model,
      "<robot name='test'><group name='arm'><chain base_link='base' tip_link='link5'/></group></robot>")) {
    return nullptr;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  for (int index = 0; index < 6; ++index) {
    const std::string name = "Joint" + std::to_string(index);
    auto* joint = model->getJointModel(name);
    auto bounds = joint->getVariableBounds(name);
    bounds.velocity_bounded_ = true;
    bounds.max_velocity_ = 1.0;
    bounds.min_velocity_ = -1.0;
    bounds.acceleration_bounded_ = true;
    bounds.max_acceleration_ = 0.8;
    bounds.min_acceleration_ = -0.8;
    bounds.jerk_bounded_ = true;
    bounds.max_jerk_ = max_jerk;
    bounds.min_jerk_ = -max_jerk;
    joint->setVariableBounds(name, bounds);
  }
  return model;
}

TEST(TrajectorySmoothing, ProducesStrictTimingAcrossJerkSweep)
{
  const std::vector<std::vector<double>> positions{
    {0.20, -0.30, 0.25, 0.10, -0.10, 0.15},
    {0.45, -0.55, 0.50, 0.25, -0.20, 0.30},
    {0.55, -0.65, 0.60, 0.30, -0.25, 0.35}};
  for (const double max_jerk : {
      0.03, 0.05, 0.075, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00, 1.60}) {
    SCOPED_TRACE("max_jerk=" + std::to_string(max_jerk));
    auto model = makeModel(max_jerk);
    ASSERT_TRUE(model);
    robot_trajectory::RobotTrajectory trajectory(model, "arm");
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    trajectory.addSuffixWayPoint(state, 0.0);
    for (const auto& point : positions) {
      state.setJointGroupPositions("arm", point);
      trajectory.addSuffixWayPoint(state, 0.1);
    }
    const std::string label = "synthetic_jerk_" + std::to_string(max_jerk);
    ASSERT_TRUE(d1_manipulation::retimeAndSmoothTrajectory(
      trajectory, 0.15, 0.15, rclcpp::get_logger("trajectory_smoothing_test"), label.c_str()));
    const auto quality = d1_manipulation::trajectoryQuality(trajectory);
    EXPECT_GT(quality.duration_s, 0.0);
    EXPECT_LE(quality.max_velocity, 0.15 + 1e-6);
    EXPECT_LE(quality.max_acceleration, 0.15 + 1e-6);
    EXPECT_LE(quality.max_waypoint_jerk, max_jerk + 1e-6);
    for (std::size_t index = 1; index < trajectory.getWayPointCount(); ++index) {
      EXPECT_GT(trajectory.getWayPointDurationFromPrevious(index), 0.0);
    }
  }
}

}  // namespace
