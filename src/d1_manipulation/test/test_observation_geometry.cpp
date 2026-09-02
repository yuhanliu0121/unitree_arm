#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "d1_manipulation/observation_geometry.hpp"

namespace
{
constexpr double kTolerance = 1e-9;

const std::vector<double> kBeta{0.0, -5.0, 5.0, -10.0, 10.0, -15.0, 15.0};
const std::vector<double> kAlpha{45.0, 40.0, 50.0, 35.0, 55.0, 30.0, 60.0, 65.0, 70.0};
const std::vector<double> kDistance{0.35, 0.30, 0.40, 0.45, 0.50, 0.55, 0.60};
}

TEST(ObservationGeometry, PreservesConfiguredLexicographicOrder)
{
  const auto candidates = d1_manipulation::generateObservationCandidates(
    {0.4, 0.1, -0.2}, Eigen::Vector3d::UnitZ(), kBeta, kAlpha, kDistance);
  ASSERT_EQ(candidates.size(), 441U);
  EXPECT_DOUBLE_EQ(candidates[0].beta_deg, 0.0);
  EXPECT_DOUBLE_EQ(candidates[0].alpha_deg, 45.0);
  EXPECT_DOUBLE_EQ(candidates[0].distance_m, 0.35);
  EXPECT_DOUBLE_EQ(candidates[1].distance_m, 0.30);
  EXPECT_DOUBLE_EQ(candidates[7].alpha_deg, 40.0);
  EXPECT_DOUBLE_EQ(candidates[63].beta_deg, -5.0);
}

TEST(ObservationGeometry, OpticalAxisPointsAtTargetAndImageUpTracksGravity)
{
  const Eigen::Vector3d target(0.4, -0.1, -0.2);
  const Eigen::Vector3d up = Eigen::Vector3d(0.2, -0.3, 0.93).normalized();
  const auto candidates = d1_manipulation::generateObservationCandidates(
    target, up, {10.0}, {55.0}, {0.45});
  ASSERT_EQ(candidates.size(), 1U);
  const auto& candidate = candidates.front();
  const Eigen::Vector3d expected_forward =
    (target - candidate.camera_position).normalized();
  EXPECT_TRUE(candidate.planning_from_camera.col(2).isApprox(expected_forward, kTolerance));
  EXPECT_NEAR(
    (candidate.camera_position - target).norm(), candidate.distance_m, kTolerance);
  EXPECT_TRUE(
    candidate.planning_from_camera.transpose()
      .isApprox(candidate.planning_from_camera.inverse(), kTolerance));
  EXPECT_NEAR(candidate.planning_from_camera.determinant(), 1.0, kTolerance);

  Eigen::Vector3d projected_up = up - up.dot(expected_forward) * expected_forward;
  projected_up.normalize();
  EXPECT_TRUE((-candidate.planning_from_camera.col(1)).isApprox(projected_up, kTolerance));
}

TEST(ObservationGeometry, RejectsUndefinedHorizontalDirection)
{
  EXPECT_THROW(
    d1_manipulation::generateObservationCandidates(
      {0.0, 0.0, 0.3}, Eigen::Vector3d::UnitZ(), {0.0}, {45.0}, {0.35}),
    std::invalid_argument);
}
