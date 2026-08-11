#pragma once

#include <vector>

#include <Eigen/Core>

namespace d1_manipulation
{

struct ObservationCandidate
{
  double beta_deg{};
  double alpha_deg{};
  double distance_m{};
  Eigen::Vector3d camera_position{Eigen::Vector3d::Zero()};
  // Rotation from RGB optical coordinates to the planning frame. Columns are
  // optical +X (image right), +Y (image down), and +Z (forward).
  Eigen::Matrix3d planning_from_camera{Eigen::Matrix3d::Identity()};
};

std::vector<ObservationCandidate> generateObservationCandidates(
  const Eigen::Vector3d& target,
  const Eigen::Vector3d& up,
  const std::vector<double>& beta_degrees,
  const std::vector<double>& alpha_degrees,
  const std::vector<double>& distances_m);

}  // namespace d1_manipulation
