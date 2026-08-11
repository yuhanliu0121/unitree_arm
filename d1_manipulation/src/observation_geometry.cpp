#include "d1_manipulation/observation_geometry.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace d1_manipulation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumNorm = 1e-9;

void requireFinite(const Eigen::Vector3d& vector, const char* name)
{
  if (!vector.allFinite()) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}
}  // namespace

std::vector<ObservationCandidate> generateObservationCandidates(
  const Eigen::Vector3d& target,
  const Eigen::Vector3d& up_value,
  const std::vector<double>& beta_degrees,
  const std::vector<double>& alpha_degrees,
  const std::vector<double>& distances_m)
{
  requireFinite(target, "target");
  requireFinite(up_value, "up");
  if (up_value.norm() < kMinimumNorm) {
    throw std::invalid_argument("up direction has zero length");
  }
  if (beta_degrees.empty() || alpha_degrees.empty() || distances_m.empty()) {
    throw std::invalid_argument("observation sample lists must not be empty");
  }

  const Eigen::Vector3d up = up_value.normalized();
  const Eigen::Vector3d horizontal_value = target - target.dot(up) * up;
  if (horizontal_value.norm() < kMinimumNorm) {
    throw std::invalid_argument(
      "target has no horizontal direction from the planning-frame origin");
  }
  const Eigen::Vector3d base_to_target_horizontal = horizontal_value.normalized();
  const Eigen::Vector3d target_to_base_horizontal = -base_to_target_horizontal;

  std::vector<ObservationCandidate> candidates;
  candidates.reserve(beta_degrees.size() * alpha_degrees.size() * distances_m.size());
  for (const double beta_deg : beta_degrees) {
    if (!std::isfinite(beta_deg)) {
      throw std::invalid_argument("beta samples must be finite");
    }
    const double beta = beta_deg * kPi / 180.0;
    const Eigen::Vector3d horizontal =
      Eigen::AngleAxisd(beta, up) * target_to_base_horizontal;
    for (const double alpha_deg : alpha_degrees) {
      if (!std::isfinite(alpha_deg) || alpha_deg <= 0.0 || alpha_deg >= 90.0) {
        throw std::invalid_argument("alpha samples must be finite and inside (0, 90) degrees");
      }
      const double alpha = alpha_deg * kPi / 180.0;
      const Eigen::Vector3d target_to_camera =
        std::cos(alpha) * horizontal + std::sin(alpha) * up;
      const Eigen::Vector3d camera_forward = -target_to_camera;
      Eigen::Vector3d image_up = up - up.dot(camera_forward) * camera_forward;
      if (image_up.norm() < kMinimumNorm) {
        throw std::invalid_argument("camera forward is parallel to the up direction");
      }
      image_up.normalize();
      const Eigen::Vector3d camera_y = -image_up;
      const Eigen::Vector3d camera_x = camera_y.cross(camera_forward).normalized();

      Eigen::Matrix3d rotation;
      rotation.col(0) = camera_x;
      rotation.col(1) = camera_y;
      rotation.col(2) = camera_forward;
      if (rotation.determinant() < 0.999999) {
        throw std::runtime_error("constructed camera frame is not right-handed");
      }

      for (const double distance_m : distances_m) {
        if (!std::isfinite(distance_m) || distance_m <= 0.0) {
          throw std::invalid_argument("distance samples must be finite and positive");
        }
        ObservationCandidate candidate;
        candidate.beta_deg = beta_deg;
        candidate.alpha_deg = alpha_deg;
        candidate.distance_m = distance_m;
        candidate.camera_position = target + distance_m * target_to_camera;
        candidate.planning_from_camera = rotation;
        candidates.push_back(candidate);
      }
    }
  }
  return candidates;
}

}  // namespace d1_manipulation
