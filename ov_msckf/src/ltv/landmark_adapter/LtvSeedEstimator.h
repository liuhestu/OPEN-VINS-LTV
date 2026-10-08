#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace ltv {

// All rotations map the second frame into the first; perturbations are right multiplicative.
struct SeedPose {
  Eigen::Matrix3d R_WB = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_WB = Eigen::Vector3d::Zero();
  double t = 0.0;
};
struct SeedCamera {
  Eigen::Matrix3d R_BC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_BC = Eigen::Vector3d::Zero();
};
struct SeedObservation {
  size_t pose_index = 0;
  size_t camera_index = 0;
  Eigen::Vector3d bearing_C = Eigen::Vector3d::UnitZ();
  std::string pixel_source_key;
  Eigen::Matrix2d pixel_to_tangent = Eigen::Matrix2d::Zero();
};
struct SeedInput {
  std::vector<SeedPose> poses;
  std::vector<SeedCamera> cameras;
  std::vector<SeedObservation> observations;
  size_t execution_pose_index = 0;
  size_t anchor_observation = 0;
};
struct SeedEstimate {
  bool valid = false;
  std::string reason = "UNCOMPUTED";
  Eigen::Vector3d point_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d landmark_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchor_ray_B = Eigen::Vector3d::Zero();
  Eigen::Matrix3d information_geometry = Eigen::Matrix3d::Zero();
  double condition = 0.0;
  double max_angle_rad = 0.0;
  double max_residual_rad = 0.0;
  double min_ray = 0.0;
  double range = 0.0;   // Euclidean distance from anchor camera, not optical depth.
  double depth_z = 0.0; // Anchor camera optical-axis coordinate; may be negative for signed bearings.
};
class LtvSeedEstimator {
public:
  // Numerical validity only: quality admission thresholds belong to the selector.
  static SeedEstimate estimate(const SeedInput &input);
  static Eigen::Matrix<double, 3, 2> tangentBasis(const Eigen::Vector3d &unit_bearing);
  static Eigen::Matrix3d skew(const Eigen::Vector3d &v);
};
} // namespace ltv
