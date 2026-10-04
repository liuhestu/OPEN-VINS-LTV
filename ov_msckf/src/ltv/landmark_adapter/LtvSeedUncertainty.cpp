#include "ltv/landmark_adapter/LtvSeedUncertainty.h"
#include <algorithm>
#include <cmath>

namespace ltv {
int LtvSeedUncertainty::inputDimension(const SeedInput &u) {
  return static_cast<int>(6 * u.poses.size() + 6 * u.cameras.size() + 2 * u.observations.size());
}
Eigen::MatrixXd LtvSeedUncertainty::jacobian(const SeedInput &u, const SeedEstimate &mean) {
  if (!mean.valid)
    return Eigen::MatrixXd();
  const auto checked = LtvSeedEstimator::estimate(u);
  if (!checked.valid || (checked.landmark_B - mean.landmark_B).norm() > 1e-12 * std::max(1.0, checked.landmark_B.norm()) ||
      (checked.point_W - mean.point_W).norm() > 1e-12 * std::max(1.0, checked.point_W.norm()) ||
      (checked.information_geometry - mean.information_geometry).norm() > 1e-12 ||
      (checked.anchor_ray_B - mean.anchor_ray_B).norm() > 1e-12 || std::abs(checked.range - mean.range) > 1e-12 * checked.range)
    return Eigen::MatrixXd();
  const int n = inputDimension(u), camera_start = static_cast<int>(6 * u.poses.size());
  const int bearing_start = camera_start + static_cast<int>(6 * u.cameras.size());
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(3, n);
  for (size_t k = 0; k < u.observations.size(); ++k) {
    const auto &o = u.observations[k];
    const auto &p = u.poses[o.pose_index];
    const auto &c = u.cameras[o.camera_index];
    Eigen::Vector3d d = p.R_WB * c.R_BC * o.bearing_C;
    Eigen::Vector3d delta = mean.point_W - p.p_WB - p.R_WB * c.p_BC;
    Eigen::Matrix3d h = Eigen::Matrix3d::Identity() - d * d.transpose();
    Eigen::Matrix3d direction_effect = d.dot(delta) * Eigen::Matrix3d::Identity() + d * delta.transpose();
    const int pi = static_cast<int>(6 * o.pose_index), ci = camera_start + static_cast<int>(6 * o.camera_index);
    rhs.block<3, 3>(0, pi) +=
        h * (-p.R_WB * LtvSeedEstimator::skew(c.p_BC)) + direction_effect * (-p.R_WB * LtvSeedEstimator::skew(c.R_BC * o.bearing_C));
    rhs.block<3, 3>(0, pi + 3) += h;
    rhs.block<3, 3>(0, ci) += direction_effect * (-p.R_WB * c.R_BC * LtvSeedEstimator::skew(o.bearing_C));
    rhs.block<3, 3>(0, ci + 3) += h * p.R_WB;
    rhs.block<3, 2>(0, bearing_start + static_cast<int>(2 * k)) +=
        direction_effect * p.R_WB * c.R_BC * LtvSeedEstimator::tangentBasis(o.bearing_C);
  }
  const auto &execution = u.poses[u.execution_pose_index];
  Eigen::MatrixXd result = execution.R_WB.transpose() * mean.information_geometry.ldlt().solve(rhs);
  result.block<3, 3>(0, static_cast<int>(6 * u.execution_pose_index)) += LtvSeedEstimator::skew(mean.landmark_B);
  result.block<3, 3>(0, static_cast<int>(6 * u.execution_pose_index + 3)) -= execution.R_WB.transpose();
  return result;
}
SeedUncertainty LtvSeedUncertainty::propagate(const SeedInput &u, const SeedEstimate &mean, const Eigen::MatrixXd &cov) {
  SeedUncertainty out;
  out.reason = "INVALID_COVARIANCE";
  const int n = inputDimension(u);
  if (!mean.valid || cov.rows() != n || cov.cols() != n || !cov.allFinite())
    return out;
  const double scale = std::max(1.0, cov.norm());
  if ((cov - cov.transpose()).norm() > 1e-10 * scale)
    return out;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(0.5 * (cov + cov.transpose()));
  if (eig.info() != Eigen::Success)
    return out;
  out.covariance_min_eigenvalue = eig.eigenvalues()(0);
  if (out.covariance_min_eigenvalue < -1e-12 * scale)
    return out;
  out.jacobian = jacobian(u, mean);
  if (out.jacobian.rows() != 3 || out.jacobian.cols() != n) {
    out.reason = "INPUT_MEAN_MISMATCH";
    return out;
  }
  out.covariance_B = out.jacobian * (0.5 * (cov + cov.transpose())) * out.jacobian.transpose();
  out.covariance_B = (0.5 * (out.covariance_B + out.covariance_B.transpose())).eval();
  const double variance = mean.anchor_ray_B.dot(out.covariance_B * mean.anchor_ray_B);
  if (!out.covariance_B.allFinite() || variance < -1e-12 * scale)
    return out;
  out.sigma_parallel = std::sqrt(std::max(0.0, variance));
  out.relative_parallel_risk = out.sigma_parallel / mean.range;
  out.valid = std::isfinite(out.relative_parallel_risk);
  out.reason = out.valid ? "LOCAL_LINEAR_RISK_NOT_CALIBRATED_CI" : "NONFINITE";
  return out;
}
} // namespace ltv
