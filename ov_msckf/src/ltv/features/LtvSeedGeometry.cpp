#include "ltv/features/LtvSeedGeometry.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace ltv {
namespace {
bool rotationValid(const Eigen::Matrix3d &r) {
  return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-8 && std::abs(r.determinant() - 1) < 1e-8;
}
double angle(double cosine) { return std::acos(std::max(-1.0, std::min(1.0, cosine))); }
} // namespace
Eigen::Matrix3d LtvSeedEstimator::skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d s;
  s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return s;
}
Eigen::Matrix<double, 3, 2> LtvSeedEstimator::tangentBasis(const Eigen::Vector3d &z) {
  Eigen::Index axis;
  z.cwiseAbs().minCoeff(&axis);
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  a(axis) = 1;
  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = (a - z * z.dot(a)).normalized();
  basis.col(1) = z.cross(basis.col(0));
  return basis;
}
SeedEstimate LtvSeedEstimator::estimate(const SeedInput &u) {
  SeedEstimate out;
  out.reason = "INVALID_INPUT";
  if (u.observations.size() < 2 || u.execution_pose_index >= u.poses.size() || u.anchor_observation >= u.observations.size())
    return out;
  const auto &execution = u.poses[u.execution_pose_index];
  for (const auto &p : u.poses)
    if (!rotationValid(p.R_WB) || !p.p_WB.allFinite() || !std::isfinite(p.t) || p.t > execution.t + 1e-9)
      return out;
  for (const auto &c : u.cameras)
    if (!rotationValid(c.R_BC) || !c.p_BC.allFinite())
      return out;
  std::vector<Eigen::Vector3d> centers, directions;
  std::set<std::pair<size_t, size_t>> events;
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (const auto &o : u.observations) {
    if (o.pose_index >= u.poses.size() || o.camera_index >= u.cameras.size() || !o.bearing_C.allFinite() ||
        std::abs(o.bearing_C.norm() - 1.0) > 1e-8)
      return out;
    if (!events.emplace(o.pose_index, o.camera_index).second) {
      out.reason = "DUPLICATE_CAMERA_EVENT";
      return out;
    }
    const auto &p = u.poses[o.pose_index];
    const auto &c = u.cameras[o.camera_index];
    centers.push_back(p.p_WB + p.R_WB * c.p_BC);
    directions.push_back(p.R_WB * c.R_BC * o.bearing_C);
    Eigen::Matrix3d h = Eigen::Matrix3d::Identity() - directions.back() * directions.back().transpose();
    out.information_geometry += h;
    rhs += h * centers.back();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(out.information_geometry);
  out.reason = "DEGENERATE";
  if (eig.info() != Eigen::Success || eig.eigenvalues()(0) <= 1e-12 * eig.eigenvalues()(2))
    return out;
  out.condition = eig.eigenvalues()(2) / eig.eigenvalues()(0);
  out.point_W = out.information_geometry.ldlt().solve(rhs);
  out.landmark_B = execution.R_WB.transpose() * (out.point_W - execution.p_WB);
  out.min_ray = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < directions.size(); ++j) {
    Eigen::Vector3d delta = out.point_W - centers[j];
    if (delta.norm() < 1e-10) {
      out.reason = "AT_CAMERA_CENTER";
      return out;
    }
    out.min_ray = std::min(out.min_ray, directions[j].dot(delta));
    out.max_residual_rad = std::max(out.max_residual_rad, angle(directions[j].dot(delta.normalized())));
    for (size_t k = 0; k < j; ++k)
      out.max_angle_rad = std::max(out.max_angle_rad, angle(directions[j].dot(directions[k])));
  }
  const auto &anchor = u.observations[u.anchor_observation];
  out.anchor_ray_B = execution.R_WB.transpose() * directions[u.anchor_observation];
  Eigen::Vector3d anchor_delta = out.point_W - centers[u.anchor_observation];
  out.range = anchor_delta.norm();
  out.depth_z = (u.cameras[anchor.camera_index].R_BC.transpose() * u.poses[anchor.pose_index].R_WB.transpose() * anchor_delta).z();
  out.valid = out.landmark_B.allFinite() && out.point_W.allFinite() && out.min_ray > 0;
  out.reason = out.valid ? "OK" : "NONPOSITIVE_RAY";
  return out;
}

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
