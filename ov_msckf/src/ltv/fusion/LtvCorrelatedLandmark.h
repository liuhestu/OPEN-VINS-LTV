#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <stdexcept>
namespace ltv {
// Pure finite-model algebra; no State mutation, no production enable switch.
// All P/N/R inputs must be in the SAME declared error chart. An unvalidated
// mean or production covariance cannot acquire qualification by calling this.
struct CorrelatedLandmarkMoments {
  Eigen::Matrix3d R;
  Eigen::MatrixXd N;
};
inline CorrelatedLandmarkMoments correlatedLandmarkMoments(const Eigen::Matrix3d &tt, const Eigen::Matrix3d &aa, const Eigen::Matrix3d &ta,
                                                           const Eigen::Matrix3d &a, const Eigen::MatrixXd &xt, const Eigen::MatrixXd &xa) {
  if (xt.cols() != 3 || xa.cols() != 3 || xt.rows() != xa.rows())
    throw std::invalid_argument("landmark cross chart dimensions");
  CorrelatedLandmarkMoments m;
  m.R = tt + a * aa * a.transpose() - ta * a.transpose() - a * ta.transpose();
  m.R = (.5 * (m.R + m.R.transpose())).eval();
  m.N = xt - xa * a.transpose();
  return m;
}
inline Eigen::MatrixXd landmarkVisualNoiseCross(const Eigen::MatrixXd &current_visual, const Eigen::MatrixXd &anchor_visual,
                                                const Eigen::Matrix3d &a) {
  if (current_visual.rows() != 3 || anchor_visual.rows() != 3 || current_visual.cols() != anchor_visual.cols())
    throw std::invalid_argument("landmark visual cross dimensions");
  return current_visual - a * anchor_visual;
}
struct CorrelatedUpdateResult {
  Eigen::MatrixXd innovation, gain, posterior;
  Eigen::VectorXd correction;
};
inline CorrelatedUpdateResult correlatedUpdate(const Eigen::MatrixXd &p, const Eigen::MatrixXd &h, const Eigen::MatrixXd &r,
                                               const Eigen::MatrixXd &n, const Eigen::VectorXd &innovation_minus_declared_mean) {
  if (p.rows() != p.cols() || h.cols() != p.rows() || r.rows() != h.rows() || r.cols() != h.rows() || n.rows() != p.rows() ||
      n.cols() != h.rows() || innovation_minus_declared_mean.size() != h.rows())
    throw std::invalid_argument("correlated update dimensions");
  const Eigen::MatrixXd joint = (Eigen::MatrixXd(p.rows() + r.rows(), p.cols() + r.cols()) << p, n, n.transpose(), r).finished();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> prior(joint);
  if (!joint.allFinite() || prior.info() != Eigen::Success || prior.eigenvalues().minCoeff() < -1e-10 * std::max(1e-12, joint.norm()))
    throw std::invalid_argument("incompatible joint state/noise covariance");
  CorrelatedUpdateResult out;
  out.innovation = h * p * h.transpose() + r + h * n + n.transpose() * h.transpose();
  out.innovation = (.5 * (out.innovation + out.innovation.transpose())).eval();
  Eigen::LDLT<Eigen::MatrixXd> solve(out.innovation);
  if (solve.info() != Eigen::Success || solve.vectorD().minCoeff() <= 1e-14 * std::max(1e-12, out.innovation.norm()))
    throw std::invalid_argument("singular/nonpositive correlated innovation");
  const Eigen::MatrixXd cross = p * h.transpose() + n;
  out.gain = solve.solve(cross.transpose()).transpose();
  out.posterior = p - cross * solve.solve(cross.transpose());
  out.posterior = (.5 * (out.posterior + out.posterior.transpose())).eval();
  out.correction = out.gain * innovation_minus_declared_mean;
  if (!out.posterior.allFinite() || !out.correction.allFinite())
    throw std::runtime_error("nonfinite correlated posterior");
  return out;
}
inline Eigen::Matrix3d landmarkSkew(const Eigen::Vector3d &p) {
  Eigen::Matrix3d s;
  s << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
  return s;
}
// Columns [theta_t,p_t,theta_a,p_a], m=-estimated local perturbation. For a
// nonzero actual true-error mean, compose this with M(e0)^{-1} before pairing
// with a true-error covariance. Clock/calibration are fixed in this module.
inline Eigen::Matrix<double, 3, 12> landmarkProgramPoseJacobian(const Eigen::Matrix3d &rt, const Eigen::Vector3d &pt,
                                                                const Eigen::Matrix3d &ra, const Eigen::Vector3d &pa,
                                                                const Eigen::Vector3d &anchor_point) {
  const Eigen::Matrix3d a = rt * ra.transpose();
  const Eigen::Vector3d predicted = rt * (ra.transpose() * anchor_point + pa - pt);
  Eigen::Matrix<double, 3, 12> h;
  h << landmarkSkew(predicted), -rt, -a * landmarkSkew(anchor_point), rt;
  return h;
}
} // namespace ltv
