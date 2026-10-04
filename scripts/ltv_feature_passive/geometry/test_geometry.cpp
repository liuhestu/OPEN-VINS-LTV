#include "ltv/landmark_adapter/LtvSeedEstimator.h"
#include "ltv/landmark_adapter/LtvSeedUncertainty.h"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
using namespace ltv;
static Eigen::Matrix3d exp3(const Eigen::Vector3d &v) {
  return v.norm() == 0 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
}
static void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
static SeedInput perturb(SeedInput u, int i, double h) {
  int poses = 6 * u.poses.size(), cameras = 6 * u.cameras.size();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  if (i < poses) {
    int j = i / 6, k = i % 6;
    v(k % 3) = h;
    if (k < 3)
      u.poses[j].R_WB *= exp3(v);
    else
      u.poses[j].p_WB += v;
  } else if (i < poses + cameras) {
    int j = (i - poses) / 6, k = (i - poses) % 6;
    v(k % 3) = h;
    if (k < 3)
      u.cameras[j].R_BC *= exp3(v);
    else
      u.cameras[j].p_BC += v;
  } else {
    int j = (i - poses - cameras) / 2, k = (i - poses - cameras) % 2;
    auto basis = LtvSeedEstimator::tangentBasis(u.observations[j].bearing_C);
    u.observations[j].bearing_C = (u.observations[j].bearing_C + h * basis.col(k)).normalized();
  }
  return u;
}
int main() {
  SeedInput u;
  u.poses.resize(3);
  u.cameras.resize(2);
  u.execution_pose_index = 2;
  u.cameras[0].R_BC = exp3(Eigen::Vector3d(.1, -.2, .04));
  u.cameras[0].p_BC = Eigen::Vector3d(.12, .02, -.03);
  u.cameras[1] = u.cameras[0];
  u.cameras[1].p_BC += Eigen::Vector3d(.2, .03, 0);
  Eigen::Vector3d f(2, 1, 5);
  for (int j = 0; j < 3; ++j) {
    u.poses[j].R_WB = exp3(Eigen::Vector3d(.1 * j, .2, -.05));
    u.poses[j].p_WB = Eigen::Vector3d(.3 * j, .03 * j, 0);
    u.poses[j].t = .1 * j;
    for (int k = 0; k < 2; ++k) {
      SeedObservation o;
      o.pose_index = j;
      o.camera_index = k;
      o.bearing_C =
          (u.cameras[k].R_BC.transpose() * (u.poses[j].R_WB.transpose() * (f - u.poses[j].p_WB) - u.cameras[k].p_BC)).normalized();
      u.observations.push_back(o);
    }
  }
  u.anchor_observation = 4;
  auto mean = LtvSeedEstimator::estimate(u);
  require(mean.valid, "mean valid");
  require((mean.point_W - f).norm() < 1e-10, "nonzero lever arm exact recovery");
  require(std::abs(mean.range - mean.depth_z) > .01, "range and optical depth differ");
  auto J = LtvSeedUncertainty::jacobian(u, mean);
  double maxerr = 0;
  for (double h : {1e-5, 1e-6})
    for (int k = 0; k < J.cols(); ++k) {
      auto plus = LtvSeedEstimator::estimate(perturb(u, k, h)), minus = LtvSeedEstimator::estimate(perturb(u, k, -h));
      Eigen::Vector3d fd = (plus.landmark_B - minus.landmark_B) / (2 * h);
      maxerr = std::max(maxerr, (fd - J.col(k)).norm());
      require((fd - J.col(k)).norm() < 2e-5, "composite Jacobian finite difference");
    }
  SeedInput transformed = u;
  Eigen::Matrix3d G = exp3(Eigen::Vector3d(.5, -.3, .2));
  Eigen::Vector3d q(4, -2, 3);
  for (auto &p : transformed.poses) {
    p.p_WB = G * p.p_WB + q;
    p.R_WB = G * p.R_WB;
  }
  require((LtvSeedEstimator::estimate(transformed).landmark_B - mean.landmark_B).norm() < 1e-10, "gauge mean invariance");
  // Common world gauge columns include pose cross correlations and the execution pose only once.
  Eigen::MatrixXd gauge = Eigen::MatrixXd::Zero(J.cols(), 6);
  for (int k = 0; k < 3; ++k) {
    gauge.block<3, 3>(6 * k, 0) = u.poses[k].R_WB.transpose();
    gauge.block<3, 3>(6 * k + 3, 0) = -LtvSeedEstimator::skew(u.poses[k].p_WB);
    gauge.block<3, 3>(6 * k + 3, 3) = Eigen::Matrix3d::Identity();
  }
  require((J * gauge).norm() < 1e-9, "common gauge cancels in body seed");
  auto uncertainty = LtvSeedUncertainty::propagate(u, mean, 1e-4 * gauge * gauge.transpose());
  require(uncertainty.valid && uncertainty.covariance_B.norm() < 1e-12, "singular covariance accepted without jitter");
  Eigen::MatrixXd bad = Eigen::MatrixXd::Identity(J.cols(), J.cols());
  bad(0, 0) = -1;
  require(!LtvSeedUncertainty::propagate(u, mean, bad).valid, "indefinite covariance rejected");
  SeedInput degenerate = u;
  degenerate.observations.resize(2);
  degenerate.anchor_observation = 0;
  degenerate.observations[1] = degenerate.observations[0];
  require(LtvSeedEstimator::estimate(degenerate).reason == "DUPLICATE_CAMERA_EVENT", "duplicate event rejected");
  degenerate.observations[1].pose_index = 1;
  degenerate.poses[1] = degenerate.poses[0];
  require(LtvSeedEstimator::estimate(degenerate).reason == "DEGENERATE", "no false confidence from duplicate ray");
  auto stale = mean;
  stale.landmark_B.x() += 1;
  require(LtvSeedUncertainty::jacobian(u, stale).size() == 0, "stale mean rejected");
  SeedInput invalid;
  require(LtvSeedUncertainty::jacobian(invalid, mean).size() == 0, "forged valid mean cannot index invalid input");
  SeedInput future = u;
  future.poses[0].t = 1;
  require(!LtvSeedEstimator::estimate(future).valid, "future pose rejected");
  std::cout << "PASS geometry Jacobian_max_error=" << maxerr << " gauge_norm=" << (J * gauge).norm() << "\n";
}
