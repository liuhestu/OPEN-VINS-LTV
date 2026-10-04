#include "ltv/observer/ltv_observer.h"
#include <iostream>

// Inject into a test-owned, non-const observer through its existing accessor.
// No production visibility changes: propagateImu calls the real sanitizeCovariance.
int main() {
  ltv::LtvObserver observer;
  ltv::LtvConfig config;
  config.enable = true;
  config.v_landmark = 0.3;
  config.v_velocity = 0.1;
  config.v_gravity = 0.2;
  observer.configure(config);
  observer.start(0.0);
  ltv::LtvFeatureObservation feature;
  feature.feature_id = 10;
  feature.normalized_coordinate << 0.1, 0.2, 1;
  observer.updateFeatures(0, 0, {feature}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  Eigen::MatrixXd before = 100.0 * Eigen::MatrixXd::Identity(9, 9);
  for (int i = 0; i < 9; ++i)
    for (int j = i + 1; j < 9; ++j) {
      before(i, j) = 2.0 * (i + j + 1);
      before(j, i) = -before(i, j);
    }
  const_cast<Eigen::MatrixXd &>(observer.covariance()) = before;
  const double dt = 0.001;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  A.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
  A.block<3, 3>(3, 6).setIdentity();
  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(9, 9);
  noise.topLeftCorner<3, 3>().diagonal().setConstant(config.v_landmark);
  noise.block<3, 3>(3, 3).diagonal().setConstant(config.v_velocity);
  noise.bottomRightCorner<3, 3>().diagonal().setConstant(config.v_gravity);
  const Eigen::MatrixXd propagated = before + dt * (A * before + before * A.transpose() + noise);
  const Eigen::MatrixXd expected = 0.5 * (propagated + propagated.transpose());
  // Expected symmetric part is strictly positive, so eigenvalue flooring is inactive.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(expected);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= config.covariance_floor)
    return 2;
  observer.propagateImu(dt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const double error = (observer.covariance() - expected).cwiseAbs().maxCoeff();
  const double asymmetry = (observer.covariance() - observer.covariance().transpose()).cwiseAbs().maxCoeff();
  std::cout << "actual_target_sanitize max_error=" << error << " max_asymmetry=" << asymmetry << " started=" << observer.started()
            << std::endl;
  // Full-RHS evaluation writes exactly the same floating-point sum to each
  // mirrored pair. Require that postcondition exactly: an epsilon check would
  // hide the old final in-place expression after eigensolver reconstruction.
  return observer.started() && error < 1e-10 && asymmetry == 0.0 ? 0 : 1;
}
