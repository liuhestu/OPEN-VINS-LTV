#include "ltv/observer/ltv_observer.h"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <random>

using namespace ltv;
static void near(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, double tolerance = 1e-11) {
  assert(a.rows() == b.rows() && a.cols() == b.cols());
  assert((a - b).norm() < tolerance);
}
int main() {
  // Independent explicit augmented reference, including a nonzero error/source
  // cross block. This also models a saved anchor: identity rows freeze it while
  // live error rows evolve, preserving cross-time and cross-point covariance.
  Eigen::MatrixXd root(5, 5);
  root << 1, 0, 0, 0, 0, .3, 1, 0, 0, 0, .2, .4, 1, 0, 0, .1, .2, .3, 1, 0, .4, .1, .2, .3, 1;
  const Eigen::MatrixXd joint = root * root.transpose();
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(3, 3), b(3, 2);
  f(1, 0) = .2;
  b << .1, .3, .2, -.1, 0, 0;
  Eigen::MatrixXd full(3, 5);
  full << f, b;
  near(shadowJointMap(joint.topLeftCorner(3, 3), f, b, joint.bottomRightCorner(2, 2), joint.topRightCorner(3, 2)),
       full * joint * full.transpose());
  // Permutation + retirement maps preserve physical covariance exactly.
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2, 3);
  m(0, 2) = 1;
  m(1, 0) = 1;
  near(shadowJointMap(joint.topLeftCorner(3, 3), m, Eigen::MatrixXd::Zero(2, 0), Eigen::MatrixXd::Zero(0, 0), Eigen::MatrixXd::Zero(3, 0)),
       m * joint.topLeftCorner(3, 3) * m.transpose());
  // Repeated source must include cross terms. The wrong independent-substep
  // model is demonstrably different, not a production fallback.
  Eigen::MatrixXd fc = Eigen::MatrixXd::Identity(3, 3) * .8, bc = Eigen::MatrixXd::Identity(3, 3) * .2;
  const Eigen::MatrixXd repeated = fc * bc + bc;
  assert((repeated * repeated.transpose() - fc * bc * bc.transpose() * fc.transpose() - bc * bc.transpose()).norm() > .1);

  const char *path = "/tmp/test_ltv_error_shadow.csv";
  std::remove(path);
  unsetenv("LTV_JOINT_SHADOW_PATH");
  LtvObserver off;
  setenv("LTV_JOINT_SHADOW_PATH", path, 1);
  LtvObserver shadow;
  unsetenv("LTV_JOINT_SHADOW_PATH");
  LtvConfig cfg;
  cfg.enable = true;
  cfg.min_features = 1;
  cfg.max_features = 2;
  off.configure(cfg);
  shadow.configure(cfg);
  off.start(0);
  shadow.start(0);
  assert(off.enableControlledFeatures(1));
  assert(shadow.enableControlledFeatures(1));
  LtvControlledFeatures control;
  control.epoch = 1;
  control.imu_timestamp = 0;
  control.retained_ids = {1};
  LtvLandmarkSeed seed;
  seed.feature_id = 1;
  seed.apply_mean = true;
  seed.mean_body = Eigen::Vector3d(.1, .2, 3);
  control.births.push_back(seed);
  LtvFeatureObservation observation;
  observation.feature_id = 1;
  observation.normalized_coordinate = Eigen::Vector3d(.1, .1, 1);
  for (int i = 0; i < 8; ++i) {
    if (i) {
      off.propagateImu(.01, Eigen::Vector3d(.1, 0, 9.8), Eigen::Vector3d(.01, .02, .03), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      shadow.propagateImu(.01, Eigen::Vector3d(.1, 0, 9.8), Eigen::Vector3d(.01, .02, .03), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero());
      control.births.clear();
    }
    control.imu_timestamp = .01 * i;
    assert(off.updateFeaturesControlled(.01 * i, .01 * i, {observation}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), control)
               .accepted);
    assert(shadow.updateFeaturesControlled(.01 * i, .01 * i, {observation}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), control)
               .accepted);
    near(off.state(), shadow.state(), 1e-30);
    near(off.covariance(), shadow.covariance(), 1e-30);
  }
  assert(shadow.errorShadow().cameraSourceMap().rows() == 9);
  assert(shadow.errorShadow().cameraSourceMap().allFinite());
  const Eigen::Vector3d acc(.1, 0, 9.8), gyro(.01, .02, .03);
  const LtvObserver base = off;
  shadow.propagateImu(.01, acc, gyro, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  off.propagateImu(.01, acc, gyro, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const Eigen::MatrixXd input_map = shadow.errorShadow().imuSourceMap();
  const double step = 1e-7;
  for (int axis = 0; axis < 6; ++axis) {
    LtvObserver plus = base, minus = base;
    Eigen::Vector3d ap = acc, am = acc, gp = gyro, gm = gyro;
    if (axis < 3) {
      ap[axis] += step;
      am[axis] -= step;
    } else {
      gp[axis - 3] += step;
      gm[axis - 3] -= step;
    }
    plus.propagateImu(.01, ap, gp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    minus.propagateImu(.01, am, gm, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    near((plus.state() - minus.state()) / (2 * step), input_map.col(axis), 1e-8);
  }
  // Native production Observer controlled one-Euler-step calibration. Fixed
  // prior/gain and corrected inputs only; not real landmark truth calibration.
  // Contract: 1000 IID samples, seed 20261008, sigma 1e-4, relative Frobenius
  // tolerance 0.12. Six source directions share one draw across all state rows.
  std::mt19937 generator(20261008);
  std::normal_distribution<double> normal(0, 1e-4);
  Eigen::MatrixXd errors(off.state().size(), 1000);
  for (int sample = 0; sample < 1000; ++sample) {
    Eigen::VectorXd source(6);
    for (int axis = 0; axis < 6; ++axis)
      source[axis] = normal(generator);
    LtvObserver trial = base;
    trial.propagateImu(.01, acc + source.head<3>(), gyro + source.tail<3>(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    errors.col(sample) = trial.state() - off.state();
    near(errors.col(sample), input_map * source, 1e-12);
  }
  errors.colwise() -= errors.rowwise().mean();
  const Eigen::MatrixXd sample_cov = errors * errors.transpose() / 999;
  const Eigen::MatrixXd predicted_cov = 1e-8 * input_map * input_map.transpose();
  const double relative = (sample_cov - predicted_cov).norm() / predicted_cov.norm();
  assert(relative < .12);
  std::cout << "native_imu_mc_relative_covariance_error=" << relative << " samples=1000 seed=20261008\n";
  std::cout << "PASS joint reference, anchor, lifecycle, repeated source counterexample, actual Observer exact OFF\n";
}
