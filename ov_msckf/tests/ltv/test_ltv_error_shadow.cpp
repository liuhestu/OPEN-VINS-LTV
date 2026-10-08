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
  std::remove("/tmp/test_ltv_error_shadow.csv.matrices.bin");
  std::remove("/tmp/test_ltv_error_shadow.csv.matrices.jsonl");
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
  // Camera finite differences hold the nominal gain schedule fixed. This
  // intentionally excludes the derivative of P_R/eigenvalue sanitization.
  const LtvObserver matched_shadow = shadow;
  shadow.propagateImu(.01, Eigen::Vector3d(.1, 0, 9.8), Eigen::Vector3d(.01, .02, .03), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const Eigen::VectorXd initial_camera = shadow.state();
  control.imu_timestamp = .08;
  assert(shadow.updateFeaturesControlled(.08, .08, {observation}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), control).accepted);
  const auto &gains = shadow.errorShadow().cameraGains();
  const auto frozen_camera = [&gains, &initial_camera](const Eigen::Vector3d &u_raw) {
    const Eigen::Vector3d u = u_raw.normalized();
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3, initial_camera.size());
    c.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() - u * u.transpose();
    Eigen::VectorXd x = initial_camera;
    for (const auto &gain : gains)
      x -= gain * c * x;
    return x;
  };
  const Eigen::Vector3d nominal_u = observation.normalized_coordinate.normalized();
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d up = nominal_u, um = nominal_u;
    up[axis] += 1e-7;
    um[axis] -= 1e-7;
    near((frozen_camera(up) - frozen_camera(um)) / 2e-7, shadow.errorShadow().cameraSourceMap().col(axis), 1e-8);
  }
  // Restore the matched state before the separate IMU calibration below.
  shadow = matched_shadow;
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
  // Nonzero saved-anchor synthetic source contribution and identity invalidation.
  const char *life_path = "/tmp/test_ltv_error_shadow_lifecycle.csv";
  std::remove(life_path);
  std::remove("/tmp/test_ltv_error_shadow_lifecycle.csv.matrices.bin");
  std::remove("/tmp/test_ltv_error_shadow_lifecycle.csv.matrices.jsonl");
  setenv("LTV_JOINT_SHADOW_PATH", life_path, 1);
  LtvErrorShadow lifecycle;
  unsetenv("LTV_JOINT_SHADOW_PATH");
  lifecycle.reset(0);
  lifecycle.setEpoch(11);
  lifecycle.imu(0, .01, Eigen::MatrixXd::Identity(6, 6), Eigen::MatrixXd::Identity(6, 6));
  Eigen::MatrixXd birth = Eigen::MatrixXd::Zero(9, 6);
  birth.topLeftCorner<3, 3>().setIdentity();
  birth.bottomRows(6).setIdentity();
  lifecycle.lifecycle(.01, birth, 1, 0, {1});
  lifecycle.camera(.01, Eigen::MatrixXd::Identity(9, 9) * .8, Eigen::MatrixXd::Zero(9, 3), 1, {1});
  Eigen::MatrixXd two_point_birth = Eigen::MatrixXd::Zero(12, 9);
  two_point_birth.topLeftCorner<3, 3>().setIdentity();
  two_point_birth.block<3, 3>(3, 0).setIdentity();
  two_point_birth.bottomRightCorner(6, 6).setIdentity();
  lifecycle.lifecycle(.015, two_point_birth, 1, 0, {2, 1});
  lifecycle.camera(.015, Eigen::MatrixXd::Identity(12, 12) * .9, Eigen::MatrixXd::Zero(12, 6), 1, {2, 1});
  Eigen::MatrixXd retire = Eigen::MatrixXd::Zero(6, 12);
  retire.rightCols(6).setIdentity();
  lifecycle.lifecycle(.02, retire, 0, 1, {});
  bool rejects_revival = false;
  try {
    lifecycle.lifecycle(.03, birth, 1, 0, {1});
  } catch (const std::runtime_error &) {
    rejects_revival = true;
  }
  assert(rejects_revival);
  lifecycle.reset(.04);
  lifecycle.setEpoch(12);
  lifecycle.lifecycle(.04, birth, 1, 0, {1});
  lifecycle.camera(.04, Eigen::MatrixXd::Identity(9, 9), Eigen::MatrixXd::Zero(9, 3), 0, {1});
  std::cout << "PASS joint reference, anchor, lifecycle, repeated source counterexample, actual Observer exact OFF\n";
}
