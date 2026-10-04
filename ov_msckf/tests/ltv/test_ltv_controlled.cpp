#include "ltv/observer/ltv_observer.h"
#include <cassert>
#include <iostream>

int main() {
  ltv::LtvConfig c;
  c.enable = true;
  ltv::LtvObserver o;
  o.configure(c);
  o.start(0);
  assert(o.enableControlledFeatures(7));
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d p(0.05, -0.02, 0.03);
  std::vector<ltv::LtvFeatureObservation> obs;
  ltv::LtvControlledFeatures ctl;
  ctl.epoch = 7;
  for (int id : {10, 20, 30}) {
    ltv::LtvFeatureObservation z;
    z.feature_id = id;
    z.normalized_coordinate = Eigen::Vector3d(0.1 * id, 0.2, 1).normalized();
    obs.push_back(z);
    ltv::LtvLandmarkSeed seed;
    seed.feature_id = id;
    seed.mean_body = p + 4.0 * z.normalized_coordinate;
    ctl.births.push_back(seed);
    ctl.retained_ids.push_back(id);
  }
  assert(o.updateFeaturesControlled(0, 0, obs, R, p, ctl).accepted);
  for (size_t i = 0; i < ctl.births.size(); ++i)
    assert((o.state().segment<3>(3 * i) - ctl.births[i].mean_body).norm() == 0);
  assert(o.state().tail<6>().norm() == 0);
  assert((o.covariance() - Eigen::MatrixXd::Identity(15, 15)).norm() == 0);
  for (int i = 0; i < 10; ++i)
    o.propagateImu(.005, Eigen::Vector3d(0.3, -.1, -9.81), Eigen::Vector3d(.1, .2, -.1), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ctl.imu_timestamp = .05;
  auto x = o.state().eval();
  auto P = o.covariance().eval();
  // A duplicate mean write, old epoch, or timestamp must fail before any mutation.
  auto duplicate = o.updateFeaturesControlled(.05, .05, obs, R, p, ctl);
  assert(!duplicate.accepted);
  assert((x - o.state()).norm() == 0 && (P - o.covariance()).norm() == 0);
  ctl.births.clear();
  ctl.epoch = 6;
  assert(!o.updateFeaturesControlled(.05, .05, obs, R, p, ctl).accepted);
  ctl.epoch = 7;
  ctl.imu_timestamp = .1;
  assert(!o.updateFeaturesControlled(.05, .05, obs, R, p, ctl).accepted);
  assert((x - o.state()).norm() == 0 && (P - o.covariance()).norm() == 0);
  ctl.imu_timestamp = .05;
  assert(o.updateFeaturesControlled(.05, .05, obs, R, p, ctl).accepted);
  x = o.state();
  P = o.covariance();
  // Zero correction duration isolates lifecycle remapping. No private state writes.
  ctl.retained_ids = {30, 10, 40};
  obs = {obs[2], obs[0]};
  ltv::LtvFeatureObservation z;
  z.feature_id = 40;
  z.normalized_coordinate = Eigen::Vector3d::UnitZ();
  obs.push_back(z);
  ltv::LtvLandmarkSeed seed;
  seed.feature_id = 40;
  seed.mean_body = Eigen::Vector3d(1, 2, 5);
  ctl.births = {seed};
  assert(o.updateFeaturesControlled(.051, .05, obs, R, p, ctl).accepted);
  const int indices[] = {6, 7, 8, 0, 1, 2, -1, -1, -1, 9, 10, 11, 12, 13, 14};
  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(15, 15);
  for (int i = 0; i < 15; ++i) {
    if (indices[i] >= 0)
      assert(std::abs(o.state()(i) - x(indices[i])) < 1e-12);
    for (int j = 0; j < 15; ++j)
      if (indices[i] >= 0 && indices[j] >= 0)
        expected(i, j) = P(indices[i], indices[j]);
  }
  expected.block<3, 3>(6, 6).setIdentity();
  const double error = (expected - o.covariance()).cwiseAbs().maxCoeff();
  assert(error < 1e-8);
  assert((o.state().segment<3>(6) - seed.mean_body).norm() == 0);
  assert(o.slotForFeature(20) == -1);
  ctl.births = {};
  ctl.retained_ids.push_back(20);
  assert(!o.updateFeaturesControlled(.052, .05, obs, R, p, ctl).accepted);
  // Empty visible set continues physical time without resetting.
  ctl.retained_ids = {30, 10, 40};
  o.propagateImu(.005, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ctl.imu_timestamp = .055;
  assert(o.updateFeaturesControlled(.055, .055, {}, R, p, ctl).accepted && o.started());
  std::cout << "PASS controlled once/causality/old mean/full cross-block/no-reset; P error=" << error << '\n';
}
