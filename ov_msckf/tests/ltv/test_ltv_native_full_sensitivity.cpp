#include "ltv/observer/ltv_observer.h"
#include <cassert>
#include <iostream>
using namespace ltv;
std::string fixture = "bounded";
LtvObserver run(const Eigen::VectorXd &bias, const Eigen::Vector3d &seed_delta, const Eigen::Vector3d &last_bearing_delta) {
  LtvObserver o;
  LtvConfig c;
  c.enable = true;
  c.min_features = 1;
  c.max_features = 1;
  c.q_landmark = 17.;
  // This synthetic derivative fixture uses a bounded gain metric. Default
  // V_o=1e6 with q=17 exceeds the native substep cap, as attempt1 demonstrated.
  c.v_landmark = c.v_velocity = c.v_gravity = .02;
  if (fixture == "production") {
    c.q_landmark = 1e-4;
    c.v_landmark = c.v_velocity = c.v_gravity = 1e6;
  }
  if (fixture == "reject")
    c.v_landmark = c.v_velocity = c.v_gravity = 1e6;
  c.max_camera_substeps = 100;
  o.configure(c);
  o.start(0);
  assert(o.enableControlledFeatures(1));
  LtvControlledFeatures control;
  control.epoch = 1;
  control.imu_timestamp = 0;
  control.retained_ids = {1};
  LtvLandmarkSeed seed;
  seed.feature_id = 1;
  seed.apply_mean = true;
  seed.mean_body = Eigen::Vector3d(.5, -.2, 4.) + seed_delta;
  control.births = {seed};
  LtvFeatureObservation z;
  z.feature_id = 1;
  z.normalized_coordinate = Eigen::Vector3d(.15, -.03, 1.3);
  assert(o.updateFeaturesControlled(0, 0, {z}, Eigen::Matrix3d::Identity(), Eigen::Vector3d(.1, 0, 0), control).accepted);
  control.births.clear();
  for (int frame = 1; frame <= 3; ++frame) {
    for (int step = 0; step < 5; ++step)
      o.propagateImu(.01, Eigen::Vector3d(.2, -.1, .4), Eigen::Vector3d(.1, -.2, .15), bias.head<3>(), bias.tail<3>());
    control.imu_timestamp = .05 * frame;
    z.normalized_coordinate = Eigen::Vector3d(.15 + .01 * frame, -.03, 1.3);
    if (frame == 3)
      z.normalized_coordinate += last_bearing_delta;
    const Eigen::Vector3d ray = z.normalized_coordinate.normalized();
    const Eigen::Matrix3d pi = Eigen::Matrix3d::Identity() - ray * ray.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rate(c.q_landmark * pi * o.covariance().topLeftCorner<3, 3>() * pi);
    const int required = std::max(1, static_cast<int>(std::ceil(.05 * rate.eigenvalues().maxCoeff() / c.camera_euler_safety)));
    const auto result =
        o.updateFeaturesControlled(.05 * frame, .05 * frame, {z}, Eigen::Matrix3d::Identity(), Eigen::Vector3d(.1, 0, 0), control);
    if (!result.accepted) {
      std::cerr << "frame=" << frame << " reason=" << result.reason << " reset=" << static_cast<int>(result.snapshot.last_reset_reason)
                << " imu_started=" << o.started() << " required_substeps=" << required << " max_substeps=" << c.max_camera_substeps << "\n";
    }
    if (fixture == "reject") {
      assert(!result.accepted);
      assert(required > c.max_camera_substeps);
      return o;
    }
    assert(result.accepted);
  }
  return o;
}
int main(int argc, char **argv) {
  assert(argc == 2 || argc == 3);
  if (argc == 3)
    fixture = argv[2];
  setenv("LTV_JOINT_SHADOW_PATH", argv[1], 1);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const Eigen::Vector3d z = Eigen::Vector3d::Zero();
  auto base = run(zero, z, z);
  unsetenv("LTV_JOINT_SHADOW_PATH");
  if (fixture == "reject") {
    std::cout << "actual_substep_cap_rejection=PASS fixture=q17_V1e6 boundary_not_differentiable\n";
    return 0;
  }
  const Eigen::MatrixXd input = base.errorShadow().fullInputOffsetMap(), bearing = base.errorShadow().fullBearingMap();
  const double eps = 1e-6;
  double worst = 0;
  for (int k = 0; k < 6; ++k) {
    auto p = zero, m = zero;
    p[k] = eps;
    m[k] = -eps;
    worst = std::max(worst, ((run(p, z, z).state() - run(m, z, z).state()) / (2 * eps) - input.col(k)).norm());
  }
  for (int k = 0; k < 3; ++k) {
    auto p = z, m = z;
    p[k] = eps;
    m[k] = -eps;
    worst = std::max(worst, ((run(zero, p, z).state() - run(zero, m, z).state()) / (2 * eps) - input.col(6 + k)).norm());
    worst = std::max(worst, ((run(zero, z, p).state() - run(zero, z, m).state()) / (2 * eps) - bearing.col(k)).norm());
  }
  assert(worst < 1e-8);
  std::cout << "actual_observer_multi_IMU_multi_camera_full_gain_FD_max=" << worst << " fixture=" << fixture << " eps=" << eps
            << " tolerance=1e-8 seed_bias_last_raw_bearing=PASS camera_substeps=" << base.snapshot().camera_substeps << '\n';
}
