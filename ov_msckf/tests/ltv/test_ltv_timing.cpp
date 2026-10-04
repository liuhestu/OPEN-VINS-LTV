#include "ltv/observer/LtvAdapter.h"
#include "native_test_support.h"
using namespace ov_msckf;
void feed(LtvAdapter &a, double begin, double end, double step = 0.005) {
  for (double t = begin; t < end + 1e-8; t += step) {
    ov_core::ImuData x;
    x.timestamp = t;
    x.am << 0.1, -0.2, 9.9;
    x.wm << 0.01, 0.02, 0.03;
    a.feed_imu(x);
  }
}
int main() {
  LtvOptions options;
  options.enabled = true;
  options.observer.min_features = 1;
  options.observer.warmup_camera_updates = 1;
  options.validate(StateOptions());
  LtvAdapter a(options);
  LtvCalibration c;
  c.offset = 0.003;
  Eigen::Vector3d ba(0.1, -0.2, 0.09), bg(0.01, 0.02, 0.03);
  const std::vector<LtvBearing> bearings = {{size_t(INT_MAX) + 100, Eigen::Vector3d(0.1, 0.2, 1)}};
  feed(a, 0, 0.3);
  auto first = a.process(0.01, bearings, c, ba, bg, 1);
  require(first.available && a.claim(first) && !a.claim(first), "one attempt stereo token");
  const Eigen::MatrixXd p0 = a.core().covariance();
  auto second = a.process(0.031, bearings, c, ba, bg, 2);
  require(second.available, "fractional endpoints");
  metric("integrated_interval", std::abs(second.integrated_seconds - 0.021), 1e-12);
  require(second.integrated_steps == 5, "five boundary intervals");
  require((a.core().covariance() - p0).norm() > 1, "persistent Riccati propagation");
  const auto snapshot = second.snapshot;
  require(!a.process(0.031, bearings, c, ba, bg, 3).available, "duplicate frame rejected");
  require(a.claim(second), "duplicate cannot alter original attempt");
  auto third = a.process(0.051, {}, c, ba, bg, 4);
  require(third.available && third.snapshot.state_features == 1, "short missed feature retained");
  require((snapshot.velocity_body - second.snapshot.velocity_body).norm() == 0, "snapshot value copy");
  auto missing = a.process(0.31, bearings, c, ba, bg, 5);
  require(!missing.available && missing.reason == "missing_imu_bracket", "no extrapolation");
  require(!a.claim(third), "pause invalidates old frame");
  feed(a, 0.305, 0.5);
  auto resumed = a.process(0.32, bearings, c, ba, bg, 6);
  require(resumed.available && resumed.epoch > first.epoch && resumed.snapshot.healthy_camera_updates == 1, "restart warmup");
  a.pause(0.34, "zupt", 7);
  auto zupt = a.process(0.36, bearings, c, ba, bg, 8);
  require(zupt.available && zupt.integrated_seconds == resumed.integrated_seconds, "pause does not integrate skipped intervals");
  ov_core::ImuData invalid;
  invalid.timestamp = 0.4;
  invalid.am.setZero();
  invalid.wm.setZero();
  a.feed_imu(invalid);
  require(a.process(0.38, bearings, c, ba, bg, 9).reason == "invalid_or_out_of_order_imu", "out of order IMU diagnosed");
  a.reset();
  require(!a.claim(zupt), "main reset invalidates token");
  feed(a, 1, 1.2);
  auto reset = a.process(1.01, bearings, c, ba, bg, 10);
  require(reset.available && reset.epoch > zupt.epoch, "main reset resumes new epoch");
  LtvAdapter gap(options);
  feed(gap, 0, 0.01);
  feed(gap, 0.2, 0.21);
  require(!gap.process(0.1, bearings, c, ba, bg, 1).available, "large interpolation gap rejected");
  LtvAdapter corrected(options);
  feed(corrected, 0, 0.1);
  LtvCalibration identity;
  corrected.process(0, bearings, identity, ba, bg, 1);
  auto result = corrected.process(0.02, bearings, identity, ba, bg, 2);
  ltv::LtvObserver reference;
  auto cfg = options.observer;
  cfg.enable = true;
  reference.configure(cfg);
  reference.start(0);
  ltv::LtvFeatureObservation f;
  f.feature_id = 0;
  f.normalized_coordinate = bearings[0].coordinate;
  reference.updateFeatures(0, 0, {f}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  for (int i = 0; i < 4; ++i)
    reference.propagateImu(0.005, Eigen::Vector3d(0.1, -0.2, 9.9), Eigen::Vector3d(0.01, 0.02, 0.03), ba, bg);
  reference.updateFeatures(0.02, 0.02, {f}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  metric("identity_bias_contract_state", (reference.state() - corrected.core().state()).norm(), 1e-10);
  metric("identity_bias_contract_P", (reference.covariance() - corrected.core().covariance()).norm(), 1e-8);
  require(result.available, "identity candidate");
  std::cout << "T6 PASS" << std::endl;
}
