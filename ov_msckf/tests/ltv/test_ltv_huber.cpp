#include "native_test_support.h"
int main() {
  auto state = make_state();
  state->_timestamp = 1;
  LtvFrame f;
  f.available = true;
  f.epoch = f.sequence = f.version = 1;
  f.camera_time = f.imu_time = f.cursor = 1;
  f.snapshot.frame_timestamp = f.snapshot.imu_timestamp = 1;
  f.snapshot.valid = f.snapshot.velocity_valid = f.snapshot.gravity_valid = true;
  f.snapshot.observed_features = 30;
  f.snapshot.gravity_body = state->_imu->Rot() * Eigen::Vector3d(0, 0, -9.81);
  f.snapshot.velocity_body = state->_imu->Rot() * state->_imu->vel() + Eigen::Vector3d(6, 0, 0);
  LtvOptions plain;
  plain.enabled = plain.enable_gravity = plain.enable_velocity = plain.allow_correlated_pseudomeasurements = true;
  plain.enable_nis_gate = false;
  auto robust = plain;
  robust.enable_huber = true;
  auto a = UpdaterLTV(plain).build(state, f), b = UpdaterLTV(robust).build(state, f);
  metric("G_unaffected_by_V_outlier", (a.R.topLeftCorner(2, 2) - b.R.topLeftCorner(2, 2)).norm(), 1e-14);
  metric("huber_information_weight_one_third", (b.R.bottomRightCorner(3, 3) - 3 * a.R.bottomRightCorner(3, 3)).norm(), 1e-14);
  metric("residual_unchanged", (a.res - b.res).norm(), 0);
  metric("FEJ_jacobian_unchanged", (a.H - b.H).norm(), 0);
  auto ctx = UpdaterLTV(plain).context(*state);
  metric("disabled_matches_original_block", (a.R.bottomRightCorner(3, 3) - UpdaterLTV(plain).velocity(*state, ctx, f.snapshot).R).norm(),
         0);
  // Independent dense posterior for the weighted joint measurement.
  auto before = StateHelper::get_full_covariance(state);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(b.H.rows(), before.rows());
  int col = 0;
  for (auto type : b.order) {
    H.middleCols(type->id(), type->size()) = b.H.middleCols(col, type->size());
    col += type->size();
  }
  Eigen::MatrixXd S = H * before * H.transpose() + b.R;
  Eigen::MatrixXd K = before * H.transpose() * S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(before.rows(), before.cols()) - K * H;
  Eigen::MatrixXd expected = A * before * A.transpose() + K * b.R * K.transpose();
  ov_type::IMU expected_state;
  expected_state.set_value(state->_imu->value());
  expected_state.update((K * b.res).head(15));
  MeasurementBlock empty;
  empty.H.resize(0, 0);
  empty.R.resize(0, 0);
  require(UpdaterLTV::apply_joint(state, empty, b), "robust measurement submitted");
  metric("weighted_native_state", (state->_imu->value() - expected_state.value()).norm(), 1e-10);
  metric("weighted_Joseph_posterior", (StateHelper::get_full_covariance(state) - expected).norm(), 1e-10);
  require(b.receipt->diagnostics.ekf_calls == 1, "one joint update");
  require(!UpdaterLTV::apply_joint(state, empty, b), "no repeated factor");
  f.snapshot.velocity_body = state->_imu->Rot() * state->_imu->vel() + Eigen::Vector3d(1, 0, 0);
  a = UpdaterLTV(plain).build(state, f);
  b = UpdaterLTV(robust).build(state, f);
  metric("inlier_unmodified", (a.R - b.R).norm(), 0);
  f.snapshot.velocity_body += Eigen::Vector3d(100, 0, 0);
  robust.enable_nis_gate = true;
  b = UpdaterLTV(robust).build(state, f);
  require(b.receipt->diagnostics.velocity_reason == "nis", "Huber cannot bypass raw NIS rejection");
  robust.enable_nis_gate = false;
  robust.enable_quality_gate = true;
  b = UpdaterLTV(robust).build(state, f);
  require(b.receipt->diagnostics.velocity_reason == "quality", "Huber cannot bypass quality rejection");
  auto gravity_state = make_state();
  gravity_state->_timestamp = 1;
  auto gravity_context = UpdaterLTV(plain).context(*gravity_state);
  const Eigen::Vector3d direction = gravity_context.R_current * gravity_context.gamma;
  f.snapshot.gravity_body = 9.81 * (std::cos(.45) * direction + std::sin(.45) * gravity_context.T.col(0));
  auto gravity_options = plain;
  gravity_options.enable_huber = true;
  gravity_options.enable_velocity = false;
  auto gravity_block = UpdaterLTV(gravity_options).build(gravity_state, f);
  require(gravity_block.res.size() == 2, "large-angle G remains independently eligible");
  const double sigma_g = gravity_options.sigma_gravity_deg * std::acos(-1.0) / 180;
  const double expected_variance = sigma_g * std::sin(.45) / gravity_options.huber_delta;
  metric("G_Huber_whitening_in_radians", std::abs(gravity_block.R(0, 0) - expected_variance), 1e-14);
  auto invalid = robust;
  invalid.huber_delta = 0;
  bool rejected = false;
  try {
    invalid.validate(StateOptions());
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "invalid delta rejected");
}
