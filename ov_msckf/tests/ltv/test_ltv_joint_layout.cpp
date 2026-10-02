#include "cam/CamRadtan.h"
#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "native_test_support.h"
#include "update/UpdaterHelper.h"
#include "update/UpdaterMSCKF.h"
LtvFrame candidate(const std::shared_ptr<State> &s) {
  LtvFrame f;
  f.available = true;
  f.epoch = 1;
  f.sequence = 1;
  f.version = 1;
  f.camera_time = s->_timestamp;
  f.imu_time = s->_timestamp;
  f.cursor = s->_timestamp;
  f.snapshot.imu_timestamp = f.imu_time;
  f.snapshot.frame_timestamp = f.camera_time;
  f.snapshot.valid = true;
  f.snapshot.velocity_valid = true;
  f.snapshot.gravity_valid = true;
  f.snapshot.observed_features = 30;
  f.snapshot.gravity_body = s->_imu->Rot() * Eigen::Vector3d(0, 0, -9.81);
  f.snapshot.velocity_body = s->_imu->Rot() * s->_imu->vel();
  return f;
}
int main() {
  LtvOptions options;
  options.enabled = true;
  options.enable_gravity = true;
  options.enable_velocity = true;
  options.allow_correlated_pseudomeasurements = true;
  options.validate(StateOptions());
  UpdaterLTV updater(options);
  for (int mask = 0; mask < 4; ++mask) {
    auto s = make_state();
    s->_timestamp = 1;
    StateHelper::augment_clone(s, Eigen::Vector3d::Zero());
    auto f = candidate(s);
    f.snapshot.gravity_valid = mask & 1;
    f.snapshot.velocity_valid = mask & 2;
    auto block = updater.build(s, f);
    require(block.res.size() == ((mask & 1) ? 2 : 0) + ((mask & 2) ? 3 : 0), "independent G/V acceptance");
    const auto before = StateHelper::get_full_covariance(s);
    const auto value = s->_imu->value();
    UpdaterOptions native;
    ov_core::FeatureInitializerOptions fi;
    UpdaterMSCKF msckf(native, fi);
    std::vector<std::shared_ptr<ov_core::Feature>> features;
    msckf.update(s, features, &block);
    require(block.receipt->diagnostics.ekf_calls == (mask ? 1 : 0), "empty visual 0/1 joint calls");
    auto after = StateHelper::get_full_covariance(s);
    require((value - s->_imu->value()).norm() < 1e-12, "zero innovation preserves nominal");
    if (mask)
      require(after.trace() < before.trace(), "zero innovation reduces full P");
    msckf.update(s, features, &block);
    require((StateHelper::get_full_covariance(s) - after).norm() == 0, "receipt never reused");
  }
  auto s = make_state();
  s->_timestamp = 1;
  StateHelper::augment_clone(s, Eigen::Vector3d::Zero());
  auto clone = s->_clones_IMU.at(1);
  auto f = candidate(s);
  auto block = updater.build(s, f);
  auto ctx = updater.context(*s);
  metric("tangent_orthogonal", (ctx.T.transpose() * ctx.R_linearization * ctx.gamma).norm(), 1e-12);
  metric("tangent_orthonormal", (ctx.T.transpose() * ctx.T - Eigen::Matrix2d::Identity()).norm(), 1e-12);
  auto reverse = f;
  reverse.snapshot.gravity_body *= -1;
  auto rejection = updater.build(s, reverse);
  require(rejection.receipt->diagnostics.gravity_rows == 0 && rejection.receipt->diagnostics.velocity_rows == 3,
          "reverse gravity cannot reject V");
  auto scaled = f;
  scaled.snapshot.gravity_body *= 1.01;
  auto scale_block = updater.build(s, scaled);
  metric("gravity_scale_invariance", (scale_block.res.head(2) - block.res.head(2)).norm(), 1e-12);
  auto bad = f;
  bad.imu_time = std::numeric_limits<double>::quiet_NaN();
  require(updater.build(s, bad).empty(), "nonfinite time rejected");
  MeasurementBlock visual;
  visual.order = {clone->p(), s->_imu->v(), clone->q()};
  visual.H = Eigen::MatrixXd::Random(4, 9);
  visual.res = Eigen::VectorXd::Constant(4, 0.01);
  visual.R = 4 * Eigen::Matrix4d::Identity();
  auto merged = UpdaterLTV::merge(visual, block);
  require(merged.order.size() == 4 && merged.order.back() == s->_imu->q(), "preserve visual order append current q");
  Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(9, s->max_covariance_size());
  int col = 0;
  for (const auto &type : visual.order) {
    dense.block(0, type->id(), 4, type->size()) = visual.H.middleCols(col, type->size());
    col += type->size();
  }
  dense.block(4, s->_imu->q()->id(), 5, 3) = block.H.leftCols(3);
  dense.block(4, s->_imu->v()->id(), 5, 3) = block.H.rightCols(3);
  Eigen::MatrixXd scattered = Eigen::MatrixXd::Zero(9, s->max_covariance_size());
  col = 0;
  for (const auto &type : merged.order) {
    scattered.middleCols(type->id(), type->size()) = merged.H.middleCols(col, type->size());
    col += type->size();
  }
  metric("joint_dense_scatter", (dense - scattered).norm(), 1e-14);
  metric("pixel_noise_preserved", (merged.R.topLeftCorner(4, 4) - visual.R).norm(), 0);
  MeasurementBlock overlap;
  overlap.order = {s->_imu};
  overlap.H = Eigen::MatrixXd::Zero(1, 15);
  overlap.res = Eigen::VectorXd::Zero(1);
  overlap.R = Eigen::MatrixXd::Identity(1, 1);
  bool threw = false;
  try {
    UpdaterLTV::merge(overlap, block);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  require(threw, "parent overlap rejected");
  auto malformed = block;
  malformed.H.resize(1, 1);
  std::string reason;
  require(!UpdaterLTV::validate(s, malformed, reason), "dimensions rejected");
  auto invalid_noise = block;
  invalid_noise.R = -invalid_noise.R;
  require(!UpdaterLTV::validate(s, invalid_noise, reason), "negative R rejected");
  auto before = StateHelper::get_full_covariance(s);
  MeasurementBlock empty;
  empty.H.resize(0, 0);
  empty.R.resize(0, 0);
  UpdaterLTV::apply_joint(s, empty, invalid_noise);
  require((StateHelper::get_full_covariance(s) - before).norm() == 0, "invalid R leaves state P unchanged");
  auto large_options = options;
  large_options.sigma_gravity_deg *= 2;
  large_options.sigma_velocity_mps *= 2;
  UpdaterLTV large(large_options);
  auto loud = large.build(s, f);
  metric("sigma_squared_once", (loud.R - 4 * block.R).norm(), 1e-12);
  auto stale = updater.build(s, f);
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(15);
  dx(3) = 0.01;
  s->_imu->update(dx);
  UpdaterLTV::apply_joint(s, empty, stale);
  require(stale.receipt->diagnostics.ekf_calls == 0 && stale.receipt->diagnostics.submit_reason == "stale_prior",
          "stale context rejected before write");
  // A feature removed during the real MSCKF cleaning stage reaches ct_meas<1.
  auto fresh = updater.build(s, candidate(s));
  UpdaterOptions no;
  ov_core::FeatureInitializerOptions fi;
  UpdaterMSCKF msckf(no, fi);
  auto feature = std::make_shared<ov_core::Feature>();
  feature->featid = 999;
  std::vector<std::shared_ptr<ov_core::Feature>> features = {feature};
  msckf.update(s, features, &fresh);
  require(fresh.receipt->diagnostics.ekf_calls == 1, "all visual rejected still submits auxiliary once");
  auto clone_stale = updater.build(s, candidate(s));
  Eigen::VectorXd dc = Eigen::VectorXd::Zero(6);
  dc(3) = 0.01;
  clone->update(dc);
  UpdaterLTV::apply_joint(s, empty, clone_stale);
  require(clone_stale.receipt->diagnostics.ekf_calls == 0, "stale clone context rejected");
  auto invalidSstate = make_state();
  StateHelper::set_initial_covariance(invalidSstate, -Eigen::MatrixXd::Identity(15, 15), {invalidSstate->_imu});
  MeasurementBlock invalidS;
  invalidS.order = {invalidSstate->_imu->q()};
  invalidS.H = Eigen::Matrix3d::Identity();
  invalidS.res = Eigen::Vector3d::Zero();
  invalidS.R = 0.1 * Eigen::Matrix3d::Identity();
  require(!UpdaterLTV::validate(invalidSstate, invalidS, reason) && reason == "nonSPD_S", "nonSPD innovation refused before state write");
  auto current = make_state();
  auto fej = current->_imu->value();
  ov_type::IMU perturbed;
  perturbed.set_value(fej);
  Eigen::VectorXd change = Eigen::VectorXd::Zero(15);
  change(0) = 0.05;
  change(6) = 0.2;
  perturbed.update(change);
  current->_imu->set_fej(perturbed.value());
  auto linearization = updater.context(*current);
  auto snap = candidate(current).snapshot;
  auto g = updater.gravity(*current, linearization, snap), v = updater.velocity(*current, linearization, snap);
  metric("current_residual_with_different_FEJ", g.res.norm() + v.res.norm(), 1e-12);
  require((linearization.R_current - linearization.R_linearization).norm() > 0.01, "distinct FEJ context");
  for (double angle : {0.0, std::acos(-1.0) / 2, 0.7}) {
    auto x = current->_imu->value();
    x.block<4, 1>(0, 0) = rot_2_quat(exp_so3(Eigen::Vector3d(0, 0, angle)));
    current->_imu->set_value(x);
    current->_imu->set_fej(x);
    auto c1 = updater.context(*current);
    auto prediction1 = updater.velocity(*current, c1, snap).res;
    x.block<4, 1>(0, 0) *= -1;
    current->_imu->set_value(x);
    current->_imu->set_fej(x);
    auto c2 = updater.context(*current);
    metric("quaternion_sign_invariance", (updater.velocity(*current, c2, snap).res - prediction1).norm(), 1e-12);
  }
  auto invalidq = current->_imu->value();
  invalidq.block<4, 1>(0, 0).setZero();
  current->_imu->set_value(invalidq);
  bool rejected_q = false;
  try {
    updater.build(current, candidate(current));
  } catch (const std::runtime_error &) {
    rejected_q = true;
  }
  require(rejected_q, "zero native quaternion is a blocking corruption");
  // Increased sigma must reduce information for the same frozen linear model.
  auto low_noise_state = make_state(), high_noise_state = make_state();
  auto low_noise = updater.build(low_noise_state, candidate(low_noise_state));
  auto high_noise = large.build(high_noise_state, candidate(high_noise_state));
  UpdaterLTV::apply_joint(low_noise_state, empty, low_noise);
  UpdaterLTV::apply_joint(high_noise_state, empty, high_noise);
  const Eigen::MatrixXd information_difference =
      StateHelper::get_full_covariance(high_noise_state) - StateHelper::get_full_covariance(low_noise_state);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> information_eigen(information_difference);
  require(information_eigen.info() == Eigen::Success && information_eigen.eigenvalues().minCoeff() >= -1e-10 &&
              information_difference.trace() > 0,
          "larger sigma yields less information in PSD order");
  // Exercise native compression's empty output and the actual auxiliary submission.
  Eigen::MatrixXd zero_columns(2, 0);
  Eigen::VectorXd zero_residual = Eigen::Vector2d::Zero();
  UpdaterHelper::measurement_compress_inplace(zero_columns, zero_residual);
  require(zero_columns.rows() == 0 && zero_residual.rows() == 0, "native compression empty result");
  auto compressed_state = make_state();
  auto compressed_aux = updater.build(compressed_state, candidate(compressed_state));
  MeasurementBlock compressed;
  compressed.H = zero_columns;
  compressed.res = zero_residual;
  compressed.R.resize(0, 0);
  UpdaterLTV::apply_joint(compressed_state, compressed, compressed_aux);
  require(compressed_aux.receipt->diagnostics.ekf_calls == 1, "empty compressed visual submits one auxiliary update");
  // Valid normalized bearings triangulate; inconsistent pixels must be rejected by chi2.
  auto chi_state = make_state();
  auto camera = std::make_shared<CamRadtan>(640, 480);
  Eigen::VectorXd intrinsics(8);
  intrinsics << 300, 300, 320, 240, 0, 0, 0, 0;
  camera->set_value(intrinsics);
  chi_state->_cam_intrinsics_cameras[0] = camera;
  chi_state->_cam_intrinsics[0]->set_value(intrinsics);
  auto chi_feature = std::make_shared<Feature>();
  chi_feature->featid = 123;
  for (int frame = 0; frame < 3; ++frame) {
    Eigen::Matrix<double, 16, 1> x = Eigen::Matrix<double, 16, 1>::Zero();
    x(3) = 1;
    x(4) = 0.5 * frame;
    chi_state->_imu->set_value(x);
    chi_state->_imu->set_fej(x);
    chi_state->_timestamp = frame;
    StateHelper::augment_clone(chi_state, Eigen::Vector3d::Zero());
    chi_feature->timestamps[0].push_back(frame);
    chi_feature->uvs_norm[0].push_back(Eigen::Vector2f(-0.1 * frame, 0));
    chi_feature->uvs[0].push_back(Eigen::Vector2f(320 - 30 * frame, 240 + (frame == 1 ? 100 : 0)));
  }
  FeatureInitializer initializer(fi);
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> poses;
  for (const auto &entry : chi_state->_clones_IMU)
    poses[0].emplace(entry.first, FeatureInitializer::ClonePose(entry.second->Rot(), entry.second->pos()));
  require(initializer.single_triangulation(chi_feature, poses) && initializer.single_gaussnewton(chi_feature, poses),
          "chi2 fixture passes real triangulation and refinement");
  auto chi_aux = updater.build(chi_state, candidate(chi_state));
  std::vector<std::shared_ptr<Feature>> chi_features = {chi_feature};
  msckf.update(chi_state, chi_features, &chi_aux);
  require(chi_features.empty() && chi_feature->to_delete && chi_aux.receipt->diagnostics.ekf_calls == 1,
          "all visual chi2 rejected still submits auxiliary once");
  std::cout << "T4 joint layout and gates PASS" << std::endl;
}
