#include "native_test_support.h"
int main() {
  std::mt19937 generator(42);
  std::normal_distribution<double> normal;
  auto random = [&](int r, int c) {
    Eigen::MatrixXd a(r, c);
    for (int i = 0; i < r; ++i)
      for (int j = 0; j < c; ++j)
        a(i, j) = normal(generator);
    return a;
  };
  double max_p = 0, max_mean = 0, max_sequential = 0;
  for (int sample = 0; sample < 50; ++sample) {
    auto state = make_state();
    state->_timestamp = 1;
    Eigen::MatrixXd A = random(15, 15), P = A * A.transpose() + Eigen::MatrixXd::Identity(15, 15);
    StateHelper::set_initial_covariance(state, P, {state->_imu});
    StateHelper::augment_clone(state, Eigen::Vector3d::Zero());
    auto clone = state->_clones_IMU.at(1);
    P = StateHelper::get_full_covariance(state);
    LtvOptions options;
    options.enabled = true;
    options.enable_gravity = true;
    options.enable_velocity = true;
    options.allow_correlated_pseudomeasurements = true;
    UpdaterLTV updater(options);
    LtvFrame frame;
    frame.available = true;
    frame.epoch = 1;
    frame.sequence = 1;
    frame.version = 1;
    frame.camera_time = frame.imu_time = frame.cursor = 1;
    frame.snapshot.imu_timestamp = frame.imu_time;
    frame.snapshot.frame_timestamp = frame.camera_time;
    frame.ready_G = frame.ready_V = true;
    frame.snapshot.valid = frame.snapshot.gravity_valid = frame.snapshot.velocity_valid = true;
    frame.snapshot.gravity_body = exp_so3(Eigen::Vector3d(0.01, -0.02, 0.005)) * state->_imu->Rot() * Eigen::Vector3d(0, 0, -9.81);
    frame.snapshot.velocity_body = state->_imu->Rot() * state->_imu->vel() + Eigen::Vector3d(0.03, -0.01, 0.02);
    auto auxiliary = updater.build(state, frame);
    require(auxiliary.res.size() == 5, "GV active");
    MeasurementBlock visual;
    visual.order = {clone};
    visual.H = random(4, 6);
    visual.res = 0.01 * random(4, 1);
    visual.R = 2 * Eigen::Matrix4d::Identity();
    auto joint = UpdaterLTV::merge(visual, auxiliary);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 21);
    int offset = 0;
    for (const auto &type : joint.order) {
      H.middleCols(type->id(), type->size()) = joint.H.middleCols(offset, type->size());
      offset += type->size();
    }
    const Eigen::MatrixXd S = H * P * H.transpose() + joint.R, K = P * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(9, 9));
    const Eigen::VectorXd delta = K * joint.res;
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(21, 21);
    const Eigen::MatrixXd joseph = (I - K * H) * P * (I - K * H).transpose() + K * joint.R * K.transpose();
    ov_type::IMU expected_imu;
    expected_imu.set_value(state->_imu->value());
    expected_imu.update(delta.head(15));
    ov_type::PoseJPL expected_clone;
    expected_clone.set_value(clone->value());
    expected_clone.update(delta.tail(6));
    UpdaterLTV::apply_joint(state, visual, auxiliary);
    require(auxiliary.receipt->diagnostics.ekf_calls == 1, "one batch call");
    const auto actual = StateHelper::get_full_covariance(state);
    max_p = std::max(max_p, (actual - joseph).norm() / std::max(1.0, P.norm()));
    max_mean = std::max(max_mean, (state->_imu->value() - expected_imu.value()).norm() + (clone->value() - expected_clone.value()).norm());
    Eigen::MatrixXd sequential = P;
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(21);
    for (const auto &range : std::vector<std::pair<int, int>>{{0, 4}, {4, 5}}) {
      const Eigen::MatrixXd h = H.middleRows(range.first, range.second),
                            r = joint.R.block(range.first, range.first, range.second, range.second);
      const Eigen::MatrixXd si = h * sequential * h.transpose() + r,
                            ki = sequential * h.transpose() * si.ldlt().solve(Eigen::MatrixXd::Identity(range.second, range.second));
      dx += ki * (joint.res.segment(range.first, range.second) - h * dx);
      sequential = sequential - ki * si * ki.transpose();
    }
    max_sequential = std::max(max_sequential, (sequential - joseph).norm() / std::max(1.0, P.norm()) + (dx - delta).norm());
  }
  metric("native_joint_Joseph", max_p, 1e-9);
  metric("native_joint_full_mean", max_mean, 1e-9);
  metric("batch_sequential_fixed_linear", max_sequential, 1e-9);
}
