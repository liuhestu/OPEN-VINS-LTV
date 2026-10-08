#include "ltv/fusion/UpdaterLandmarkApprox.h"
#include "native_test_support.h"
int main() {
  std::mt19937 rng(20261008);
  std::uniform_real_distribution<double> dist(-1, 1);
  auto vec = [&]() { return Eigen::Vector3d(dist(rng), dist(rng), dist(rng)); };
  const double eps[] = {1e-5, 1e-6, 1e-7};
  double prediction_error[3] = {0, 0, 0}, residual_error[3] = {0, 0, 0};
  for (int sample = 0; sample < 300; ++sample) {
    ov_type::PoseJPL t, a;
    Eigen::Matrix<double, 7, 1> xt, xa, ft, fa;
    xt << rot_2_quat(exp_so3(vec())), vec();
    xa << rot_2_quat(exp_so3(vec())), vec();
    ft << rot_2_quat(exp_so3(vec())), vec();
    fa << rot_2_quat(exp_so3(vec())), vec();
    t.set_value(xt);
    t.set_fej(ft);
    a.set_value(xa);
    a.set_fej(fa);
    const Eigen::Vector3d la = 5 * vec();
    for (bool fej : {false, true}) {
      const auto H = UpdaterLandmarkApprox::jacobian(t, a, la, fej);
      metric("translation_gauge", (H.block<3, 3>(0, 3) + H.block<3, 3>(0, 9)).norm(), 1e-12);
      for (int e = 0; e < 3; ++e)
        for (int j = 0; j < 12; ++j) {
          ov_type::PoseJPL tp, tm, ap, am;
          tp.set_value(fej ? ft : xt);
          tm.set_value(fej ? ft : xt);
          ap.set_value(fej ? fa : xa);
          am.set_value(fej ? fa : xa);
          Eigen::VectorXd dx = Eigen::VectorXd::Zero(6);
          dx(j % 6) = eps[e];
          if (j < 6) {
            tp.update(dx);
            tm.update(-dx);
          } else {
            ap.update(dx);
            am.update(-dx);
          }
          const Eigen::Vector3d hp = UpdaterLandmarkApprox::prediction(tp, ap, la);
          const Eigen::Vector3d hm = UpdaterLandmarkApprox::prediction(tm, am, la);
          const Eigen::Vector3d dh = (hp - hm) / (2 * eps[e]);
          const Eigen::Vector3d measurement = vec();
          const Eigen::Vector3d dr = ((measurement - hp) - (measurement - hm)) / (2 * eps[e]);
          prediction_error[e] = std::max(prediction_error[e], (dh - H.col(j)).cwiseAbs().maxCoeff());
          residual_error[e] = std::max(residual_error[e], (dr + H.col(j)).cwiseAbs().maxCoeff());
        }
    }
    const Eigen::Matrix3d Q = exp_so3(vec());
    const Eigen::Vector3d shift = vec();
    ov_type::PoseJPL tg, ag;
    Eigen::Matrix<double, 7, 1> yt, ya;
    yt << rot_2_quat(t.Rot() * Q.transpose()), Q * t.pos() + shift;
    ya << rot_2_quat(a.Rot() * Q.transpose()), Q * a.pos() + shift;
    tg.set_value(yt);
    ag.set_value(ya);
    metric("world_SE3_gauge", (UpdaterLandmarkApprox::prediction(tg, ag, la) - UpdaterLandmarkApprox::prediction(t, a, la)).norm(), 1e-12);
  }
  for (int e = 0; e < 3; ++e) {
    std::cout << "epsilon=" << eps[e] << '\n';
    metric("native_landmark_prediction_FD", prediction_error[e], 1e-6);
    metric("native_landmark_residual_FD", residual_error[e], 1e-6);
  }
  const Eigen::Vector3d point(0, 0, 5), near = point + Eigen::Vector3d(.09, 0, 0);
  require(UpdaterLandmarkApprox::b3_gate(true, true, .55 + 3e-7, near, point), "source-derived live age and agreement");
  require(!UpdaterLandmarkApprox::b3_gate(true, true, .6, near, point), "long age rejected");
  require(!UpdaterLandmarkApprox::b3_gate(false, true, .5, near, point), "immaturity rejected");
  require(!UpdaterLandmarkApprox::b3_gate(true, false, .5, near, point), "past readiness rejected");
  require(!UpdaterLandmarkApprox::b3_gate(true, true, .5, point + Eigen::Vector3d(.2, 0, 0), point), "model disagreement rejected");
  // Native visual rows own whole clone types. Split anchor q/p columns overlap
  // them and are rejected by merge; this integration failure is not a noise gate.
  auto state = make_state();
  auto anchor = std::dynamic_pointer_cast<ov_type::PoseJPL>(StateHelper::clone(state, state->_imu->pose()));
  state->_clones_IMU.emplace(0, anchor);
  state->_timestamp = 1;
  MeasurementBlock visual;
  visual.order = {anchor};
  visual.H = Eigen::MatrixXd::Zero(3, 6);
  visual.H.leftCols(3).setIdentity();
  visual.R = Eigen::Matrix3d::Identity();
  visual.res = Eigen::Vector3d::Constant(.001);
  MeasurementBlock old_point;
  old_point.order = {state->_imu->q(), state->_imu->p(), anchor->q(), anchor->p()};
  old_point.H = UpdaterLandmarkApprox::jacobian(*state->_imu->pose(), *anchor, point, true);
  old_point.R = Eigen::Matrix3d::Identity();
  old_point.res = Eigen::Vector3d::Constant(.01);
  bool overlap_rejected = false;
  try {
    UpdaterLTV::merge(visual, old_point);
  } catch (const std::invalid_argument &) {
    overlap_rejected = true;
  }
  require(overlap_rejected, "old split-type visual overlap counterexample");
  LtvOptions enabled;
  enabled.enabled = enabled.allow_correlated_pseudomeasurements = true;
  LtvFrame frame;
  frame.available = true;
  frame.epoch = frame.sequence = 1;
  frame.camera_time = frame.imu_time = frame.cursor = 1;
  frame.snapshot.valid = true;
  frame.snapshot.imu_timestamp = frame.snapshot.frame_timestamp = 1;
  UpdaterLTV generic(enabled);
  auto base = generic.build(state, frame);
  MeasurementBlock point_block = old_point;
  point_block.order = {state->_imu->pose(), anchor};
  auto auxiliary = UpdaterLTV::merge(base, point_block);
  auxiliary.receipt->diagnostics.landmark_rows = 3;
  const auto stacked = UpdaterLTV::merge(visual, auxiliary);
  require(stacked.H.cols() == 12 && stacked.H.rows() == 6, "whole-pose native visual+landmark layout");
  require(UpdaterLTV::apply_joint(state, visual, auxiliary), "native visual+landmark receipt consumed");
  const auto &applied = auxiliary.receipt->diagnostics;
  require(applied.consumed && applied.landmark_rows == 3 && applied.submit_reason == "joint_applied" && applied.ekf_calls == 1,
          "actual whole-pose joint native update");
  require(applied.landmark_update_norm > 0, "actual landmark component in joint native K*r");
  LtvOptions defaults;
  require(!defaults.enable_landmark_approx && !defaults.landmark_approx_shadow, "default OFF");
  std::cout << "PASS native JPL/FEJ landmark model, units, gauge, B3 gate, default OFF\n";
}
