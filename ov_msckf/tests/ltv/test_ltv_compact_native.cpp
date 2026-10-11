#include "ltv/runtime/LtvLandmarkHistory.h"
#include "native_test_support.h"
#include "update/UpdaterMSCKF.h"
#include <cassert>
class Host : public UpdaterMSCKF {
public:
  Host(UpdaterOptions &u, FeatureInitializerOptions &f, const LtvOptions &o) : UpdaterMSCKF(u, f, o) {}
  using UpdaterMSCKF::update_joint;
};
int main() {
  // Landmark derivatives at both current and FEJ charts, using native JPL retraction.
  std::mt19937 rng(18);
  std::normal_distribution<double> normal;
  auto vec = [&]() { return Eigen::Vector3d(normal(rng), normal(rng), normal(rng)); };
  double max_error = 0;
  for (int sample = 0; sample < 100; ++sample) {
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
    const Eigen::Vector3d point = vec();
    for (bool fej : {false, true}) {
      const auto H = UpdaterLTV::landmark_jacobian(t, a, point, fej);
      for (int j = 0; j < 12; ++j) {
        ov_type::PoseJPL tp, tm, ap, am;
        tp.set_value(fej ? ft : xt);
        tm.set_value(fej ? ft : xt);
        ap.set_value(fej ? fa : xa);
        am.set_value(fej ? fa : xa);
        Eigen::VectorXd dx = Eigen::VectorXd::Zero(6);
        dx(j % 6) = 1e-6;
        if (j < 6) {
          tp.update(dx);
          tm.update(-dx);
        } else {
          ap.update(dx);
          am.update(-dx);
        }
        const Eigen::Vector3d numerical =
            (UpdaterLTV::landmark_prediction(tp, ap, point) - UpdaterLTV::landmark_prediction(tm, am, point)) / 2e-6;
        max_error = std::max(max_error, (numerical - H.col(j)).norm());
      }
    }
  }
  metric("landmark_current_and_FEJ_FD", max_error, 1e-7);
  LtvOptions o;
  o.enabled = o.allow_correlated_pseudomeasurements = o.enable_gravity = o.enable_velocity = o.enable_landmark_approx = true;
  o.enable_nis_gate = false;
  UpdaterLTV model(o);
  UpdaterOptions u;
  FeatureInitializerOptions fi;
  auto fixture = []() {
    auto s = make_state();
    auto a = std::dynamic_pointer_cast<ov_type::PoseJPL>(StateHelper::clone(s, s->_imu->pose()));
    s->_clones_IMU.emplace(.5, a);
    s->_timestamp = 1;
    StateHelper::set_initial_covariance(s, .01 * Eigen::MatrixXd::Identity(21, 21), {s->_imu, a});
    return s;
  };
  auto s = fixture(), expected = fixture();
  LtvFrame f;
  f.epoch = f.sequence = 1;
  f.available = f.ready_G = f.ready_V = true;
  f.camera_time = f.imu_time = f.cursor = 1;
  f.snapshot.frame_timestamp = f.snapshot.imu_timestamp = 1;
  f.snapshot.valid = f.snapshot.gravity_valid = f.snapshot.velocity_valid = true;
  f.snapshot.gravity_body = 9.81 * s->_imu->Rot() * Eigen::Vector3d(0, 0, -1);
  f.snapshot.velocity_body = s->_imu->Rot() * s->_imu->vel() + Eigen::Vector3d(.01, -.02, .01);
  f.landmarks.push_back({1, 7, 0, 0, Eigen::Vector3d(.01, 0, 5), Eigen::Vector3d(0, 0, 5), .5, .5});
  auto visual = [](const std::shared_ptr<State> &state) {
    MeasurementBlock v;
    v.order = {state->_clones_IMU.at(.5)};
    v.H = Eigen::MatrixXd::Identity(6, 6);
    v.R = .1 * Eigen::MatrixXd::Identity(6, 6);
    v.res = Eigen::VectorXd::Constant(6, .001);
    return v;
  };
  const auto aux = model.build(s, f);
  require(aux.receipt->diagnostics.landmark_rows == 3, "L constructed from value-only candidate");
  auto full = merge_measurements(visual(expected), model.build(expected, f));
  require(full.H.cols() == 15 && full.H.rows() == 14, "visual/G/V/L mixed type columns");
  StateHelper::EKFUpdate(expected, full.order, full.H, full.res, full.R);
  Host host(u, fi, o);
  LtvDiagnostics d;
  host.update_joint(s, visual(s), &f, &d);
  require(d.ekf_calls == 1 && d.gravity_rows == 2 && d.velocity_rows == 3 && d.landmark_rows == 3, "one native host update");
  metric("host_mean", (s->_imu->value() - expected->_imu->value()).norm(), 1e-12);
  metric("host_P", (StateHelper::get_full_covariance(s) - StateHelper::get_full_covariance(expected)).norm(), 1e-12);
  const Eigen::VectorXd saved = s->_imu->value();
  host.update_joint(s, MeasurementBlock(), &f, &d);
  require(d.ekf_calls == 0 && d.submit_reason == "duplicate_or_stale_token" && (s->_imu->value() - saved).norm() == 0,
          "duplicate auxiliary not resubmitted");
  auto empty_s = fixture();
  Host empty_host(u, fi, o);
  std::vector<std::shared_ptr<Feature>> no_features;
  empty_host.update(empty_s, no_features, &f, &d);
  require(d.ekf_calls == 1 && d.visual_rows == 0 && d.landmark_rows == 3, "actual empty-visual entry updates auxiliary");
  auto reject_s = fixture(), reject_expected = fixture();
  Host reject_host(u, fi, o);
  auto bad = f;
  bad.camera_time = 2;
  auto v = visual(reject_expected);
  StateHelper::EKFUpdate(reject_expected, v.order, v.H, v.res, v.R);
  reject_host.update_joint(reject_s, visual(reject_s), &bad, &d);
  require(d.ekf_calls == 1 && d.gravity_rows == 0 && d.velocity_rows == 0 && d.landmark_rows == 0, "rejected auxiliary retains visual");
  require(d.landmark_reason == "time_or_token", "enabled L records its rejection reason");
  metric("visual_fallback", (reject_s->_imu->value() - reject_expected->_imu->value()).norm(), 1e-12);
  // OFF bypasses all auxiliary models even if a frame is accidentally supplied.
  auto off = fixture(), off_expected = fixture();
  Host off_host(u, fi, LtvOptions());
  v = visual(off_expected);
  StateHelper::EKFUpdate(off_expected, v.order, v.H, v.res, v.R);
  off_host.update_joint(off, visual(off), &f, &d);
  metric("OFF_native_mean", (off->_imu->value() - off_expected->_imu->value()).norm(), 1e-12);
  metric("OFF_native_P", (StateHelper::get_full_covariance(off) - StateHelper::get_full_covariance(off_expected)).norm(), 1e-12);
  require(LtvLandmarkHistory::b3_gate(true, true, .5, Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(0, 0, 5)), "past L gate");
  std::cout << "PASS native models, mixed columns, one EKF, empty visual, rejected auxiliary, duplicate and OFF\n";
}
