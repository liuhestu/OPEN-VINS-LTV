#include "native_test_support.h"
int main() {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-1, 1);
  auto vec = [&]() { return Eigen::Vector3d(dist(rng), dist(rng), dist(rng)); };
  double errors[3] = {0, 0, 0}, residual_errors[3] = {0, 0, 0}, relative_errors[3] = {0, 0, 0};
  const double epsilons[] = {1e-5, 1e-6, 1e-7};
  for (int sample = 0; sample < 500; ++sample) {
    ov_type::IMU imu;
    auto x = initial_value();
    x.head<4>() = rot_2_quat(exp_so3(vec()));
    x.segment<3>(7) = sample % 10 == 0 ? Eigen::Vector3d::Zero() : vec();
    imu.set_value(x);
    auto fej = x;
    fej.head<4>() = rot_2_quat(exp_so3(vec()));
    fej.segment<3>(7) = vec();
    imu.set_fej(fej);
    const Eigen::Vector3d gamma(0, 0, sample % 2 ? 1.0 : -1.0);
    const Eigen::Matrix<double, 3, 2> T = exp_so3(vec()).leftCols<2>();
    const Eigen::VectorXd measurement = gv_prediction(imu, gamma, T) + Eigen::VectorXd::Constant(5, 0.2);
    for (bool use_fej : {false, true}) {
      const auto H = gv_h(imu, gamma, T, use_fej);
      ov_type::IMU base;
      base.set_value(use_fej ? fej : x);
      for (int e = 0; e < 3; ++e)
        for (int j = 0; j < 15; ++j) {
          ov_type::IMU plus, minus;
          plus.set_value(base.value());
          minus.set_value(base.value());
          Eigen::VectorXd dx = Eigen::VectorXd::Zero(15);
          dx(j) = epsilons[e];
          plus.update(dx);
          minus.update(-dx);
          const Eigen::VectorXd hp = gv_prediction(plus, gamma, T), hm = gv_prediction(minus, gamma, T);
          const Eigen::VectorXd dh = (hp - hm) / (2 * epsilons[e]);
          const Eigen::VectorXd dr = ((measurement - hp) - (measurement - hm)) / (2 * epsilons[e]);
          errors[e] = std::max(errors[e], (dh - H.col(j)).cwiseAbs().maxCoeff());
          for (int row = 0; row < H.rows(); ++row)
            if (std::abs(H(row, j)) >= 1e-3)
              relative_errors[e] = std::max(relative_errors[e], std::abs(dh(row) - H(row, j)) / std::abs(H(row, j)));
          residual_errors[e] = std::max(residual_errors[e], (dr + H.col(j)).cwiseAbs().maxCoeff());
        }
    }
    ov_type::JPLQuat q;
    q.set_value(x.head<4>());
    Eigen::Vector3d delta = 1e-7 * vec();
    q.update(delta);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(15);
    dx.head<3>() = delta;
    imu.update(dx);
    require((q.Rot() - imu.Rot()).norm() < 1e-14, "JPLQuat versus IMU retraction");
    require((q.Rot() - (Eigen::Matrix3d::Identity() - skew_x(delta)) * quat_2_Rot(x.head<4>())).norm() < 1e-12, "left retraction sign");
    const Eigen::Matrix3d R_IC = exp_so3(vec()), R_BC = R_IC.transpose();
    const Eigen::Vector3d p_IC = vec(), p_BC = -R_BC * p_IC, p_I = vec();
    require((R_BC * (R_IC * p_I + p_IC) + p_BC - p_I).norm() < 1e-12, "camera lever arm inverse");
  }
  for (int e = 0; e < 3; ++e) {
    std::cout << "eps=" << epsilons[e] << " ";
    metric("prediction_jacobian", errors[e], 1e-6);
    metric("residual_jacobian", residual_errors[e], 1e-6);
    metric("relative_error_nonzero_ge_1e_3", relative_errors[e], 1e-5);
  }
  // Verify actual native calibration and single bias subtraction by stationary propagation.
  for (auto model : {StateOptions::KALIBR, StateOptions::RPNG}) {
    StateOptions options;
    options.imu_model = model;
    options.integration_method = StateOptions::DISCRETE;
    auto s = make_state(options);
    auto x = initial_value();
    x.segment<3>(7).setZero();
    s->_imu->set_value(x);
    s->_imu->set_fej(x);
    Eigen::Matrix<double, 6, 1> params;
    params << 1.1, 0.02, 0.03, 1.2, 0.04, 0.9;
    Eigen::Matrix3d expected;
    if (model == StateOptions::KALIBR)
      expected << 1.1, 0, 0, 0.02, 1.2, 0, 0.03, 0.04, 0.9;
    else
      expected << 1.1, 0.02, 1.2, 0, 0.03, 0.04, 0, 0, 0.9;
    metric("Dm_layout", (State::Dm(model, params) - expected).norm(), 1e-14);
    s->_calib_imu_da->set_value(params);
    s->_calib_imu_dw->set_value(params);
    Eigen::VectorXd tg(9);
    tg << 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009;
    s->_calib_imu_tg->set_value(tg);
    Eigen::Matrix3d Tg;
    Tg << 0.001, 0.004, 0.007, 0.002, 0.005, 0.008, 0.003, 0.006, 0.009;
    metric("Tg_layout", (State::Tg(tg) - Tg).norm(), 1e-14);
    s->_calib_imu_ACCtoIMU->set_value(rot_2_quat(exp_so3(Eigen::Vector3d(0.1, 0.2, -0.1))));
    const Eigen::Vector3d corrected = s->_imu->Rot() * Eigen::Vector3d(0, 0, 9.81);
    ImuData a, b;
    a.timestamp = 0;
    b.timestamp = 0.01;
    a.am = expected.inverse() * s->_calib_imu_ACCtoIMU->Rot().transpose() * corrected + s->_imu->bias_a();
    a.wm = s->_imu->bias_g() + Tg * corrected;
    b.am = a.am;
    b.wm = a.wm;
    TestPropagator prop;
    Eigen::MatrixXd F, Q;
    prop.predict_and_compute(s, a, b, F, Q);
    metric("stationary_calibrated_mean", (s->_imu->value() - x).norm(), 1e-12);
  }
}
