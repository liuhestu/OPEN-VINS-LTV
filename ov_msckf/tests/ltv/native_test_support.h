#pragma once
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterLTV.h"
#include <iomanip>
#include <iostream>
#include <random>

using namespace ov_msckf;
using namespace ov_core;
inline void require(bool ok, const char *message) {
  if (!ok) {
    std::cerr << "FAIL " << message << std::endl;
    std::exit(1);
  }
}
inline void metric(const char *name, double value, double limit) {
  std::cout << std::setprecision(17) << name << "=" << value << " limit=" << limit << std::endl;
  require(std::isfinite(value) && value <= limit, name);
}
class TestPropagator : public Propagator {
public:
  TestPropagator() : Propagator(NoiseManager(), 9.81) {}
  using Propagator::predict_and_compute;
};
inline Eigen::Matrix<double, 16, 1> initial_value() {
  Eigen::Matrix<double, 16, 1> x;
  x << rot_2_quat(exp_so3(Eigen::Vector3d(0.3, -0.2, 0.15))), 0.4, -0.3, 0.2, 0.6, 0.2, -0.1, 0.01, -0.02, 0.03, 0.1, -0.05, 0.02;
  return x;
}
inline std::shared_ptr<State> make_state(StateOptions options = StateOptions()) {
  auto s = std::make_shared<State>(options);
  s->_imu->set_value(initial_value());
  s->_imu->set_fej(initial_value());
  s->_timestamp = 0;
  return s;
}
inline Eigen::MatrixXd gv_h(const ov_type::IMU &imu, const Eigen::Vector3d &gamma, const Eigen::Matrix<double, 3, 2> &T, bool fej) {
  StateOptions options;
  options.do_fej = fej;
  auto state = std::make_shared<State>(options);
  state->_imu->set_value(imu.value());
  state->_imu->set_fej(imu.fej());
  LtvOptions settings;
  UpdaterLTV updater(settings);
  auto context = updater.context(*state);
  context.T = T;
  context.gamma = gamma;
  ltv::LtvSnapshot snapshot;
  snapshot.gravity_body = Eigen::Vector3d(0, 0, -9.81);
  snapshot.velocity_body.setZero();
  auto gravity = updater.gravity(*state, context, snapshot), velocity = updater.velocity(*state, context, snapshot);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(5, 15);
  H.block<3, 3>(0, 0) = velocity.H.leftCols(3);
  H.block<3, 3>(0, 6) = velocity.H.rightCols(3);
  H.block<2, 3>(3, 0) = gravity.H.leftCols(3);
  return H;
}
inline Eigen::VectorXd gv_prediction(const ov_type::IMU &imu, const Eigen::Vector3d &gamma, const Eigen::Matrix<double, 3, 2> &T) {
  Eigen::VectorXd z(5);
  z << imu.Rot() * imu.vel(), T.transpose() * imu.Rot() * gamma;
  return z;
}
inline Eigen::MatrixXd gauge(const ov_type::IMU &imu) {
  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(15, 4);
  const Eigen::Vector3d gamma(0, 0, -1);
  N.block<3, 3>(3, 0).setIdentity();
  N.block<3, 1>(0, 3) = imu.Rot_fej() * gamma;
  N.block<3, 1>(3, 3) = -skew_x(imu.pos_fej()) * gamma;
  N.block<3, 1>(6, 3) = -skew_x(imu.vel_fej()) * gamma;
  return N;
}
