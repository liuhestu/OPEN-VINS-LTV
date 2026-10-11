#include "ltv/fusion/UpdaterLTV.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace ov_msckf;
struct Input {
  int n, m, c;
  Eigen::VectorXd imu, fej, residual;
  Eigen::MatrixXd P, H, R;
  std::vector<int> ids, sizes;
};
static Eigen::MatrixXd read_matrix(std::istream &s, int n, int m) {
  Eigen::MatrixXd x(n, m);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < m; j++)
      s >> x(i, j);
  return x;
}
static std::shared_ptr<State> fixture(const Input &in) {
  StateOptions options;
  auto s = std::make_shared<State>(options);
  s->_timestamp = 1;
  s->_imu->set_value(in.imu);
  s->_imu->set_fej(in.fej);
  std::vector<std::shared_ptr<ov_type::Type>> order = {s->_imu};
  for (int i = 15; i < in.n; i += 6) {
    auto a = std::dynamic_pointer_cast<ov_type::PoseJPL>(StateHelper::clone(s, s->_imu->pose()));
    s->_clones_IMU.emplace(i, a);
    order.push_back(a);
  }
  StateHelper::set_initial_covariance(s, in.P, order);
  return s;
}
static MeasurementBlock visual(const Input &in, const std::shared_ptr<State> &s) {
  MeasurementBlock b;
  b.H = in.H;
  b.R = in.R;
  b.res = in.residual;
  for (size_t i = 0; i < in.ids.size(); i++) {
    bool found = false;
    for (const auto &a : s->_clones_IMU)
      if (a.second->id() == in.ids[i] && a.second->size() == in.sizes[i]) {
        b.order.push_back(a.second);
        found = true;
        break;
      }
    if (!found)
      throw std::runtime_error("visual clone metadata cannot be mapped");
  }
  return b;
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      return 2;
    std::ifstream f(argv[1]);
    int cases;
    f >> cases;
    std::cout << std::setprecision(17);
    for (int sample = 0; sample < cases; sample++) {
      Input in;
      f >> in.n >> in.m >> in.c;
      int count;
      f >> count;
      for (int k = 0; k < count; k++) {
        int id, size;
        f >> id >> size;
        in.ids.push_back(id);
        in.sizes.push_back(size);
      }
      in.imu = read_matrix(f, 16, 1);
      in.fej = read_matrix(f, 16, 1);
      in.P = read_matrix(f, in.n, in.n);
      in.H = read_matrix(f, in.m, in.c);
      in.R = read_matrix(f, in.m, in.m);
      in.residual = read_matrix(f, in.m, 1);
      if (!f || !in.P.allFinite())
        throw std::runtime_error("invalid input");
      auto expected = fixture(in);
      auto ve = visual(in, expected);
      StateHelper::EKFUpdate(expected, ve.order, ve.H, ve.res, ve.R);
      for (int mode = 0; mode < 3; mode++) {
        auto actual = fixture(in);
        auto va = visual(in, actual);
        LtvOptions o;
        o.enabled = o.allow_correlated_pseudomeasurements = true;
        UpdaterLTV u(o);
        LtvFrame frame;
        frame.available = true;
        frame.epoch = frame.sequence = 1;
        frame.camera_time = frame.imu_time = frame.cursor = 1;
        frame.snapshot.valid = true;
        frame.snapshot.imu_timestamp = frame.snapshot.frame_timestamp = 1;
        auto aux = u.build(actual, frame);
        if (mode) {
          MeasurementBlock zero;
          zero.order = {actual->_imu->q(), actual->_imu->p(), actual->_clones_IMU.begin()->second};
          zero.H = Eigen::MatrixXd::Zero(3, 12);
          zero.R = Eigen::Matrix3d::Identity();
          zero.res = Eigen::Vector3d::Constant(mode == 2 ? .1 : 0.);
          aux = UpdaterLTV::merge(aux, zero);
          aux.receipt->diagnostics.landmark_rows = 3;
        }
        UpdaterLTV::apply_joint(actual, va, aux);
        double mean = (actual->_imu->value() - expected->_imu->value()).cwiseAbs().maxCoeff();
        double cov = (StateHelper::get_full_covariance(actual) - StateHelper::get_full_covariance(expected)).cwiseAbs().maxCoeff();
        double clones = 0;
        for (const auto &a : actual->_clones_IMU)
          clones = std::max(clones, (a.second->value() - expected->_clones_IMU.at(a.first)->value()).cwiseAbs().maxCoeff());
        std::cout << "case=" << sample << " mode=" << mode << " mean_max_abs=" << mean << " P_max_abs=" << cov
                  << " clones_max_abs=" << clones << " LM_component=" << aux.receipt->diagnostics.landmark_update_norm << '\n';
        if (!std::isfinite(mean) || !std::isfinite(cov) || mean > 1e-10 || cov > 1e-10 || clones > 1e-10)
          throw std::runtime_error("zero-information native joint path changed state beyond numerical tolerance");
      }
    }
    std::cout << "PASS real-matrix native visual-only / empty receipt / zeroH-zeroRres / zeroH-nonzerores controls\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
