#include "ltv/diagnostics/joint/LtvDiscreteSensitivity.h"
#include "ltv/diagnostics/joint/LtvMainCrossShadow.h"
#include "utils/quat_ops.h"
#include <cassert>
#include <iostream>
using namespace ltv;
Eigen::MatrixXd floorP(const Eigen::MatrixXd &p, double f) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> e((.5 * (p + p.transpose())).eval());
  return e.eigenvectors() * e.eigenvalues().cwiseMax(f).asDiagonal() * e.eigenvectors().transpose();
}
int main() {
  const int n = 9, m = 3;
  const double eps = 1e-6, hq = .017;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(n, -.4, 1.1), y = Eigen::VectorXd::LinSpaced(m, .1, .7);
  Eigen::MatrixXd p = Eigen::MatrixXd::Identity(n, n) * 2.;
  p(0, 4) = p(4, 0) = .2;
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(m, n);
  c.leftCols(3) = Eigen::Matrix3d::Identity();
  double worst = 0;
  for (int k = 0; k < n + n * n + m * n + m + 1; ++k) {
    ObserverTangent d{Eigen::VectorXd::Zero(n), Eigen::MatrixXd::Zero(n, n)};
    Eigen::MatrixXd dc = Eigen::MatrixXd::Zero(m, n);
    Eigen::VectorXd dy = Eigen::VectorXd::Zero(m);
    double dq = 0;
    if (k < n)
      d.mean[k] = 1;
    else if (k < n + n * n) {
      int i = k - n;
      d.riccati(i % n, i / n) = 1;
    } else if (k < n + n * n + m * n) {
      int i = k - n - n * n;
      dc(i % m, i / m) = 1;
    } else if (k < n + n * n + m * n + m)
      dy[k - n - n * n - m * n] = 1;
    else
      dq = 1;
    auto f = [&](double t) {
      ObserverTangent out;
      const auto xx = (x + t * d.mean).eval();
      const auto pp = (p + t * d.riccati).eval();
      const auto cc = (c + t * dc).eval();
      const auto yy = (y + t * dy).eval();
      const double q = hq + t * dq;
      out.mean = xx + q * pp * cc.transpose() * (yy - cc * xx);
      out.riccati = pp - q * pp * cc.transpose() * cc * pp;
      return out;
    };
    auto a = LtvDiscreteSensitivity::camera(x, p, c, y, hq, d, dc, dy, dq), plus = f(eps), minus = f(-eps);
    worst = std::max(worst, ((plus.mean - minus.mean) / (2 * eps) - a.mean).norm());
    worst = std::max(worst, ((plus.riccati - minus.riccati) / (2 * eps) - a.riccati).norm());
  }
  assert(worst < 1e-8);
  Eigen::MatrixXd sp = Eigen::Vector3d(-.1, .7, 2.).asDiagonal();
  Eigen::MatrixXd dp = Eigen::Matrix3d::Ones();
  dp(1, 2) = dp(2, 1) = .4;
  bool boundary = false;
  auto ds = LtvDiscreteSensitivity::spectralFloor(sp, dp, .2, &boundary);
  assert(!boundary);
  assert(((floorP(sp + eps * dp, .2) - floorP(sp - eps * dp, .2)) / (2 * eps) - ds).norm() < 1e-8);
  sp(0, 0) = .2;
  LtvDiscreteSensitivity::spectralFloor(sp, dp, .2, &boundary);
  assert(boundary);
  Eigen::Vector3d raw(.2, -.4, 2.);
  auto j = LtvDiscreteSensitivity::normalizedBearing(raw);
  for (int k = 0; k < 3; ++k) {
    auto plus = raw, minus = raw;
    plus[k] += eps;
    minus[k] -= eps;
    assert(((plus.normalized() - minus.normalized()) / (2 * eps) - j.col(k)).norm() < 1e-8);
  }
  const Eigen::Vector3d correction(.4, -.2, .3);
  const auto reset = LtvMainCrossShadow::nativeJplReset(correction);
  auto chart = [&](const Eigen::Vector3d &residual) {
    Eigen::Vector4d q, p;
    q << .5 * (correction + residual), 1.;
    p << -.5 * correction, 1.;
    const auto z = ov_core::quat_multiply(q.normalized(), p.normalized());
    return (2 * z.head<3>() / z[3]).eval();
  };
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    r[k] = eps;
    assert(((chart(r) - chart(-r)) / (2 * eps) - reset.col(k)).norm() < 1e-8);
  }
  const auto ad = LtvMainCrossShadow::nativeJplProgramAd(correction);
  auto injection = [&](const Eigen::Vector3d &old, const Eigen::Vector3d &delta) {
    Eigen::Vector4d q, p, inverse;
    q << .5 * (correction + delta), 1.;
    p << .5 * old, 1.;
    inverse << -.5 * correction, 1.;
    const auto z = ov_core::quat_multiply(ov_core::quat_multiply(q.normalized(), p.normalized()), inverse.normalized());
    return (2 * z.head<3>() / z[3]).eval();
  };
  const auto true_map = LtvMainCrossShadow::nativeTrueErrorMap(correction);
  auto true_error = [&](const Eigen::Vector3d &estimate_delta) {
    Eigen::Vector4d truth, inverse;
    truth << .5 * correction, 1.;
    inverse << -.5 * estimate_delta, 1.;
    const auto z = ov_core::quat_multiply(truth.normalized(), inverse.normalized());
    return (2 * z.head<3>() / z[3]).eval();
  };
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    r[k] = eps;
    assert(((injection(r, Eigen::Vector3d::Zero()) - injection(-r, Eigen::Vector3d::Zero())) / (2 * eps) - ad.col(k)).norm() < 1e-8);
    assert(((injection(Eigen::Vector3d::Zero(), r) - injection(Eigen::Vector3d::Zero(), -r)) / (2 * eps) - reset.col(k)).norm() < 1e-8);
    assert(((true_error(r) - true_error(-r)) / (2 * eps) + true_map.col(k)).norm() < 1e-8);
  }
  std::cout << "native_program_injection_two_inputs=PASS nominal_nonzero_true_error_map=PASS\n";
  std::cout << "native_jpl_rational_reset=PASS additive_reset=identity\n";
  std::cout << "full_camera_all_inputs_max_error=" << worst
            << " spectral_fixed_branch=PASS spectral_boundary=NONSMOOTH normalized_raw=PASS\n";
}
