#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ltv {
// Forward directional derivative of the ACTUAL Euler observer, including its
// internal Riccati state. P is a gain metric, never an error covariance here.
struct ObserverTangent {
  Eigen::VectorXd mean;
  Eigen::MatrixXd riccati;
};
class LtvDiscreteSensitivity {
public:
  static ObserverTangent imu(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &a, const Eigen::MatrixXd &b,
                             const Eigen::Vector3d &acc, double h, const ObserverTangent &d, const Eigen::MatrixXd &da,
                             const Eigen::Vector3d &dacc, double dh = 0, const Eigen::MatrixXd &dnoise = Eigen::MatrixXd()) {
    ObserverTangent out;
    out.mean = d.mean + h * (a * d.mean + da * x + b * dacc) + dh * (a * x + b * acc);
    out.riccati = d.riccati + h * (a * d.riccati + d.riccati * a.transpose() + da * p + p * da.transpose());
    if (dnoise.size())
      out.riccati += h * dnoise;
    // dh derivative of process noise is supplied separately by callers, since
    // the gain process metric is not a sensor-noise covariance.
    out.riccati += dh * (a * p + p * a.transpose());
    return out;
  }
  static ObserverTangent camera(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &c, const Eigen::VectorXd &y,
                                double hq, const ObserverTangent &d, const Eigen::MatrixXd &dc, const Eigen::VectorXd &dy, double dhq = 0) {
    const Eigen::VectorXd r = y - c * x;
    const Eigen::MatrixXd pc = p * c.transpose();
    const Eigen::MatrixXd dpc = d.riccati * c.transpose() + p * dc.transpose();
    ObserverTangent out;
    out.mean = d.mean + hq * (dpc * r + pc * (dy - dc * x - c * d.mean)) + dhq * pc * r;
    out.riccati = d.riccati - hq * (dpc * c * p + pc * dc * p + pc * c * d.riccati) - dhq * pc * c * p;
    return out;
  }
  // Frechet derivative of symmetrize -> eigenvalue floor -> symmetrize.
  // At an eigenvalue exactly on the floor the map is not differentiable. A
  // directional limit must be checked; callers may not claim a local Jacobian.
  static Eigen::MatrixXd spectralFloor(const Eigen::MatrixXd &p, const Eigen::MatrixXd &dp, double floor, bool *boundary = nullptr) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es((.5 * (p + p.transpose())).eval());
    if (es.info() != Eigen::Success)
      throw std::runtime_error("sensitivity spectral solve failed");
    const auto &v = es.eigenvalues();
    const auto &u = es.eigenvectors();
    Eigen::MatrixXd divided(v.size(), v.size());
    bool hit = false;
    for (int i = 0; i < v.size(); ++i) {
      hit = hit || std::abs(v[i] - floor) <= 1e-10 * std::max(1., std::abs(v[i]));
      for (int j = 0; j < v.size(); ++j) {
        const double gap = v[i] - v[j];
        if (std::abs(gap) > 1e-12 * std::max(1., std::max(std::abs(v[i]), std::abs(v[j]))))
          divided(i, j) = (std::max(v[i], floor) - std::max(v[j], floor)) / gap;
        else
          divided(i, j) = v[i] > floor ? 1. : 0.;
      }
    }
    if (boundary)
      *boundary = hit;
    return u * (divided.array() * (u.transpose() * (.5 * (dp + dp.transpose())) * u).array()).matrix() * u.transpose();
  }
  static Eigen::Matrix3d normalizedBearing(const Eigen::Vector3d &raw) {
    const double norm = raw.norm();
    if (!(norm > 1e-12))
      throw std::invalid_argument("invalid bearing sensitivity");
    const Eigen::Vector3d unit = raw / norm;
    return (Eigen::Matrix3d::Identity() - unit * unit.transpose()) / norm;
  }
  static Eigen::Matrix3d projection(const Eigen::Vector3d &unit, const Eigen::Vector3d &dunit) {
    return -dunit * unit.transpose() - unit * dunit.transpose();
  }
};
} // namespace ltv
