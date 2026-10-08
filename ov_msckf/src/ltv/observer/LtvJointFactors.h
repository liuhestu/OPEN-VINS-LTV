#pragma once
#include "ltv/observer/LtvDiscreteSensitivity.h"
#include <map>
#include <memory>
#include <string>
namespace ltv {
// Whitened source factors. Every named latent source is a ROW block in the
// joint map, not a repeatedly injected independent draw. Compression keeps
// their rows and therefore every state/source and source/source cross.
class LtvJointFactors {
public:
  int columns() const { return columns_; }
  Eigen::MatrixXd &block(const std::string &name) { return blocks_.at(name); }
  const Eigen::MatrixXd &block(const std::string &name) const { return blocks_.at(name); }
  bool contains(const std::string &name) const { return blocks_.count(name); }
  void set(const std::string &name, const Eigen::MatrixXd &map) {
    if (map.cols() != columns_)
      throw std::invalid_argument("factor columns");
    blocks_[name] = map;
  }
  void zero(const std::string &name, int rows) { blocks_[name] = Eigen::MatrixXd::Zero(rows, columns_); }
  void erase(const std::string &name) { blocks_.erase(name); }
  void initial(const std::string &name, const Eigen::MatrixXd &cov) {
    if (columns_ != 0)
      throw std::invalid_argument("joint factor already initialized");
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> e((.5 * (cov + cov.transpose())).eval());
    if (e.info() != Eigen::Success || e.eigenvalues().minCoeff() < -1e-10 * std::max(1., cov.norm()))
      throw std::invalid_argument("invalid initial law");
    columns_ = cov.rows();
    blocks_[name] = e.eigenvectors() * e.eigenvalues().cwiseMax(0.).cwiseSqrt().asDiagonal();
  }
  void source(const std::string &name, const Eigen::MatrixXd &cov) {
    if (contains(name)) {
      const auto &existing = block(name);
      if (existing.rows() != cov.rows() || cov.rows() != cov.cols() || !cov.allFinite() ||
          (existing * existing.transpose() - cov).norm() > 1e-10 * std::max(1e-12, cov.norm()))
        throw std::invalid_argument("reused source identity has changed dimension/covariance");
      return;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> e((.5 * (cov + cov.transpose())).eval());
    if (e.info() != Eigen::Success || e.eigenvalues().minCoeff() < -1e-12 * std::max(1., cov.norm()))
      throw std::invalid_argument("invalid source law");
    const int old = columns_, count = cov.rows();
    columns_ += count;
    for (auto &entry : blocks_) {
      entry.second.conservativeResize(entry.second.rows(), columns_);
      entry.second.rightCols(count).setZero();
    }
    zero(name, count);
    blocks_[name].rightCols(count) = e.eigenvectors() * e.eigenvalues().cwiseMax(0.).cwiseSqrt().asDiagonal();
  }
  Eigen::MatrixXd covariance(const std::string &a, const std::string &b) const { return block(a) * block(b).transpose(); }
  // Stack ALL retained states, archived anchors and source handles. D'=Q R
  // implies D D'=R' R. Thus replacement D_new=R' preserves the entire joint
  // distribution, including source cross blocks; it is not covariance clipping.
  void compress() {
    int rows = 0;
    for (const auto &entry : blocks_)
      rows += entry.second.rows();
    if (columns_ <= rows || rows == 0)
      return;
    Eigen::MatrixXd joined(rows, columns_);
    int at = 0;
    for (const auto &entry : blocks_) {
      joined.middleRows(at, entry.second.rows()) = entry.second;
      at += entry.second.rows();
    }
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(joined.transpose());
    Eigen::MatrixXd factor = qr.matrixQR().topRows(rows).template triangularView<Eigen::Upper>().transpose();
    at = 0;
    for (auto &entry : blocks_) {
      entry.second = factor.middleRows(at, entry.second.rows());
      at += entry.second.rows();
    }
    columns_ = rows;
  }

private:
  int columns_ = 0;
  std::map<std::string, Eigen::MatrixXd> blocks_;
};
// Attached only to the independent finite-state diagnostic Observer. It is
// invoked from actual native lifecycle/IMU/camera/spectral operations. The
// factors represent variations of program estimates about fixed truth; their
// covariance is not P_R and does not assert the real error mean is zero.
class LtvObserverSourceSensitivity {
public:
  explicit LtvObserverSourceSensitivity(std::shared_ptr<LtvJointFactors> factors) : factors_(std::move(factors)) {}
  void reset(int n) {
    factors_->zero("observer_mean", n);
    factors_->zero("observer_riccati", n * n);
  }
  void imuInput(const Eigen::MatrixXd &directions) { imu_input_ = directions; }
  void cameraInput(const Eigen::Vector3d &raw, const Eigen::MatrixXd &directions, const Eigen::Matrix3d &rbc, const Eigen::Vector3d &pbc) {
    raw_ = raw;
    camera_input_ = directions;
    rbc_ = rbc;
    pbc_ = pbc;
  }
  void seedInput(const Eigen::MatrixXd &directions) { seed_input_ = directions; }
  void lifecycle(const Eigen::MatrixXd &map) {
    const int old = map.cols(), n = map.rows(), s = factors_->columns();
    factors_->set("observer_mean", (map * factors_->block("observer_mean")).eval());
    Eigen::MatrixXd out(n * n, s);
    const auto &prior = factors_->block("observer_riccati");
    for (int k = 0; k < s; ++k) {
      Eigen::Map<const Eigen::MatrixXd> dp(prior.col(k).data(), old, old);
      Eigen::MatrixXd value = map * dp * map.transpose();
      out.col(k) = Eigen::Map<Eigen::VectorXd>(value.data(), n * n);
    }
    factors_->set("observer_riccati", out);
  }
  void seed(int slot, bool apply) {
    if (apply) {
      if (seed_input_.cols() != factors_->columns())
        throw std::invalid_argument("seed source receipt");
      factors_->block("observer_mean").middleRows(3 * slot, 3) = seed_input_;
    }
  }
  void imu(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &a, const Eigen::MatrixXd &b,
           const Eigen::Vector3d &acc, double dt) {
    const int n = x.size(), s = factors_->columns();
    if (imu_input_.rows() != 6 || imu_input_.cols() != s)
      throw std::invalid_argument("IMU source receipt");
    Eigen::MatrixXd mean(n, s), metric(n * n, s);
    for (int k = 0; k < s; ++k) {
      Eigen::Map<const Eigen::MatrixXd> dp(factors_->block("observer_riccati").col(k).data(), n, n);
      Eigen::Vector3d w = imu_input_.col(k).tail<3>();
      Eigen::Matrix3d skew;
      skew << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
      Eigen::MatrixXd da = Eigen::MatrixXd::Zero(n, n);
      for (int i = 0; i < n; i += 3)
        da.block<3, 3>(i, i) = -skew;
      ObserverTangent d{factors_->block("observer_mean").col(k), dp};
      auto out = LtvDiscreteSensitivity::imu(x, p, a, b, acc, dt, d, da, imu_input_.col(k).head<3>());
      mean.col(k) = out.mean;
      metric.col(k) = Eigen::Map<Eigen::VectorXd>(out.riccati.data(), n * n);
    }
    factors_->set("observer_mean", mean);
    factors_->set("observer_riccati", metric);
  }
  void camera(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &c, const Eigen::VectorXd &y, double hq, int slot) {
    const int n = x.size(), s = factors_->columns();
    if (c.rows() != 3 || camera_input_.rows() != 3 || camera_input_.cols() != s)
      throw std::invalid_argument("finite camera receipt");
    Eigen::MatrixXd mean(n, s), metric(n * n, s);
    const Eigen::Vector3d unit = rbc_ * raw_.normalized();
    for (int k = 0; k < s; ++k) {
      Eigen::Map<const Eigen::MatrixXd> dp(factors_->block("observer_riccati").col(k).data(), n, n);
      const Eigen::Vector3d du = rbc_ * LtvDiscreteSensitivity::normalizedBearing(raw_) * camera_input_.col(k);
      const Eigen::Matrix3d dpi = LtvDiscreteSensitivity::projection(unit, du);
      Eigen::MatrixXd dc = Eigen::MatrixXd::Zero(3, n);
      dc.middleCols(3 * slot, 3) = dpi;
      ObserverTangent d{factors_->block("observer_mean").col(k), dp};
      auto out = LtvDiscreteSensitivity::camera(x, p, c, y, hq, d, dc, dpi * pbc_);
      mean.col(k) = out.mean;
      metric.col(k) = Eigen::Map<Eigen::VectorXd>(out.riccati.data(), n * n);
    }
    factors_->set("observer_mean", mean);
    factors_->set("observer_riccati", metric);
  }
  void sanitize(const Eigen::MatrixXd &p, double floor) {
    const int n = p.rows();
    auto &metric = factors_->block("observer_riccati");
    for (int k = 0; k < metric.cols(); ++k) {
      Eigen::Map<const Eigen::MatrixXd> dp(metric.col(k).data(), n, n);
      bool hit = false;
      Eigen::MatrixXd out = LtvDiscreteSensitivity::spectralFloor(p, dp, floor, &hit);
      metric.col(k) = Eigen::Map<Eigen::VectorXd>(out.data(), n * n);
      if (hit)
        ++spectral_boundary_events_;
    }
  }
  unsigned long boundaries() const { return spectral_boundary_events_; }

private:
  std::shared_ptr<LtvJointFactors> factors_;
  Eigen::MatrixXd imu_input_, camera_input_, seed_input_;
  Eigen::Vector3d raw_ = Eigen::Vector3d::UnitZ(), pbc_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rbc_ = Eigen::Matrix3d::Identity();
  unsigned long spectral_boundary_events_ = 0;
};
} // namespace ltv
