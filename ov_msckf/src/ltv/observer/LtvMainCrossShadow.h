#pragma once
#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace ltv {
// Read-only companion of a concrete State instance. Stores physical cross blocks
// C=Cov(delta_x,aux), with delta_x=true-est and aux=est-true. A supplied seed
// Jacobian therefore has C=-P J' + Cov(delta_x,epsilon) J_z'. Never silently
// substitutes zero for unavailable source blocks. Missing blocks are explicit.
class LtvMainCrossShadow {
public:
  static LtvMainCrossShadow &instance() {
    static LtvMainCrossShadow x;
    return x;
  }
  bool enabled() const { return enabled_; }
  void initialize(const void *owner, const Eigen::MatrixXd &p) {
    if (!enabled_)
      return;
    if (owner_ != owner || p_.rows() != p.rows()) {
      owner_ = owner;
      p_ = p;
      stored_ = p;
      cross_ = Eigen::MatrixXd::Zero(p.rows(), 0);
      aux_ = Eigen::MatrixXd(0, 0);
      sources_.clear();
      source_main_ = Eigen::MatrixXd::Zero(p.rows(), 0);
      source_aux_ = Eigen::MatrixXd(0, 0);
      source_variance_.resize(0);
      pending_visual_.resize(0, 0);
      missing_.clear();
      missing_.insert("initial_observer_true_error_law");
      write("initialize");
    }
  }
  void reset(const void *owner, const Eigen::MatrixXd &p) {
    if (!enabled_)
      return;
    owner_ = nullptr;
    initialize(owner, p);
  }
  // Any auxiliary variables including archived poses / anchors survive main
  // marginalization. Their cross rows are selected; their covariance is kept.
  void mainMap(const void *owner, const Eigen::MatrixXd &old_p, const Eigen::MatrixXd &map, const Eigen::MatrixXd &new_p,
               const std::string &event) {
    if (!enabled_)
      return;
    initialize(owner, old_p);
    cross_ = (map * cross_).eval();
    source_main_ = (map * source_main_).eval();
    const Eigen::MatrixXd q = new_p - map * old_p * map.transpose();
    p_ = (map * p_ * map.transpose() + q).eval();
    stored_ = new_p;
    write(event);
  }
  // B=Cov(delta_x,eps_visual), D=Cov(aux,eps_visual). Under residual H delta_x+eps,
  // delta_x^+=(I-KH)delta_x-K eps. This is the actual StateHelper update map.
  void visual(const void *owner, const Eigen::MatrixXd &old_p, const Eigen::MatrixXd &h, const Eigen::MatrixXd &k,
              const Eigen::MatrixXd &new_p, const Eigen::MatrixXd &r, const Eigen::MatrixXd *aux_noise_cross = nullptr,
              const Eigen::MatrixXd *main_noise_cross = nullptr) {
    if (!enabled_)
      return;
    initialize(owner, old_p);
    Eigen::MatrixXd traced_main, traced_aux;
    if (pending_visual_.rows() == k.cols() && pending_visual_.cols() == source_main_.cols()) {
      traced_main = source_main_ * pending_visual_.transpose();
      traced_aux = source_aux_ * pending_visual_.transpose();
      main_noise_cross = &traced_main;
      aux_noise_cross = &traced_aux;
    }
    const Eigen::MatrixXd f = Eigen::MatrixXd::Identity(p_.rows(), p_.rows()) - k * h;
    cross_ = (f * cross_).eval();
    p_ = (f * p_ * f.transpose() + k * r * k.transpose()).eval();
    if (main_noise_cross)
      p_ -= f * (*main_noise_cross) * k.transpose() + k * main_noise_cross->transpose() * f.transpose();
    else
      missing_.insert("main_visual_prior_noise_source_cross");
    if (aux_noise_cross) {
      if (aux_noise_cross->rows() != aux_.rows() || aux_noise_cross->cols() != k.cols())
        throw std::invalid_argument("visual cross shape");
      cross_.noalias() -= k * aux_noise_cross->transpose();
    } else if (aux_.rows())
      missing_.insert("visual_aux_noise_source_cross");
    source_main_ = (f * source_main_).eval();
    if (pending_visual_.rows() == k.cols() && pending_visual_.cols() == source_main_.cols())
      source_main_.noalias() -= k * pending_visual_ * source_variance_.asDiagonal();
    pending_visual_.resize(0, 0);
    stored_ = new_p;
    write("main_visual_actual_tracked_pixel_sources");
  }
  // Archive a main quantity/anchor rather than dropping its historical cross
  // when the clone leaves the active state. sign=-1 maps true-est -> est-true.
  int archiveMain(const Eigen::MatrixXd &j, double sign = -1.) {
    const Eigen::MatrixXd old_cross = cross_, old_aux = aux_;
    const int at = aux_.rows(), sz = j.rows();
    cross_.conservativeResize(p_.rows(), at + sz);
    cross_.rightCols(sz) = sign * p_ * j.transpose();
    aux_.conservativeResize(at + sz, at + sz);
    aux_.topRightCorner(at, sz) = sign * old_cross.transpose() * j.transpose();
    aux_.bottomLeftCorner(sz, at) = aux_.topRightCorner(at, sz).transpose();
    aux_.bottomRightCorner(sz, sz) = j * p_ * j.transpose();
    write("archive_main_anchor");
    return at;
  }
  // Joint seed append with ALL supplied historic bearing correlations. Unlike
  // admission-quality covariance this engine refuses incomplete source laws.
  int seed(const Eigen::MatrixXd &jx, const Eigen::MatrixXd &jz, const Eigen::MatrixXd &q, const Eigen::MatrixXd &main_source,
           const Eigen::MatrixXd &aux_source) {
    if (jx.cols() != p_.rows() || main_source.rows() != p_.rows() || aux_source.rows() != aux_.rows() || main_source.cols() != q.rows() ||
        aux_source.cols() != q.rows() || jz.cols() != q.rows())
      throw std::invalid_argument("seed joint source shape");
    const int at = aux_.rows(), sz = jx.rows();
    const Eigen::MatrixXd c = -p_ * jx.transpose() + main_source * jz.transpose();
    const Eigen::MatrixXd a = -cross_.transpose() * jx.transpose() + aux_source * jz.transpose();
    const Eigen::MatrixXd v = jx * p_ * jx.transpose() + jz * q * jz.transpose() - jx * main_source * jz.transpose() -
                              jz * main_source.transpose() * jx.transpose();
    cross_.conservativeResize(p_.rows(), at + sz);
    cross_.rightCols(sz) = c;
    aux_.conservativeResize(at + sz, at + sz);
    aux_.topRightCorner(at, sz) = a;
    aux_.bottomLeftCorner(sz, at) = a.transpose();
    aux_.bottomRightCorner(sz, sz) = v;
    write("seed_true_est_sign_complete_supplied_law");
    return at;
  }
  // Native JPLQuat::update uses normalized [dx/2,1], not exponential.
  // Differentiating 2 vec(q(dx+r) x inv(q(dx))) / scalar at r=0 gives
  // (I - [dx]/2)/(1+|dx|^2/4). Additive position/bias reset is identity.
  static Eigen::Matrix3d nativeJplReset(const Eigen::Vector3d &dx) {
    Eigen::Matrix3d skew;
    skew << 0, -dx.z(), dx.y(), dx.z(), 0, -dx.x(), -dx.y(), dx.x(), 0;
    return (Eigen::Matrix3d::Identity() - .5 * skew) / (1. + .25 * dx.squaredNorm());
  }
  static std::string pixelKey(size_t feature, int camera, double imu_time) {
    std::ostringstream out;
    out << feature << '/' << camera << '/' << std::setprecision(17) << imu_time;
    return out.str();
  }
  int sourceColumn(const std::string &key) const {
    auto it = sources_.find(key);
    return it == sources_.end() ? -1 : it->second;
  }
  int sourceDimension() const { return source_main_.cols(); }
  void usedVisualSources(const std::vector<std::string> &keys) {
    for (const auto &key : keys)
      used_visual_.insert(key);
  }
  void visualNoiseMap(const Eigen::MatrixXd &map) { pending_visual_ = map; }
  bool registerPixel(const std::string &key, double variance) {
    if (key.empty() || !(variance > 0)) {
      missing_.insert("pixel_source_metadata_unavailable");
      return false;
    }
    if (sources_.count(key))
      return true;
    if (source_main_.cols() + 2 > 4096) {
      missing_.insert("pixel_source_capacity_4096");
      return false;
    }
    if (used_visual_.count(key))
      missing_.insert("pixel_registered_after_prior_visual_consumption");
    const int col = source_main_.cols();
    sources_[key] = col;
    source_main_.conservativeResize(p_.rows(), col + 2);
    source_main_.rightCols(2).setZero();
    source_aux_.conservativeResize(aux_.rows(), col + 2);
    source_aux_.rightCols(2).setZero();
    source_variance_.conservativeResize(col + 2);
    source_variance_.tail(2).setConstant(variance);
    return true;
  }
  int trackedSeed(const Eigen::MatrixXd &jx, const Eigen::MatrixXd &jpixel, const std::vector<std::string> &keys, double variance) {
    if (jpixel.cols() != 2 * static_cast<int>(keys.size()))
      throw std::invalid_argument("seed pixel columns");
    for (const auto &key : keys)
      if (!registerPixel(key, variance))
        return -1;
    Eigen::MatrixXd mx(p_.rows(), jpixel.cols()), ax(aux_.rows(), jpixel.cols());
    Eigen::MatrixXd q = Eigen::MatrixXd::Identity(jpixel.cols(), jpixel.cols()) * variance;
    Eigen::MatrixXd all_j = Eigen::MatrixXd::Zero(jpixel.rows(), source_main_.cols());
    for (size_t i = 0; i < keys.size(); ++i) {
      const int col = sourceColumn(keys[i]);
      mx.middleCols(2 * i, 2) = source_main_.middleCols(col, 2);
      ax.middleCols(2 * i, 2) = source_aux_.middleCols(col, 2);
      all_j.middleCols(col, 2) += jpixel.middleCols(2 * i, 2);
      for (size_t j = 0; j < i; ++j)
        if (keys[i] == keys[j]) {
          q.block<2, 2>(2 * i, 2 * j).setIdentity();
          q.block<2, 2>(2 * j, 2 * i).setIdentity();
          q.block<2, 2>(2 * i, 2 * j) *= variance;
          q.block<2, 2>(2 * j, 2 * i) *= variance;
        }
    }
    const int at = seed(jx, jpixel, q, mx, ax);
    const Eigen::MatrixXd new_source = -jx * source_main_ + all_j * source_variance_.asDiagonal();
    source_aux_.conservativeResize(aux_.rows(), source_aux_.cols());
    source_aux_.bottomRows(jx.rows()) = new_source;
    write("seed_actual_pixel_source_receipt");
    return at;
  }
  int conditionalSeed(const Eigen::MatrixXd &jx, const Eigen::MatrixXd &jz, const Eigen::MatrixXd &q) {
    missing_.insert("seed_main_historical_bearing_cross");
    missing_.insert("seed_aux_historical_bearing_cross");
    // Explicit conditional geometric/source-independent component only. The
    // missing set prevents this block from being consumed as an unconditional
    // covariance. The full seed() entry point above requires supplied cross.
    return seed(jx, jz, q, Eigen::MatrixXd::Zero(p_.rows(), q.rows()), Eigen::MatrixXd::Zero(aux_.rows(), q.rows()));
  }
  void errorReset(const Eigen::MatrixXd &j) {
    cross_ = (j * cross_).eval();
    source_main_ = (j * source_main_).eval();
    p_ = (j * p_ * j.transpose()).eval();
    write("explicit_main_error_reset");
  }
  const Eigen::MatrixXd &mainCovariance() const { return p_; }
  const Eigen::MatrixXd &cross() const { return cross_; }
  const Eigen::MatrixXd &auxCovariance() const { return aux_; }
  int auxiliaryDimension() const { return aux_.rows(); }
  void missing(const std::string &name) { missing_.insert(name); }

private:
  LtvMainCrossShadow() {
    const char *path = std::getenv("LTV_MAIN_CROSS_SHADOW_PATH");
    enabled_ = path && *path;
    if (!enabled_)
      return;
    if (std::ifstream(path).good())
      throw std::runtime_error("refusing main cross shadow overwrite");
    out_.open(path);
    if (!out_)
      throw std::runtime_error("cannot open main cross shadow");
    out_ << "event,main_dimension,aux_dimension,cross_norm,aux_trace,physical_minus_stored_norm,missing_blocks\n";
  }
  void write(const std::string &event) {
    if (!enabled_)
      return;
    out_ << event << ',' << p_.rows() << ',' << aux_.rows() << ',' << cross_.norm() << ',' << aux_.trace() << ','
         << (p_.rows() == stored_.rows() ? (p_ - stored_).norm() : -1) << ',';
    for (const auto &m : missing_)
      out_ << m << ';';
    out_ << '\n';
  }
  bool enabled_ = false;
  const void *owner_ = nullptr;
  Eigen::MatrixXd p_, stored_, cross_, aux_, source_main_, source_aux_, pending_visual_;
  Eigen::VectorXd source_variance_;
  std::map<std::string, int> sources_;
  std::set<std::string> used_visual_;
  std::set<std::string> missing_;
  std::ofstream out_;
};
} // namespace ltv
