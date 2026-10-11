#include "ltv/fusion/UpdaterLTV.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include <Eigen/Cholesky>

namespace ov_msckf {
namespace {
// Match StateHelper::EKFUpdate exactly: one authoritative upper triangle.
// Computing both triangles independently can expose cancellation roundoff;
// the native update consumes selfadjointView<Upper>(), not that raw matrix.
Eigen::MatrixXd innovation_matrix(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const Eigen::MatrixXd &R) {
  Eigen::MatrixXd S(R.rows(), R.cols());
  S.triangularView<Eigen::Upper>() = H * P * H.transpose();
  S.triangularView<Eigen::Upper>() += R;
  return S.selfadjointView<Eigen::Upper>();
}
bool layout(const MeasurementBlock &b, std::string &reason) {
  int n = 0;
  for (size_t i = 0; i < b.order.size(); ++i) {
    const auto &a = b.order[i];
    if (!a || a->id() < 0 || a->size() <= 0) {
      reason = "invalid_type";
      return false;
    }
    for (size_t j = 0; j < i; ++j) {
      const auto &other = b.order[j];
      if (a->id() < other->id() + other->size() && other->id() < a->id() + a->size()) {
        reason = "overlapping_columns";
        return false;
      }
    }
    n += a->size();
  }
  if (b.H.rows() != b.res.rows() || b.H.cols() != n || b.R.rows() != b.res.rows() || b.R.cols() != b.res.rows()) {
    reason = "dimension_mismatch";
    return false;
  }
  if (!b.H.allFinite() || !b.res.allFinite() || !b.R.allFinite()) {
    reason = "nonfinite_measurement";
    return false;
  }
  return true;
}
double nis(const std::shared_ptr<State> &state, const MeasurementBlock &b) {
  const auto P = StateHelper::get_marginal_covariance(state, b.order);
  const Eigen::MatrixXd S = innovation_matrix(b.H, P, b.R);
  Eigen::LLT<Eigen::MatrixXd> solver(S);
  if (solver.info() != Eigen::Success)
    return -1;
  const double value = b.res.dot(solver.solve(b.res));
  return std::isfinite(value) && value >= 0 ? value : -1;
}
} // namespace
LtvContext UpdaterLTV::context(const State &s) const {
  LtvContext c;
  c.R_current = s._imu->Rot();
  c.v_current = s._imu->vel();
  c.R_linearization = s._options.do_fej ? s._imu->Rot_fej() : c.R_current;
  c.v_linearization = s._options.do_fej ? s._imu->vel_fej() : c.v_current;
  const Eigen::Vector3d d = c.R_linearization * c.gamma;
  Eigen::Index axis;
  d.cwiseAbs().minCoeff(&axis);
  const Eigen::Vector3d a = Eigen::Vector3d::Unit(axis);
  c.T.col(0) = (a - d * d.dot(a)).normalized();
  c.T.col(1) = d.cross(c.T.col(0));
  return c;
}
MeasurementBlock UpdaterLTV::gravity(const State &s, const LtvContext &c, const ltv::LtvSnapshot &snapshot) const {
  MeasurementBlock b;
  b.order = {s._imu->q(), s._imu->v()};
  b.H = Eigen::MatrixXd::Zero(2, 6);
  b.H.leftCols(3) = c.T.transpose() * ov_core::skew_x(c.R_linearization * c.gamma);
  b.res = c.T.transpose() * (snapshot.gravity_body.normalized() - c.R_current * c.gamma);
  const double sigma = options_.sigma_gravity_deg * std::acos(-1.0) / 180;
  b.R = sigma * sigma * Eigen::Matrix2d::Identity();
  return b;
}
MeasurementBlock UpdaterLTV::velocity(const State &s, const LtvContext &c, const ltv::LtvSnapshot &snapshot) const {
  MeasurementBlock b;
  b.order = {s._imu->q(), s._imu->v()};
  b.H = Eigen::MatrixXd::Zero(3, 6);
  b.H.leftCols(3) = ov_core::skew_x(c.R_linearization * c.v_linearization);
  b.H.rightCols(3) = c.R_linearization;
  b.res = snapshot.velocity_body - c.R_current * c.v_current;
  b.R = options_.sigma_velocity_mps * options_.sigma_velocity_mps * Eigen::Matrix3d::Identity();
  return b;
}
MeasurementBlock UpdaterLTV::build(const std::shared_ptr<State> &state, const LtvFrame &f, MeasurementBlock *diagnostic) const {
  MeasurementBlock b;
  b.H.resize(0, 0);
  b.R.resize(0, 0);
  b.receipt = std::make_shared<LtvReceipt>();
  b.token = f;
  if (diagnostic)
    *diagnostic = b;
  auto &d = b.receipt->diagnostics;
  d.prior_rotation_difference = (state->_imu->Rot() - state->_imu->Rot_fej()).norm();
  d.prior_velocity_difference = (state->_imu->vel() - state->_imu->vel_fej()).norm();
  d.gravity_variance = std::pow(options_.sigma_gravity_deg * std::acos(-1.0) / 180, 2);
  d.velocity_variance = std::pow(options_.sigma_velocity_mps, 2);
  auto reject = [&](const std::string &reason) {
    if (options_.enable_gravity)
      d.gravity_reason = reason;
    if (options_.enable_velocity)
      d.velocity_reason = reason;
    if (options_.enable_landmark_approx)
      d.landmark_reason = reason;
  };
  if (!options_.enabled || !options_.allow_correlated_pseudomeasurements) {
    reject("disabled_or_unacknowledged");
    return b;
  }
  if (!f.available || !std::isfinite(f.camera_time) || !std::isfinite(f.imu_time) || !std::isfinite(f.cursor) ||
      !std::isfinite(f.snapshot.imu_timestamp) || !std::isfinite(f.snapshot.frame_timestamp) ||
      !std::isfinite(state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0)) || f.epoch == 0 || f.sequence == 0 ||
      std::abs(state->_timestamp - f.camera_time) > options_.time_tolerance_s ||
      std::abs(state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0) - f.imu_time) > options_.time_tolerance_s ||
      std::abs(f.cursor - f.imu_time) > options_.time_tolerance_s ||
      std::abs(f.snapshot.imu_timestamp - f.imu_time) > options_.time_tolerance_s ||
      std::abs(f.snapshot.frame_timestamp - f.camera_time) > options_.time_tolerance_s) {
    reject("time_or_token");
    return b;
  }
  if (!f.snapshot.valid) {
    reject("warmup_or_core_invalid");
    return b;
  }
  if (!state->_imu->value().allFinite() || !state->_imu->fej().allFinite() || std::abs(state->_imu->quat().norm() - 1) > 1e-8 ||
      std::abs(state->_imu->quat_fej().norm() - 1) > 1e-8) {
    throw std::runtime_error("nonfinite or invalid native quaternion/state");
  }
  const auto c = context(*state);
  const auto &snap = f.snapshot;
  auto gate = [&](const MeasurementBlock &block, double threshold, double &value, std::string &reason) {
    std::string why;
    if (!validate_measurement(state, block, why, &value)) {
      reason = why;
      return false;
    }
    if (value < 0) {
      reason = "invalid_nis";
      return false;
    }
    if (options_.enable_nis_gate && value > threshold) {
      reason = "nis";
      return false;
    }
    reason = "accepted";
    return true;
  };
  // One frozen IRLS weight per branch, after gating against the original noise.
  // This is a robust EKF step, not an iterated nonlinear sliding-window solve.
  auto robustify = [&](MeasurementBlock &block, double &variance) {
    if (options_.enable_huber) {
      Eigen::LLT<Eigen::MatrixXd> noise(block.R);
      const double magnitude = noise.matrixL().solve(block.res).norm();
      if (magnitude > options_.huber_delta)
        block.R *= magnitude / options_.huber_delta;
    }
    variance = block.R(0, 0);
  };
  if (options_.enable_gravity && !f.ready_G)
    d.gravity_reason = "not_ready_G";
  if (options_.enable_gravity && f.ready_G) {
    d.gravity_reason = "eta_invalid";
    const double norm = snap.gravity_body.norm();
    if (snap.gravity_valid && snap.gravity_body.allFinite() && norm >= options_.observer.gravity_norm_min &&
        norm <= options_.observer.gravity_norm_max) {
      const Eigen::Vector3d u = snap.gravity_body / norm;
      const double cosine = std::max(-1.0, std::min(1.0, u.dot(c.R_current * c.gamma)));
      d.gravity_reason = "angle";
      if (std::acos(cosine) <= options_.max_gravity_angle_deg * std::acos(-1.0) / 180 && u.dot(c.R_linearization * c.gamma) > 0) {
        d.gravity_eligible = true;
        auto g = gravity(*state, c, snap);
        d.gravity_residual_vector = g.res;
        d.gravity_residual = g.res.norm();
        d.gravity_quality =
            !options_.enable_quality_gate ||
            (snap.observed_features >= options_.quality_min_features && std::abs(norm - gravity_mag_) <= options_.quality_eta_error &&
             snap.innovation_norm / std::sqrt(std::max(1, 3 * snap.observed_features)) <= options_.quality_innovation);
        d.gravity_reason = "quality";
        if (d.gravity_quality && gate(g, options_.nis_gravity, d.gravity_nis, d.gravity_reason)) {
          robustify(g, d.gravity_variance);
          b = merge_measurements(b, g);
          d.gravity_rows = 2;
        }
      }
    }
  }
  if (options_.enable_velocity && !f.ready_V)
    d.velocity_reason = "not_ready_V";
  if (options_.enable_velocity && f.ready_V) {
    d.velocity_reason = "velocity_invalid";
    if (snap.velocity_valid && snap.velocity_body.allFinite()) {
      d.velocity_eligible = true;
      auto v = velocity(*state, c, snap);
      d.velocity_residual_vector = v.res;
      d.velocity_residual = v.res.norm();
      d.velocity_quality = !options_.enable_quality_gate ||
                           (snap.observed_features >= options_.quality_min_features &&
                            snap.innovation_norm / std::sqrt(std::max(1, 3 * snap.observed_features)) <= options_.quality_innovation &&
                            v.res.norm() <= options_.quality_velocity_disagreement);
      d.velocity_reason = "quality";
      if (d.velocity_quality && gate(v, options_.nis_velocity, d.velocity_nis, d.velocity_reason)) {
        robustify(v, d.velocity_variance);
        b = merge_measurements(b, v);
        d.velocity_rows = 3;
      }
    }
  }
  b = landmark(state, f, b);
  return b;
}
Eigen::Vector3d UpdaterLTV::landmark_prediction(const ov_type::PoseJPL &t, const ov_type::PoseJPL &a, const Eigen::Vector3d &l_a,
                                                bool fej) {
  const Eigen::Matrix3d Rt = fej ? t.Rot_fej() : t.Rot(), Ra = fej ? a.Rot_fej() : a.Rot();
  const Eigen::Vector3d pt = fej ? t.pos_fej() : t.pos(), pa = fej ? a.pos_fej() : a.pos();
  return Rt * (Ra.transpose() * l_a + pa - pt);
}
Eigen::Matrix<double, 3, 12> UpdaterLTV::landmark_jacobian(const ov_type::PoseJPL &t, const ov_type::PoseJPL &a, const Eigen::Vector3d &l_a,
                                                           bool fej) {
  const Eigen::Matrix3d Rt = fej ? t.Rot_fej() : t.Rot(), Ra = fej ? a.Rot_fej() : a.Rot();
  Eigen::Matrix<double, 3, 12> H;
  H << ov_core::skew_x(landmark_prediction(t, a, l_a, fej)), -Rt, -Rt * Ra.transpose() * ov_core::skew_x(l_a), Rt;
  return H;
}

MeasurementBlock UpdaterLTV::landmark(const std::shared_ptr<State> &state, const LtvFrame &frame, MeasurementBlock base) const {
  if (!options_.enable_landmark_approx)
    return base;
  auto &d = base.receipt->diagnostics;
  d.landmark_reason = "no_candidates";
  int selected = 0;
  MeasurementBlock landmarks;
  for (const auto &candidate : frame.landmarks) {
    if (selected >= options_.landmark_approx_max_points)
      break;
    if (candidate.epoch != frame.epoch || !candidate.point.allFinite() || !candidate.anchor_point.allFinite() ||
        !std::isfinite(candidate.span) || candidate.span < .2 - 1e-6 || candidate.span > .55 + 1e-6) {
      d.landmark_reason = "invalid_candidate";
      continue;
    }
    std::shared_ptr<ov_type::PoseJPL> anchor;
    for (const auto &clone : state->_clones_IMU)
      if (std::abs(clone.first + state->_calib_dt_CAMtoIMU->value()(0) - candidate.anchor_time) <= 1e-9)
        anchor = clone.second;
    if (!anchor || candidate.anchor_time >= frame.imu_time - 1e-9) {
      d.landmark_reason = "anchor_not_live";
      continue;
    }
    MeasurementBlock point;
    point.order = {state->_imu->q(), state->_imu->p(), anchor};
    point.H = landmark_jacobian(*state->_imu->pose(), *anchor, candidate.anchor_point, state->_options.do_fej);
    point.res = candidate.point - landmark_prediction(*state->_imu->pose(), *anchor, candidate.anchor_point);
    const double sigma = options_.landmark_approx_sigma_floor_m + .02 * candidate.point.norm();
    double variance = sigma * sigma * (1 + candidate.span / .55);
    const auto P = StateHelper::get_marginal_covariance(state, point.order);
    const double signal = (point.H * P * point.H.transpose()).trace();
    const double cap = options_.landmark_approx_information_cap / options_.landmark_approx_max_points;
    if (!std::isfinite(signal) || signal < 0 || !std::isfinite(cap) || cap <= 0) {
      d.landmark_reason = "invalid_information";
      continue;
    }
    variance = std::max(variance, signal / cap);
    point.R = variance * Eigen::Matrix3d::Identity();
    std::string reason;
    if (!validate_measurement(state, point, reason)) {
      d.landmark_reason = reason;
      continue;
    }
    landmarks = merge_measurements(landmarks, point);
    d.landmark_reason = "accepted";
    ++selected;
  }
  d.landmark_rows = 3 * selected;
  if (selected) {
    d.landmark_nis = nis(state, landmarks);
    base = merge_measurements(base, landmarks);
  }
  return base;
}

MeasurementBlock merge_measurements(const MeasurementBlock &a, const MeasurementBlock &b) {
  std::string reason;
  if (!layout(a, reason) || !layout(b, reason))
    throw std::invalid_argument(reason);
  MeasurementBlock result = a;
  std::vector<int> dest;
  int columns = a.H.cols();
  for (const auto &type : b.order) {
    int offset = 0, found = -1;
    for (const auto &existing : result.order) {
      if (type.get() == existing.get() && type->id() == existing->id() && type->size() == existing->size())
        found = offset;
      else if (type->id() < existing->id() + existing->size() && existing->id() < type->id() + type->size())
        throw std::invalid_argument("overlapping type ranges");
      offset += existing->size();
    }
    if (found < 0) {
      found = columns;
      columns += type->size();
      result.order.push_back(type);
    }
    dest.push_back(found);
  }
  const int ma = a.res.size(), mb = b.res.size();
  result.H = Eigen::MatrixXd::Zero(ma + mb, columns);
  result.res.resize(ma + mb);
  result.R = Eigen::MatrixXd::Zero(ma + mb, ma + mb);
  if (ma) {
    result.H.topLeftCorner(ma, a.H.cols()) = a.H;
    result.res.head(ma) = a.res;
    result.R.topLeftCorner(ma, ma) = a.R;
  }
  if (mb) {
    int source = 0;
    for (size_t i = 0; i < b.order.size(); ++i) {
      int size = b.order[i]->size();
      result.H.block(ma, dest[i], mb, size) = b.H.middleCols(source, size);
      source += size;
    }
    result.res.tail(mb) = b.res;
    result.R.bottomRightCorner(mb, mb) = b.R;
  }
  return result;
}
bool validate_measurement(const std::shared_ptr<State> &state, const MeasurementBlock &b, std::string &reason, double *nis_value) {
  if (nis_value)
    *nis_value = -1;
  if (!layout(b, reason))
    return false;
  if (b.empty())
    return true;
  for (const auto &type : b.order) {
    if (type->id() + type->size() > state->max_covariance_size()) {
      reason = "outside_covariance";
      return false;
    }
    bool owned = (type == state->_imu || state->_imu->check_if_subvariable(type) == type);
    for (const auto &clone : state->_clones_IMU)
      owned = owned || type == clone.second || clone.second->check_if_subvariable(type) == type;
    if (!owned) {
      reason = "foreign_type";
      return false;
    }
  }
  if ((b.R - b.R.transpose()).norm() > 1e-12 * std::max(1.0, b.R.norm())) {
    reason = "asymmetric_R";
    return false;
  }
  Eigen::LLT<Eigen::MatrixXd> noise(b.R);
  if (noise.info() != Eigen::Success) {
    reason = "nonSPD_R";
    return false;
  }
  const auto P = StateHelper::get_marginal_covariance(state, b.order);
  const Eigen::MatrixXd S = innovation_matrix(b.H, P, b.R);
  Eigen::LLT<Eigen::MatrixXd> innovation(S);
  if (!S.allFinite() || innovation.info() != Eigen::Success) {
    reason = "nonSPD_S";
    return false;
  }
  if (nis_value) {
    const double value = b.res.dot(innovation.solve(b.res));
    *nis_value = std::isfinite(value) && value >= 0 ? value : -1;
  }
  return true;
}
} // namespace ov_msckf
