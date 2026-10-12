#include "update/UpdaterLTV.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include <Eigen/Cholesky>

namespace ov_msckf {
namespace {
// Match StateHelper::EKFUpdate exactly: one authoritative upper triangle.
// Computing both triangles independently can expose cancellation roundoff;
// the native update consumes selfadjointView<Upper>(), not that raw matrix.
double nis(const std::shared_ptr<State> &state, const MeasurementBlock &b) {
  std::string reason;
  double value = -1;
  validate_measurement(state, b, reason, &value);
  return value;
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
MeasurementBlock UpdaterLTV::build(const std::shared_ptr<State> &state, const LtvFrame &f, LtvDiagnostics *diagnostic) const {
  MeasurementBlock b;
  b.H.resize(0, 0);
  b.R.resize(0, 0);
  LtvDiagnostics local;
  auto &d = diagnostic ? *diagnostic : local;
  d = LtvDiagnostics();
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
  if (!f.available || !std::isfinite(f.camera_time) || !std::isfinite(f.imu_time) ||
      !std::isfinite(state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0)) || f.epoch == 0 || f.sequence == 0 ||
      std::abs(state->_timestamp - f.camera_time) > options_.time_tolerance_s ||
      std::abs(state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0) - f.imu_time) > options_.time_tolerance_s) {
    reject("time_or_token");
    return b;
  }
  if (!f.valid) {
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
    if (f.gravity_valid && snap.gravity_body.allFinite() && norm >= options_.gravity_norm_min && norm <= options_.gravity_norm_max) {
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
    if (f.velocity_valid && snap.velocity_body.allFinite()) {
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
  b = landmark(state, f, b, d);
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

MeasurementBlock UpdaterLTV::landmark(const std::shared_ptr<State> &state, const LtvFrame &frame, MeasurementBlock base,
                                      LtvDiagnostics &d) const {
  if (!options_.enable_landmark_approx)
    return base;
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

} // namespace ov_msckf

namespace ov_msckf {
void UpdaterLTV::reset_health() {
  gravity_frames_ = velocity_frames_ = soft_G_ = soft_V_ = 0;
  last_good_G_ = last_good_V_ = -1;
}
void UpdaterLTV::health_diagnostics(ltv::LtvAvailabilityOutput &o) const {
  o.soft_failure_frames_G = soft_G_;
  o.soft_failure_frames_V = soft_V_;
  o.last_strict_good_G = last_good_G_;
  o.last_strict_good_V = last_good_V_;
}
std::string UpdaterLTV::update_health(const ltv::LtvAvailabilityInput &i, ltv::LtvAvailabilityOutput &o) {
  const bool current = o.observer_valid;
  const bool diagnostics = i.correction_diagnostics_valid && std::isfinite(i.prediction_angle_p95_rad) && i.prediction_angle_p95_rad >= 0 &&
                           i.prediction_angle_p95_rad <= options_.health_options.prediction_angle_limit_rad &&
                           std::isfinite(i.velocity_correction_rate) && i.velocity_correction_rate >= 0 &&
                           std::isfinite(i.gravity_correction_rate) && i.gravity_correction_rate >= 0;
  const double gn = i.gravity.norm();
  const bool gravity_physical = gn >= options_.health_options.gravity_norm_min && gn <= options_.health_options.gravity_norm_max;
  const bool common = o.pool_ready && current && diagnostics && i.actual_corrections >= options_.health_options.min_actual_corrections;
  const bool good_G = common && gravity_physical && i.gravity_correction_rate <= options_.health_options.gravity_correction_rate_limit;
  const bool good_V = common && gravity_physical && i.velocity_correction_rate <= options_.health_options.velocity_correction_rate_limit &&
                      i.gravity_correction_rate <= options_.health_options.gravity_correction_rate_limit;
  const unsigned int velocity_confirm_frames = options_.health_options.ready_confirm_frames;
  if (!options_.health_options.ready_soft_grace) {
    gravity_frames_ = good_G ? std::min(gravity_frames_ + 1, options_.health_options.ready_confirm_frames) : 0;
    velocity_frames_ = good_V ? std::min(velocity_frames_ + 1, velocity_confirm_frames) : 0;
    o.ready_G = gravity_frames_ >= options_.health_options.ready_confirm_frames;
    o.ready_V = velocity_frames_ >= velocity_confirm_frames;
  } else {
    const bool finite_diagnostics = i.correction_diagnostics_valid && std::isfinite(i.prediction_angle_p95_rad) &&
                                    i.prediction_angle_p95_rad >= 0 && std::isfinite(i.velocity_correction_rate) &&
                                    i.velocity_correction_rate >= 0 && std::isfinite(i.gravity_correction_rate) &&
                                    i.gravity_correction_rate >= 0;
    const bool hard = o.pool_ready && current && gravity_physical && finite_diagnostics &&
                      i.actual_corrections >= options_.health_options.min_actual_corrections;
    const bool envelope_G = hard && i.prediction_angle_p95_rad <= 2 * options_.health_options.prediction_angle_limit_rad &&
                            i.gravity_correction_rate <= 2 * options_.health_options.gravity_correction_rate_limit;
    const bool envelope_V = envelope_G && i.velocity_correction_rate <= 2 * options_.health_options.velocity_correction_rate_limit;
    auto branch = [&](bool strict_good, bool envelope, unsigned int required, unsigned int &confirm, unsigned int &soft, double &last_good,
                      bool &held) {
      if (strict_good) {
        confirm = std::min(confirm + 1, required);
        soft = 0;
        last_good = i.time;
        return confirm >= required;
      }
      const double deadline = std::nextafter(last_good + .10, std::numeric_limits<double>::infinity());
      if (confirm >= required && envelope && soft < 2 && last_good >= 0 && i.time <= deadline) {
        ++soft;
        held = true;
        return true;
      }
      confirm = soft = 0;
      last_good = -1;
      return false;
    };
    o.ready_G = branch(good_G, envelope_G, options_.health_options.ready_confirm_frames, gravity_frames_, soft_G_, last_good_G_, o.grace_G);
    o.ready_V = branch(good_V, envelope_V, velocity_confirm_frames, velocity_frames_, soft_V_, last_good_V_, o.grace_V);
  }
  if (o.grace_G || o.grace_V)
    return std::string(o.grace_G && o.grace_V ? "ready_soft_grace_GV" : o.grace_G ? "ready_soft_grace_G" : "ready_soft_grace_V");
  return std::string(!o.pool_ready               ? "pool_not_mature"
                     : !diagnostics              ? "prediction_or_correction_unhealthy"
                     : !gravity_physical         ? "gravity_not_physical"
                     : !(o.ready_G || o.ready_V) ? "confirming_observer_health"
                                                 : "ready");
}

} // namespace ov_msckf
