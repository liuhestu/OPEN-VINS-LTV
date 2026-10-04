#include "LtvReadiness.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace ltv {
const char *toString(LtvAvailability s) {
  switch (s) {
  case LtvAvailability::Disabled:
    return "DISABLED";
  case LtvAvailability::Collecting:
    return "COLLECTING";
  case LtvAvailability::Bootstrapping:
    return "BOOTSTRAPPING";
  case LtvAvailability::Tracking:
    return "TRACKING";
  case LtvAvailability::Degraded:
    return "DEGRADED";
  case LtvAvailability::Dormant:
    return "DORMANT";
  case LtvAvailability::Recovering:
    return "RECOVERING";
  }
  return "UNKNOWN";
}
LtvReadiness::LtvReadiness(const LtvReadinessConfig &c) : config_(c) {
  if (c.min_features < 15 || c.min_features > 30 || !c.bootstrap_confirm_frames || !c.ready_confirm_frames || !c.min_actual_corrections)
    throw std::invalid_argument("invalid readiness counts");
  for (double v :
       {c.coast_seconds, c.dormant_seconds, c.quality_bootstrap_interval, c.timestamp_tolerance, c.confirmation_max_gap, c.gravity_norm_min,
        c.gravity_norm_max, c.prediction_angle_limit_rad, c.velocity_correction_rate_limit, c.gravity_correction_rate_limit})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("invalid readiness threshold");
  if (c.dormant_seconds < c.coast_seconds || c.quality_bootstrap_interval < 30 || c.gravity_norm_min >= c.gravity_norm_max)
    throw std::invalid_argument("invalid readiness recovery limits");
}
void LtvReadiness::clearGrace() {
  soft_G_ = soft_V_ = 0;
  last_good_G_ = last_good_V_ = -1;
}
void LtvReadiness::reset() {
  state_ = LtvAvailability::Collecting;
  last_time_ = shortage_since_ = last_bootstrap_ = last_quality_bootstrap_ = -1;
  supply_frames_ = gravity_frames_ = velocity_frames_ = 0;
  clearGrace();
  pending_physical_ = pending_initial_warmup_ = false;
  awaiting_bootstrap_ = bootstrapped_ = dormant_requested_ = physical_recovery_ = fault_latched_ = false;
  bootstrap_count_ = physical_fault_count_ = 0;
  bootstrap_source_.clear();
}
void LtvReadiness::validateCommit(const LtvReadinessInput &i) const {
  if (!awaiting_bootstrap_ || i.bootstrap_was_physical_recovery != pending_physical_)
    throw std::invalid_argument("bootstrap ack has no matching pending request");
  if (pending_initial_warmup_ && i.bootstrap_source != "BOUNDED_INITIAL_ZERO_WARMUP")
    throw std::invalid_argument("initial warmup ack source mismatch");
  if (i.bootstrap_source.empty() || !i.observer_started || !i.observer_finite || !i.velocity.allFinite() || !i.gravity.allFinite() ||
      !std::isfinite(i.observer_time) || std::abs(i.observer_time - i.time) > config_.timestamp_tolerance)
    throw std::invalid_argument("invalid committed readiness bootstrap");
  if (bootstrapped_ && !i.bootstrap_was_physical_recovery && last_quality_bootstrap_ >= 0 &&
      i.time - last_quality_bootstrap_ < config_.quality_bootstrap_interval)
    throw std::invalid_argument("quality bootstrap cooldown violated");
  if (i.bootstrap_was_physical_recovery && !physical_recovery_)
    throw std::invalid_argument("undeclared physical recovery bootstrap");
}
void LtvReadiness::commit(const LtvReadinessInput &i) {
  last_bootstrap_ = i.time;
  if (!i.bootstrap_was_physical_recovery)
    last_quality_bootstrap_ = i.time;
  bootstrap_source_ = i.bootstrap_source;
  ++bootstrap_count_;
  bootstrapped_ = true;
  awaiting_bootstrap_ = physical_recovery_ = dormant_requested_ = false;
  supply_frames_ = gravity_frames_ = velocity_frames_ = 0;
  clearGrace();
  shortage_since_ = pending_initial_warmup_ && !(i.geometry_valid && i.observed_features >= config_.min_features) ? i.time : -1;
  pending_initial_warmup_ = false;
  state_ = LtvAvailability::Bootstrapping;
}
LtvReadinessOutput LtvReadiness::acknowledgeBootstrap(const LtvReadinessInput &i) {
  if (!config_.enabled || !std::isfinite(i.time) || i.time != last_time_ || !i.clock_valid || i.physical_input_fault)
    throw std::invalid_argument("same-event bootstrap ack time/fault mismatch");
  validateCommit(i);
  commit(i);
  LtvReadinessOutput o;
  o.ready_soft_grace_enabled = config_.ready_soft_grace;
  o.accepted_time = true;
  o.state = state_;
  o.observer_valid = true;
  o.reason = "bootstrap_committed_confirming";
  o.last_bootstrap_time = last_bootstrap_;
  o.last_bootstrap_source = bootstrap_source_;
  o.bootstrap_count = bootstrap_count_;
  o.physical_fault_count = physical_fault_count_;
  return o;
}
LtvReadinessOutput LtvReadiness::update(const LtvReadinessInput &i) {
  LtvReadinessOutput o;
  o.ready_soft_grace_enabled = config_.ready_soft_grace;
  auto finish = [&](const char *reason) {
    o.state = state_;
    o.reason = reason;
    o.last_bootstrap_time = last_bootstrap_;
    o.last_bootstrap_source = bootstrap_source_;
    o.bootstrap_count = bootstrap_count_;
    o.physical_fault_count = physical_fault_count_;
    o.joint_ready = o.ready_G && o.ready_V;
    o.soft_failure_frames_G = soft_G_;
    o.soft_failure_frames_V = soft_V_;
    o.last_strict_good_G = last_good_G_;
    o.last_strict_good_V = last_good_V_;
    return o;
  };
  if (!config_.enabled)
    return o;
  if (!std::isfinite(i.time) || i.time < 0 || i.time <= last_time_)
    return finish("invalid_or_duplicate_event_time");
  const bool gap = last_time_ >= 0 && i.time - last_time_ > config_.confirmation_max_gap + config_.timestamp_tolerance;
  if (i.bootstrap_committed)
    throw std::invalid_argument("delayed bootstrap ack unsupported; use same-event acknowledgeBootstrap");
  o.accepted_time = true;
  last_time_ = i.time;
  if (i.physical_input_fault || !i.clock_valid) {
    if (!fault_latched_)
      ++physical_fault_count_;
    awaiting_bootstrap_ = false;
    fault_latched_ = true;
    physical_recovery_ = true;
    supply_frames_ = gravity_frames_ = velocity_frames_ = 0;
    clearGrace();
    state_ = LtvAvailability::Dormant;
    if (!dormant_requested_) {
      o.request_dormant = true;
      dormant_requested_ = true;
    }
    return finish("physical_input_fault");
  }
  fault_latched_ = false;
  if (gap) {
    awaiting_bootstrap_ = false;
    supply_frames_ = gravity_frames_ = velocity_frames_ = 0;
    clearGrace();
  }
  const bool supply = i.geometry_valid && i.eligible_seeds >= config_.min_features;
  supply_frames_ = supply ? std::min(supply_frames_ + 1, config_.bootstrap_confirm_frames) : 0;
  o.pool_ready = i.geometry_valid && i.observed_features >= config_.min_features && i.mature_observed_features >= config_.min_features;
  const bool current = i.observer_started && i.observer_finite && i.velocity.allFinite() && i.gravity.allFinite() &&
                       std::isfinite(i.observer_time) && std::abs(i.observer_time - i.time) <= config_.timestamp_tolerance;
  o.observer_valid = current && state_ != LtvAvailability::Dormant && state_ != LtvAvailability::Collecting;
  if (config_.preserve_constrained_state && current && (i.velocity.norm() > 100.0 || i.gravity.norm() > 50.0)) {
    awaiting_bootstrap_ = false;
    gravity_frames_ = velocity_frames_ = 0;
    clearGrace();
    state_ = LtvAvailability::Dormant;
    o.observer_valid = false;
    if (!dormant_requested_) {
      o.request_dormant = true;
      dormant_requested_ = true;
    }
    return finish("observer_norm_guard");
  }
  // No reliable current state exists in collection/dormancy. Merely having a
  // finite stale vector cannot promote a stopped observer back to TRACKING.
  if (awaiting_bootstrap_ || !bootstrapped_ || state_ == LtvAvailability::Dormant || state_ == LtvAvailability::Collecting ||
      (state_ == LtvAvailability::Recovering && !current)) {
    o.observer_valid = false;
    const bool cooldown =
        physical_recovery_ || last_quality_bootstrap_ < 0 || i.time - last_quality_bootstrap_ >= config_.quality_bootstrap_interval;
    // This preserves an already current core; it does not manufacture one and
    // cannot be reused for later observer epochs or physical-fault recovery.
    if (config_.initial_unseeded_warmup && !bootstrapped_ && !physical_recovery_ && current) {
      o.request_bootstrap = !awaiting_bootstrap_;
      awaiting_bootstrap_ = true;
      pending_physical_ = false;
      pending_initial_warmup_ = true;
      state_ = LtvAvailability::Recovering;
      return finish("initial_bounded_warmup");
    }
    if (supply_frames_ >= config_.bootstrap_confirm_frames && cooldown) {
      pending_initial_warmup_ = false;
      o.request_bootstrap = !awaiting_bootstrap_;
      awaiting_bootstrap_ = true;
      pending_physical_ = physical_recovery_;
      o.bootstrap_is_physical_recovery = physical_recovery_;
      state_ = LtvAvailability::Recovering;
      return finish("atomic_bootstrap_required");
    }
    awaiting_bootstrap_ = false;
    state_ = bootstrapped_ ? LtvAvailability::Dormant : LtvAvailability::Collecting;
    return finish(!supply ? "insufficient_seed_supply" : !cooldown ? "quality_bootstrap_cooldown" : "confirming_seed_supply");
  }
  const bool adequate = i.geometry_valid && i.observed_features >= config_.min_features;
  if (config_.preserve_constrained_state && current && i.geometry_valid && i.observed_features > 0 && !adequate) {
    // Thin current constraints permit continued estimation, never readiness.
    // Only a new complete loss starts the no-observation dormant clock.
    shortage_since_ = -1;
    gravity_frames_ = velocity_frames_ = 0;
    clearGrace();
    state_ = LtvAvailability::Degraded;
    return finish("thin_observations_state_preserved_not_ready");
  }
  if (!adequate || !current) {
    gravity_frames_ = velocity_frames_ = 0;
    clearGrace();
    if (shortage_since_ < 0)
      shortage_since_ = i.time;
    const double duration = i.time - shortage_since_;
    state_ = LtvAvailability::Degraded;
    if (!current || duration >= config_.dormant_seconds) {
      state_ = LtvAvailability::Dormant;
      o.observer_valid = false;
      if (!dormant_requested_) {
        o.request_dormant = true;
        dormant_requested_ = true;
      }
      return finish(!current ? "observer_not_current_or_finite" : "prolonged_insufficient_geometry");
    }
    return finish(duration <= config_.coast_seconds ? "finite_coast_not_ready" : "insufficient_geometry_not_ready");
  }
  shortage_since_ = -1;
  const bool diagnostics = i.correction_diagnostics_valid && std::isfinite(i.prediction_angle_p95_rad) && i.prediction_angle_p95_rad >= 0 &&
                           i.prediction_angle_p95_rad <= config_.prediction_angle_limit_rad && std::isfinite(i.velocity_correction_rate) &&
                           i.velocity_correction_rate >= 0 && std::isfinite(i.gravity_correction_rate) && i.gravity_correction_rate >= 0;
  const double gn = i.gravity.norm();
  const bool gravity_physical = gn >= config_.gravity_norm_min && gn <= config_.gravity_norm_max;
  const bool common = o.pool_ready && current && diagnostics && i.actual_corrections >= config_.min_actual_corrections;
  const bool good_G = common && gravity_physical && i.gravity_correction_rate <= config_.gravity_correction_rate_limit;
  const bool good_V = common && gravity_physical && i.velocity_correction_rate <= config_.velocity_correction_rate_limit &&
                      i.gravity_correction_rate <= config_.gravity_correction_rate_limit;
  if (!config_.ready_soft_grace) {
    gravity_frames_ = good_G ? std::min(gravity_frames_ + 1, config_.ready_confirm_frames) : 0;
    velocity_frames_ = good_V ? std::min(velocity_frames_ + 1, config_.ready_confirm_frames) : 0;
    o.ready_G = gravity_frames_ >= config_.ready_confirm_frames;
    o.ready_V = velocity_frames_ >= config_.ready_confirm_frames;
  } else {
    const bool finite_diagnostics = i.correction_diagnostics_valid && std::isfinite(i.prediction_angle_p95_rad) &&
                                    i.prediction_angle_p95_rad >= 0 && std::isfinite(i.velocity_correction_rate) &&
                                    i.velocity_correction_rate >= 0 && std::isfinite(i.gravity_correction_rate) &&
                                    i.gravity_correction_rate >= 0;
    const bool hard =
        o.pool_ready && current && gravity_physical && finite_diagnostics && i.actual_corrections >= config_.min_actual_corrections;
    const bool envelope_G = hard && i.prediction_angle_p95_rad <= 2 * config_.prediction_angle_limit_rad &&
                            i.gravity_correction_rate <= 2 * config_.gravity_correction_rate_limit;
    const bool envelope_V = envelope_G && i.velocity_correction_rate <= 2 * config_.velocity_correction_rate_limit;
    auto branch = [&](bool strict_good, bool envelope, unsigned int &confirm, unsigned int &soft, double &last_good, bool &held) {
      if (strict_good) {
        confirm = std::min(confirm + 1, config_.ready_confirm_frames);
        soft = 0;
        last_good = i.time;
        return confirm >= config_.ready_confirm_frames;
      }
      const double deadline = std::nextafter(last_good + .10, std::numeric_limits<double>::infinity());
      if (confirm >= config_.ready_confirm_frames && envelope && soft < 2 && last_good >= 0 && i.time <= deadline) {
        ++soft;
        held = true;
        return true;
      }
      confirm = soft = 0;
      last_good = -1;
      return false;
    };
    o.ready_G = branch(good_G, envelope_G, gravity_frames_, soft_G_, last_good_G_, o.grace_G);
    o.ready_V = branch(good_V, envelope_V, velocity_frames_, soft_V_, last_good_V_, o.grace_V);
  }
  state_ = (o.ready_G || o.ready_V) ? LtvAvailability::Tracking : LtvAvailability::Bootstrapping;
  if (o.grace_G || o.grace_V)
    return finish(o.grace_G && o.grace_V ? "ready_soft_grace_GV" : o.grace_G ? "ready_soft_grace_G" : "ready_soft_grace_V");
  return finish(!o.pool_ready               ? "pool_not_mature"
                : !diagnostics              ? "prediction_or_correction_unhealthy"
                : !gravity_physical         ? "gravity_not_physical"
                : !(o.ready_G || o.ready_V) ? "confirming_observer_health"
                                            : "ready");
}
} // namespace ltv
