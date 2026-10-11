#include "ltv/runtime/LtvRuntime.h"
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <set>

namespace ov_msckf {
LtvRuntime::LtvRuntime(const LtvOptions &options) : landmark_history_(options), options_(options) {
  options.validate_runtime();
  if (!options.feature_readiness_enabled)
    throw std::invalid_argument("compact_v2 requires the controlled feature manager");
  {
    auto health_config = options.hardening_readiness;
    health_config.enabled = true;
    health_config.initial_unseeded_warmup = options.hardening_initial_warmup;
    health_config.preserve_constrained_state = options.hardening_preserve_constrained_state;
    health_config.ready_soft_grace = options.hardening_ready_soft_grace;
    availability_.reset(new ltv::LtvAvailabilityPolicy(health_config));
  }
  auto config = options.observer;
  config.enable = options.enabled;
  observer_.configure(config);
  if (options.feature_readiness_enabled) {
    ltv::LtvFeatureConfig pipeline;
    pipeline.manager = options.feature_manager;
    pipeline.active_consistency = options.active_consistency;
    pipeline.active_consistency.history_window_s = options.feature_manager.history.history_window;
    pipeline.manager.apply_seed = options.feature_apply_seed;
    pipeline.manager.bounded_memory = options.passive_hardening_enabled;
    if (options.feature_seed_source == "STEREO")
      pipeline.source = ltv::FeatureSeedSource::Stereo;
    else if (options.feature_seed_source == "STEREO_THEN_TEMPORAL")
      pipeline.source = ltv::FeatureSeedSource::StereoThenTemporal;
    else if (options.feature_seed_source == "TEMPORAL_POSE")
      pipeline.source = ltv::FeatureSeedSource::TemporalPose;
    else
      throw std::invalid_argument("Unsupported feature seed source");
    pipeline.bearing_sigma_rad = options.feature_bearing_sigma_rad;
    feature_frontend_.reset(new ltv::LtvFeatureFrontend(pipeline));
  }
}
void LtvRuntime::feed_imu(const ov_core::ImuData &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(sample.timestamp) || !sample.am.allFinite() || !sample.wm.allFinite()) {
    imu_input_fault_ = true;
    return;
  }
  if (!imu_.empty() && sample.timestamp <= imu_.back().timestamp) {
    imu_input_fault_ = true;
    return;
  }
  if (options_.passive_hardening_enabled && imu_.size() >= 4096) {
    // Explicit input fault; preserve a bounded newest bracket, never silently
    // integrate across discarded samples.
    imu_.pop_front();
    imu_input_fault_ = true;
  }
  imu_.push_back(sample);
  while (imu_.size() > 2 && imu_[1].timestamp < sample.timestamp - 10.0)
    imu_.pop_front();
}
LtvFrame LtvRuntime::frame(double t, uint64_t version, const std::string &reason) const {
  LtvFrame f;
  f.epoch = epoch_;
  f.version = version;
  f.sequence = sequence_;
  f.camera_time = t;
  f.camera_ns = std::llround(t * 1e9);
  f.cursor = integrated_until_;
  f.reason = reason;
  f.snapshot = observer_.snapshot();
  f.snapshot.frame_timestamp = t;
  f.snapshot.imu_timestamp = integrated_until_;
  f.snapshot.healthy_camera_updates = healthy_camera_updates_;
  f.snapshot.camera_substeps = corrected_current_ ? 1 : 0;
  f.snapshot.valid = observer_started_ && !suspended_ && healthy_camera_updates_ >= options_.observer.warmup_camera_updates;
  f.snapshot.velocity_valid = f.snapshot.valid && f.snapshot.velocity_body.allFinite();
  const double norm = f.snapshot.gravity_body.norm();
  f.snapshot.gravity_valid = f.snapshot.valid && f.snapshot.gravity_body.allFinite() && norm >= options_.observer.gravity_norm_min &&
                             norm <= options_.observer.gravity_norm_max;
  f.landmarks = landmark_history_.candidates();
  if (feature_frontend_) {
    f.mature_features = feature_frame_.management.mature_visible;
    const bool mature = feature_frame_.management.accepted_input && feature_frame_.management.enough_mature && !suspended_;
    f.ready_G = mature && f.snapshot.gravity_valid;
    f.ready_V = mature && f.snapshot.velocity_valid;
  } else {
    f.ready_G = !suspended_ && f.snapshot.gravity_valid;
    f.ready_V = !suspended_ && f.snapshot.velocity_valid;
  }
  if (availability_) {
    f.hardened = true;
    f.raw_current = observer_started_ && !suspended_ && std::isfinite(integrated_until_) && observer_.state().allFinite() &&
                    observer_.covariance().allFinite() && std::abs(f.snapshot.imu_timestamp - integrated_until_) <= 1e-9 &&
                    t == last_camera_time_;
    f.health = availability_status_;
    if (options_.hardening_health_readiness) {
      f.ready_G = f.raw_current && availability_status_.ready_G;
      f.ready_V = f.raw_current && availability_status_.ready_V;
    }
    f.prediction_angle_p95_rad = prediction_angle_;
    f.velocity_correction_rate = velocity_correction_rate_;
    f.gravity_correction_rate = gravity_correction_rate_;
    f.correction_diagnostics_valid = correction_diagnostics_valid_;
    f.actual_corrections = actual_corrections_;
  }
  f.integrated_steps = integrated_imu_steps_;
  f.integrated_seconds = integrated_imu_seconds_;
  return f;
}
LtvFrame LtvRuntime::pause(double t, const std::string &reason, uint64_t version) { return pauseManaged(t, reason, version); }

void LtvRuntime::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_.clear();
  observer_.reset();
  observer_started_ = false;
  healthy_camera_updates_ = 0;
  corrected_current_ = false;
  feature_to_observer_id_.clear();
  landmark_history_.reset();
  next_observer_id_ = 0;
  suspended_ = true;
  last_camera_time_ = -1;
  imu_input_fault_ = false;
  ++epoch_;
  if (feature_frontend_)
    feature_frontend_->reset(epoch_);
  feature_frame_ = ltv::LtvFeatureFrame();
  if (availability_) {
    availability_->reset();
    availability_status_ = ltv::LtvAvailabilityOutput();
    new_epoch_pending_ = false;
    initial_warm_attempted_ = false;
    last_correction_time_ = -1;
    actual_corrections_ = 0;
    correction_diagnostics_valid_ = false;
  }
  last_claimed_sequence_ = sequence_;
}
ov_core::ImuData LtvRuntime::correct(const ov_core::ImuData &raw, const LtvCalibration &c, const Eigen::Vector3d &ba,
                                     const Eigen::Vector3d &bg) {
  ov_core::ImuData x;
  x.timestamp = raw.timestamp;
  x.am = c.R_ACCtoIMU * c.Da * (raw.am - ba);
  x.wm = c.R_GYROtoIMU * c.Dw * (raw.wm - bg - c.Tg * x.am);
  return x;
}
LtvFrame LtvRuntime::process(double t, const std::vector<LtvBearing> &bearings, const LtvCalibration &c, const Eigen::Vector3d &ba,
                             const Eigen::Vector3d &bg, uint64_t version, const ltv::LtvFeatureContext *feature_context) {
  const auto begin = std::chrono::steady_clock::now();
  auto result = processManaged(t, bearings, c, ba, bg, version, feature_context);
  result.compute_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return result;
}

bool LtvRuntime::claim(const LtvFrame &f) {
  if (!f.available || suspended_ || f.epoch != epoch_ || f.sequence != sequence_ || f.sequence <= last_claimed_sequence_ ||
      f.camera_ns != std::llround(last_camera_time_ * 1e9) || f.version != last_version_ || f.camera_time != last_camera_time_ ||
      f.imu_time != integrated_until_)
    return false;
  last_claimed_sequence_ = f.sequence;
  return true;
}

void LtvRuntime::suspendManaged(double target) {
  // Keep the current event's x/P and lifecycle trace available for diagnostics.
  // The next event replaces the complete epoch before any further propagation.
  integrated_until_ = target;
  new_epoch_pending_ = true;
}
LtvFrame LtvRuntime::pauseManaged(double t, const std::string &reason, uint64_t version) {
  ltv::LtvAvailabilityInput input;
  input.time = t + camera_imu_offset_;
  // Startup/clone collection and an explicit ZUPT pause are availability
  // conditions, not evidence of a physical input fault. They must not bypass
  // the quality-bootstrap cooldown.
  input.physical_input_fault = reason == "invalid_or_out_of_order_imu" || reason == "missing_imu_bracket" || reason == "imu_bracket_gap" ||
                               reason == "camera_gap" || reason == "imu_gap" || reason == "out_of_order_camera" ||
                               reason == "propagation_failed";
  availability_status_ = availability_->update(input);
  observer_.reset();
  landmark_history_.reset();
  observer_started_ = false;
  healthy_camera_updates_ = 0;
  corrected_current_ = false;
  suspended_ = true;
  new_epoch_pending_ = true;
  feature_to_observer_id_.clear();
  feature_frame_ = ltv::LtvFeatureFrame();
  correction_diagnostics_valid_ = false;
  last_camera_time_ = std::max(last_camera_time_, t);
  auto f = frame(t, version, reason);
  f.raw_current = f.ready_G = f.ready_V = false;
  return f;
}
LtvFrame LtvRuntime::processManaged(double t, const std::vector<LtvBearing> &, const LtvCalibration &c, const Eigen::Vector3d &ba,
                                    const Eigen::Vector3d &bg, uint64_t version, const ltv::LtvFeatureContext *feature_context) {
  if (!std::isfinite(t) || !std::isfinite(c.offset) || !ba.allFinite() || !bg.allFinite() || !c.Da.allFinite() || !c.Dw.allFinite() ||
      !c.Tg.allFinite() || !c.R_ACCtoIMU.allFinite() || !c.R_GYROtoIMU.allFinite() || !c.R_BC.allFinite() || !c.p_BC.allFinite())
    throw std::invalid_argument("non-finite hardened context");
  const double target = t + c.offset;
  camera_imu_offset_ = c.offset;
  if (!feature_context || !std::isfinite(feature_context->time) || std::abs(feature_context->time - target) > 1e-9 ||
      feature_context->version != version)
    throw std::invalid_argument("missing or stale hardened feature context");
  const auto &history_limits = options_.feature_manager.history;
  if (feature_context->observations.size() > history_limits.max_observations_per_packet ||
      feature_context->cameras.size() > history_limits.max_camera_count || feature_context->poses.size() > 32)
    throw std::invalid_argument("hardened context exceeds declared capacity");
  auto proper_rotation = [](const Eigen::Matrix3d &r) {
    return (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-8 && std::abs(r.determinant() - 1) < 1e-8;
  };
  if (!proper_rotation(c.R_BC) || !proper_rotation(c.R_ACCtoIMU) || !proper_rotation(c.R_GYROtoIMU) || feature_context->cameras.empty() ||
      (feature_context->cameras[0].R_BC - c.R_BC).norm() > 1e-9 || (feature_context->cameras[0].p_BC - c.p_BC).norm() > 1e-9)
    throw std::invalid_argument("inconsistent or invalid hardened calibration");
  if (t <= last_camera_time_) {
    auto f = frame(t, version, "duplicate_or_backward_camera");
    f.raw_current = f.ready_G = f.ready_V = false;
    // Reject only this return value; a duplicate must not consume policy state.
    f.health.accepted_time = f.health.observer_valid = false;
    f.health.ready_G = f.health.ready_V = f.health.joint_ready = false;
    f.health.grace_G = f.health.grace_V = false;
    f.health.reason = "duplicate_or_backward_camera";
    return f;
  }
  // Stage 1: collect one causal IMU interval, retaining the shared boundary.
  last_camera_time_ = t;
  last_version_ = version;
  std::vector<ov_core::ImuData> samples;
  bool bracket_gap = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_input_fault_) {
      imu_input_fault_ = false;
      return pause(t, "invalid_or_out_of_order_imu", version);
    }
    const double start = suspended_ ? target : integrated_until_;
    if (imu_.size() < 2 || imu_.front().timestamp > start || imu_.back().timestamp < target)
      return pause(t, "missing_imu_bracket", version);
    auto interpolate = [&](double time) {
      auto hi = std::lower_bound(imu_.begin(), imu_.end(), time, [](const ov_core::ImuData &x, double y) { return x.timestamp < y; });
      if (hi == imu_.end())
        throw std::runtime_error("LTV bracket lost");
      if (hi->timestamp == time)
        return *hi;
      auto lo = std::prev(hi);
      if (hi->timestamp - lo->timestamp > options_.observer.max_imu_dt)
        bracket_gap = true;
      double fraction = (time - lo->timestamp) / (hi->timestamp - lo->timestamp);
      ov_core::ImuData x;
      x.timestamp = time;
      x.am = (1 - fraction) * lo->am + fraction * hi->am;
      x.wm = (1 - fraction) * lo->wm + fraction * hi->wm;
      return x;
    };
    samples.push_back(interpolate(start));
    for (const auto &x : imu_)
      if (x.timestamp > start && x.timestamp < target)
        samples.push_back(x);
    if (target > start)
      samples.push_back(interpolate(target));
    // Keep the shared left boundary. Never integrate the cached right sample early.
    while (imu_.size() > 2 && imu_[1].timestamp <= target)
      imu_.pop_front();
  }
  if (bracket_gap)
    return pause(t, "imu_bracket_gap", version);
  if (!suspended_ && (target <= integrated_until_ || target - integrated_until_ > options_.observer.reset_gap))
    return pause(t, "camera_gap", version);
  for (size_t i = 1; i < samples.size(); ++i)
    if (samples[i].timestamp - samples[i - 1].timestamp > options_.observer.max_imu_dt)
      return pause(t, "imu_gap", version);
  // Stage 2: start a new epoch only at the explicit runtime boundary.
  if (suspended_ || new_epoch_pending_) {
    ++epoch_;
    observer_.reset();
    observer_started_ = false;
    healthy_camera_updates_ = 0;
    corrected_current_ = false;
    feature_frontend_->reset(epoch_);
    feature_frame_ = ltv::LtvFeatureFrame();
    feature_to_observer_id_.clear();
    next_observer_id_ = 0;
    last_correction_time_ = -1;
    actual_corrections_ = 0;
    correction_diagnostics_valid_ = false;
    integrated_until_ = target;
    suspended_ = false;
    new_epoch_pending_ = false;
  }
  if (options_.hardening_initial_warmup && !initial_warm_attempted_ && availability_status_.bootstrap_count == 0 &&
      availability_status_.physical_fault_count == 0) {
    // R2 development intervention: retain the baseline's first causal zero
    // start. Readiness acknowledges this complete state at this camera event;
    // it does not imply ready, and its quality cooldown applies immediately.
    initial_warm_attempted_ = true;
    observer_.reset();
    observer_started_ = true;
  }
  // Stage 3: propagate the mathematical observer to this camera time.
  if (observer_started_) {
    for (size_t i = 1; i < samples.size(); ++i) {
      const auto a = correct(samples[i - 1], c, ba, bg), b = correct(samples[i], c, ba, bg);
      const double dt = b.timestamp - a.timestamp;
      if (!observer_.propagateImu(dt, .5 * (a.am + b.am), .5 * (a.wm + b.wm)))
        return pauseManaged(t, "core_imu_failure", version);
      ++integrated_imu_steps_;
      integrated_imu_seconds_ += dt;
    }
  }
  integrated_until_ = target;
  auto context = *feature_context;
  context.epoch = epoch_;
  if (options_.enable_landmark_approx)
    landmark_history_.before(*this, context, c);
  // Stage 4: prepare geometry, lifecycle and IDs without committing live state.
  // Stage manager, IDs and full x/P together. A failed transaction cannot
  // consume a seed in the live manager or leave half a core lifecycle event.
  const bool cold = !observer_started_;
  auto evaluated_frontend = *feature_frontend_;
  auto evaluated_frame = evaluated_frontend.process(context, !cold);
  if (!evaluated_frame.management.accepted_input)
    throw std::runtime_error("hardened feature context rejected: " + evaluated_frame.management.reason);
  ltv::LtvAvailabilityInput input;
  input.time = target;
  input.observer_time = target;
  input.eligible_seeds = evaluated_frame.management.eligible_seeds;
  input.geometry_valid = evaluated_frame.management.accepted_input;
  if (cold) {
    availability_status_ = availability_->update(input);
    if (!availability_status_.request_bootstrap) {
      *feature_frontend_ = std::move(evaluated_frontend);
      feature_frame_ = std::move(evaluated_frame);
      ++sequence_;
      auto f = frame(t, version, availability_status_.reason);
      f.imu_time = target;
      return f;
    }
  }
  auto staged_frontend = cold ? *feature_frontend_ : std::move(evaluated_frontend);
  auto staged_frame = cold ? staged_frontend.process(context, true) : std::move(evaluated_frame);
  // Stage 5: apply controlled births and camera correction on a copy.
  auto staged_observer = observer_;
  auto staged_ids = feature_to_observer_id_;
  int staged_next = next_observer_id_;
  if (cold)
    staged_observer.reset();
  ltv::LtvControlledFeatures control;
  control.epoch = epoch_;
  control.imu_timestamp = target;
  std::vector<ltv::LtvFeatureObservation> features;
  std::vector<double> angles;
  for (const auto &o : staged_frame.management.observations) {
    if (o.camera_id != 0)
      continue;
    if (!o.bearing.allFinite() || o.bearing.norm() < 1e-12)
      throw std::runtime_error("invalid managed bearing");
    if (!staged_ids.count(o.feature_id)) {
      if (staged_next == INT_MAX)
        throw std::runtime_error("controlled ID overflow");
      staged_ids.emplace(o.feature_id, staged_next++);
    }
    const int id = staged_ids.at(o.feature_id);
    ltv::LtvFeatureObservation f;
    f.feature_id = id;
    f.normalized_coordinate = o.bearing;
    features.push_back(f);
    const int slot = staged_observer.slotForFeature(id);
    if (slot >= 0) {
      Eigen::Vector3d ray = staged_observer.state().segment<3>(3 * slot) - c.p_BC;
      if (ray.norm() >= .1 && ray.allFinite()) {
        const double dot = ray.normalized().dot((c.R_BC * o.bearing).normalized());
        angles.push_back(std::acos(std::max(-1., std::min(1., dot))));
      }
    }
  }
  for (size_t id : staged_frame.management.retained_ids)
    control.retained_ids.push_back(staged_ids.at(id));
  for (const auto &b : staged_frame.management.births) {
    if (b.seed.epoch != epoch_ || std::abs(b.seed.time - target) > 1e-9)
      throw std::runtime_error("stale bootstrap seed");
    ltv::LtvLandmarkSeed s;
    s.feature_id = staged_ids.at(b.feature_id);
    s.apply_mean = b.apply_seed;
    s.mean_body = b.seed.landmark_B;
    control.births.push_back(s);
  }
  const auto predicted = staged_observer.snapshot();
  const double dt = last_correction_time_ < 0 ? 0 : target - last_correction_time_;
  if (!staged_observer.synchronize(control) || !staged_observer.correct(dt, features, c.R_BC, c.p_BC))
    return pauseManaged(t, "core_camera_failure", version);
  const auto corrected = staged_observer.snapshot();
  corrected_current_ = dt > 0 && !features.empty();
  correction_diagnostics_valid_ = dt > 0 && angles.size() >= options_.hardening_readiness.min_features && corrected_current_;
  prediction_angle_ = velocity_correction_rate_ = gravity_correction_rate_ = 0;
  if (correction_diagnostics_valid_) {
    std::sort(angles.begin(), angles.end());
    prediction_angle_ = angles[static_cast<size_t>(std::ceil(.95 * angles.size())) - 1];
    velocity_correction_rate_ = (corrected.velocity_body - predicted.velocity_body).norm() / dt;
    gravity_correction_rate_ = (corrected.gravity_body - predicted.gravity_body).norm() / dt;
  }
  if (dt > 0 && corrected_current_ && features.size() >= options_.hardening_readiness.min_features)
    ++actual_corrections_;
  input.observer_started = true;
  input.observer_finite = staged_observer.state().allFinite() && staged_observer.covariance().allFinite();
  input.velocity = corrected.velocity_body;
  input.gravity = corrected.gravity_body;
  input.observed_features = features.size();
  input.mature_observed_features = staged_frame.management.mature_visible;
  input.actual_corrections = actual_corrections_;
  input.correction_diagnostics_valid = correction_diagnostics_valid_;
  input.prediction_angle_p95_rad = prediction_angle_;
  input.velocity_correction_rate = velocity_correction_rate_;
  input.gravity_correction_rate = gravity_correction_rate_;
  if (!cold)
    availability_status_ = availability_->update(input);
  if (cold || availability_status_.request_bootstrap) {
    input.bootstrap_was_physical_recovery = availability_status_.bootstrap_is_physical_recovery;
    input.bootstrap_source = cold ? "ZERO_DELAYED_START" : "BOUNDED_INITIAL_ZERO_WARMUP";
    availability_status_ = availability_->acknowledgeBootstrap(input);
  }
  // Stage 6: commit only the complete accepted transaction.
  observer_ = std::move(staged_observer);
  observer_started_ = true;
  healthy_camera_updates_ = features.size() >= static_cast<size_t>(options_.observer.min_features) ? healthy_camera_updates_ + 1 : 0;
  *feature_frontend_ = std::move(staged_frontend);
  feature_frame_ = std::move(staged_frame);
  // Only retained slots need a mapping. Identity reuse is rejected upstream by
  // the manager; local IDs never repeat within this observer epoch.
  const std::set<size_t> retained(feature_frame_.management.retained_ids.begin(), feature_frame_.management.retained_ids.end());
  for (auto it = staged_ids.begin(); it != staged_ids.end();) {
    if (!retained.count(it->first))
      it = staged_ids.erase(it);
    else
      ++it;
  }
  feature_to_observer_id_ = std::move(staged_ids);
  next_observer_id_ = staged_next;
  last_correction_time_ = target;
  if (options_.enable_landmark_approx)
    landmark_history_.after(*this, context, c, epoch_);
  if (availability_status_.request_dormant)
    suspendManaged(target);
  ++sequence_;
  auto f = frame(t, version, availability_status_.reason);
  f.imu_time = target;
  f.available = true;
  return f;
}
} // namespace ov_msckf
