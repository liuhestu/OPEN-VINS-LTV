#include "ltv/observer/LtvAdapter.h"
#include "ltv/observer/LtvMainCrossShadow.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <set>
namespace ov_msckf {
void LtvAdapter::suspendHardened(double target) {
  // Keep the current event's x/P and lifecycle trace available for diagnostics.
  // The next event replaces the complete epoch before any further propagation.
  cursor_ = target;
  hardening_new_epoch_pending_ = true;
}
LtvFrame LtvAdapter::pauseHardened(double t, const std::string &reason, uint64_t version) {
  ltv::LtvReadinessInput input;
  input.time = t + hardening_offset_;
  // Startup/clone collection and an explicit ZUPT pause are availability
  // conditions, not evidence of a physical input fault. They must not bypass
  // the quality-bootstrap cooldown.
  input.physical_input_fault = reason == "invalid_or_out_of_order_imu" || reason == "missing_imu_bracket" || reason == "imu_bracket_gap" ||
                               reason == "camera_gap" || reason == "imu_gap" || reason == "out_of_order_camera" ||
                               reason == "propagation_failed";
  health_ = readiness_->update(input);
  observer_.reset();
  paused_ = true;
  hardening_new_epoch_pending_ = true;
  ids_.clear();
  feature_frame_ = ltv::FeaturePipelineFrame();
  correction_diagnostics_valid_ = false;
  last_camera_ = std::max(last_camera_, t);
  auto f = frame(t, version, reason);
  f.raw_current = f.ready_G = f.ready_V = false;
  return f;
}
LtvFrame LtvAdapter::processHardened(double t, const std::vector<LtvBearing> &, const LtvCalibration &c, const Eigen::Vector3d &ba,
                                     const Eigen::Vector3d &bg, uint64_t version, const ltv::FeaturePipelineContext *feature_context) {
  if (!std::isfinite(t) || !std::isfinite(c.offset) || !ba.allFinite() || !bg.allFinite() || !c.Da.allFinite() || !c.Dw.allFinite() ||
      !c.Tg.allFinite() || !c.R_ACCtoIMU.allFinite() || !c.R_GYROtoIMU.allFinite() || !c.R_BC.allFinite() || !c.p_BC.allFinite())
    throw std::invalid_argument("non-finite hardened context");
  const double target = t + c.offset;
  hardening_offset_ = c.offset;
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
  if (t <= last_camera_) {
    auto f = frame(t, version, "duplicate_or_backward_camera");
    f.raw_current = f.ready_G = f.ready_V = false;
    // Reject only this return value; a duplicate must not consume policy state.
    f.health.accepted_time = f.health.observer_valid = false;
    f.health.ready_G = f.health.ready_V = f.health.joint_ready = false;
    f.health.grace_G = f.health.grace_V = false;
    f.health.reason = "duplicate_or_backward_camera";
    return f;
  }
  last_camera_ = t;
  last_version_ = version;
  std::vector<ov_core::ImuData> samples;
  bool bracket_gap = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (input_fault_) {
      input_fault_ = false;
      return pause(t, "invalid_or_out_of_order_imu", version);
    }
    const double start = paused_ ? target : cursor_;
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
  if (!paused_ && (target <= cursor_ || target - cursor_ > options_.observer.reset_gap))
    return pause(t, "camera_gap", version);
  for (size_t i = 1; i < samples.size(); ++i)
    if (samples[i].timestamp - samples[i - 1].timestamp > options_.observer.max_imu_dt)
      return pause(t, "imu_gap", version);
  if (paused_ || hardening_new_epoch_pending_) {
    ++epoch_;
    observer_.reset();
    feature_pipeline_->reset(epoch_);
    feature_frame_ = ltv::FeaturePipelineFrame();
    ids_.clear();
    next_id_ = 0;
    last_correction_time_ = -1;
    actual_corrections_ = 0;
    correction_diagnostics_valid_ = false;
    cursor_ = target;
    paused_ = false;
    hardening_new_epoch_pending_ = false;
  }
  if (options_.hardening_initial_warmup && !initial_warm_attempted_ && health_.bootstrap_count == 0 && health_.physical_fault_count == 0) {
    // R2 development intervention: retain the baseline's first causal zero
    // start. Readiness acknowledges this complete state at this camera event;
    // it does not imply ready, and its quality cooldown applies immediately.
    initial_warm_attempted_ = true;
    observer_.start(target);
    if (!observer_.enableControlledFeatures(epoch_, true))
      throw std::runtime_error("Cannot start bounded initial warmup");
  }
  if (observer_.started()) {
    for (size_t i = 1; i < samples.size(); ++i) {
      const auto a = correct(samples[i - 1], c, ba, bg), b = correct(samples[i], c, ba, bg);
      const double dt = b.timestamp - a.timestamp;
      observer_.propagateImu(dt, .5 * (a.am + b.am), .5 * (a.wm + b.wm), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      ++steps_;
      seconds_ += dt;
      if (!observer_.started())
        return pauseHardened(t, "core_imu_reset", version);
    }
  }
  cursor_ = target;
  auto context = *feature_context;
  context.epoch = epoch_;
  if (prediction_diagnostic_)
    prediction_diagnostic_(*this, context, c);
  // Stage manager, IDs and full x/P together. A failed transaction cannot
  // consume a seed in the live manager or leave half a core lifecycle event.
  auto pipeline_before = *feature_pipeline_;
  const bool cold = !observer_.started();
  auto evaluated_pipeline = pipeline_before;
  auto evaluated_frame = evaluated_pipeline.process(context, !cold);
  if (!evaluated_frame.management.accepted_input)
    throw std::runtime_error("hardened feature context rejected: " + evaluated_frame.management.reason);
  ltv::LtvReadinessInput input;
  input.time = target;
  input.observer_time = target;
  input.eligible_seeds = evaluated_frame.management.eligible_seeds;
  input.geometry_valid = evaluated_frame.management.accepted_input;
  if (cold) {
    health_ = readiness_->update(input);
    if (!health_.request_bootstrap) {
      *feature_pipeline_ = std::move(evaluated_pipeline);
      feature_frame_ = std::move(evaluated_frame);
      ++sequence_;
      auto f = frame(t, version, health_.reason);
      f.imu_time = target;
      return f;
    }
  }
  auto staged_pipeline = cold ? pipeline_before : evaluated_pipeline;
  auto staged_frame = cold ? staged_pipeline.process(context, true) : evaluated_frame;
  auto staged_core = observer_;
  auto staged_ids = ids_;
  int staged_next = next_id_;
  if (cold) {
    staged_core.start(target);
    if (!staged_core.enableControlledFeatures(epoch_, true))
      throw std::runtime_error("controlled bootstrap failed");
  }
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
    const int slot = staged_core.slotForFeature(id);
    if (slot >= 0) {
      Eigen::Vector3d ray = staged_core.state().segment<3>(3 * slot) - c.p_BC;
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
  const auto predicted = staged_core.snapshot(t);
  const auto result = staged_core.updateFeaturesControlled(t, target, features, c.R_BC, c.p_BC, control);
  if (!result.accepted || !staged_core.started()) {
    *feature_pipeline_ = pipeline_before;
    throw std::runtime_error("atomic controlled transaction rejected: " + result.reason);
  }
  if (ltv::LtvMainCrossShadow::instance().enabled()) {
    for (const auto &birth : staged_frame.management.births) {
      for (const auto &seed : staged_frame.seeds) {
        if (seed.candidate.feature_id != birth.feature_id || !birth.apply_seed || seed.main_jacobian.size() == 0)
          continue;
        // Bounded diagnostic seed/anchor bank; capacity is a declared model
        // boundary, never silent global source independence.
        if (ltv::LtvMainCrossShadow::instance().auxiliaryDimension() + 3 > 96) {
          ltv::LtvMainCrossShadow::instance().missing("seed_bank_capacity_96");
          continue;
        }
        const int nz = seed.bearing_jacobian.cols();
        const Eigen::MatrixXd q = seed.input_covariance.bottomRightCorner(nz, nz);
        ltv::LtvMainCrossShadow::instance().conditionalSeed(seed.main_jacobian, seed.bearing_jacobian, q);
      }
    }
  }
  const auto corrected = staged_core.snapshot(t);
  const double dt = last_correction_time_ < 0 ? 0 : target - last_correction_time_;
  correction_diagnostics_valid_ = dt > 0 && angles.size() >= options_.hardening_readiness.min_features && corrected.camera_substeps > 0;
  prediction_angle_ = velocity_correction_rate_ = gravity_correction_rate_ = 0;
  if (correction_diagnostics_valid_) {
    std::sort(angles.begin(), angles.end());
    prediction_angle_ = angles[static_cast<size_t>(std::ceil(.95 * angles.size())) - 1];
    velocity_correction_rate_ = (corrected.velocity_body - predicted.velocity_body).norm() / dt;
    gravity_correction_rate_ = (corrected.gravity_body - predicted.gravity_body).norm() / dt;
  }
  if (dt > 0 && corrected.camera_substeps > 0 && features.size() >= options_.hardening_readiness.min_features)
    ++actual_corrections_;
  input.observer_started = true;
  input.observer_finite = staged_core.state().allFinite() && staged_core.covariance().allFinite();
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
    health_ = readiness_->update(input);
  if (cold || health_.request_bootstrap) {
    input.bootstrap_was_physical_recovery = health_.bootstrap_is_physical_recovery;
    input.bootstrap_source = cold ? "ZERO_DELAYED_START" : "BOUNDED_INITIAL_ZERO_WARMUP";
    health_ = readiness_->acknowledgeBootstrap(input);
  }
  observer_ = std::move(staged_core);
  *feature_pipeline_ = std::move(staged_pipeline);
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
  ids_ = std::move(staged_ids);
  next_id_ = staged_next;
  last_correction_time_ = target;
  if (health_.request_dormant)
    suspendHardened(target);
  ++sequence_;
  auto f = frame(t, version, health_.reason);
  f.imu_time = target;
  f.available = true;
  return f;
}
} // namespace ov_msckf
