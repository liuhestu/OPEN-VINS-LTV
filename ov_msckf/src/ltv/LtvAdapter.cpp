#include "LtvAdapter.h"
#include <climits>
namespace ov_msckf {
LtvAdapter::LtvAdapter(const LtvOptions &options) : options_(options) {
  if (options.passive_hardening_enabled) {
    if (!options.feature_readiness_enabled || !options.enabled || options.enable_gravity || options.enable_velocity)
      throw std::invalid_argument("Hardening requires managed Passive LTV");
    auto health_config = options.hardening_readiness;
    health_config.enabled = true;
    health_config.initial_unseeded_warmup = options.hardening_initial_warmup;
    readiness_.reset(new ltv::LtvReadiness(health_config));
  }
  auto config = options.observer;
  config.enable = options.enabled;
  observer_.configure(config);
  if (options.feature_readiness_enabled) {
    if (!options.enabled || options.enable_gravity || options.enable_velocity)
      throw std::invalid_argument("Feature readiness requires passive observer");
    ltv::FeaturePipelineConfig pipeline;
    pipeline.manager = options.feature_manager;
    pipeline.manager.apply_seed = options.feature_apply_seed;
    if (options.feature_seed_source == "STEREO")
      pipeline.source = ltv::FeatureSeedSource::Stereo;
    else if (options.feature_seed_source == "STEREO_THEN_TEMPORAL")
      pipeline.source = ltv::FeatureSeedSource::StereoThenTemporal;
    else if (options.feature_seed_source == "TEMPORAL_POSE")
      pipeline.source = ltv::FeatureSeedSource::TemporalPose;
    else
      throw std::invalid_argument("Unsupported feature seed source");
    pipeline.bearing_sigma_rad = options.feature_bearing_sigma_rad;
    feature_pipeline_.reset(new ltv::LtvFeaturePipeline(pipeline));
  }
}
void LtvAdapter::feed_imu(const ov_core::ImuData &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(sample.timestamp) || !sample.am.allFinite() || !sample.wm.allFinite()) {
    input_fault_ = true;
    return;
  }
  if (!imu_.empty() && sample.timestamp <= imu_.back().timestamp) {
    input_fault_ = true;
    return;
  }
  imu_.push_back(sample);
  while (imu_.size() > 2 && imu_[1].timestamp < sample.timestamp - 10.0)
    imu_.pop_front();
}
LtvFrame LtvAdapter::frame(double t, uint64_t version, const std::string &reason) const {
  LtvFrame f;
  f.epoch = epoch_;
  f.version = version;
  f.sequence = sequence_;
  f.camera_time = t;
  f.camera_ns = std::llround(t * 1e9);
  f.cursor = cursor_;
  f.reason = reason;
  f.snapshot = observer_.snapshot(t);
  if (feature_pipeline_) {
    f.mature_features = feature_frame_.management.mature_visible;
    const bool mature = feature_frame_.management.accepted_input && feature_frame_.management.enough_mature && !paused_;
    f.ready_G = mature && f.snapshot.gravity_valid;
    f.ready_V = mature && f.snapshot.velocity_valid;
  } else {
    f.ready_G = !paused_ && f.snapshot.gravity_valid;
    f.ready_V = !paused_ && f.snapshot.velocity_valid;
  }
  if (readiness_) {
    f.hardened = true;
    f.raw_current = observer_.started() && !paused_ && std::isfinite(cursor_) && observer_.state().allFinite() &&
                    observer_.covariance().allFinite() && std::abs(f.snapshot.imu_timestamp - cursor_) <= 1e-9 && t == last_camera_;
    f.health = health_;
    if (options_.hardening_health_readiness) {
      f.ready_G = f.raw_current && health_.ready_G;
      f.ready_V = f.raw_current && health_.ready_V;
    }
    f.prediction_angle_p95_rad = prediction_angle_;
    f.velocity_correction_rate = velocity_correction_rate_;
    f.gravity_correction_rate = gravity_correction_rate_;
    f.correction_diagnostics_valid = correction_diagnostics_valid_;
    f.actual_corrections = actual_corrections_;
  }
  f.integrated_steps = steps_;
  f.integrated_seconds = seconds_;
  return f;
}
LtvFrame LtvAdapter::pause(double t, const std::string &reason, uint64_t version) {
  if (readiness_)
    return pauseHardened(t, reason, version);
  if (!paused_) {
    if (observer_.started())
      observer_.reset();
    paused_ = true;
    ids_.clear();
    next_id_ = 0;
    feature_frame_ = ltv::FeaturePipelineFrame();
  }
  last_camera_ = std::max(last_camera_, t);
  return frame(t, version, reason);
}
void LtvAdapter::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_.clear();
  observer_.reset();
  ids_.clear();
  next_id_ = 0;
  paused_ = true;
  last_camera_ = -1;
  input_fault_ = false;
  ++epoch_;
  if (feature_pipeline_)
    feature_pipeline_->reset(epoch_);
  feature_frame_ = ltv::FeaturePipelineFrame();
  if (readiness_) {
    readiness_->reset();
    health_ = ltv::LtvReadinessOutput();
    hardening_new_epoch_pending_ = false;
    initial_warm_attempted_ = false;
    last_correction_time_ = -1;
    actual_corrections_ = 0;
    correction_diagnostics_valid_ = false;
  }
  claimed_ = sequence_;
}
ov_core::ImuData LtvAdapter::correct(const ov_core::ImuData &raw, const LtvCalibration &c, const Eigen::Vector3d &ba,
                                     const Eigen::Vector3d &bg) {
  ov_core::ImuData x;
  x.timestamp = raw.timestamp;
  x.am = c.R_ACCtoIMU * c.Da * (raw.am - ba);
  x.wm = c.R_GYROtoIMU * c.Dw * (raw.wm - bg - c.Tg * x.am);
  return x;
}
LtvFrame LtvAdapter::process(double t, const std::vector<LtvBearing> &bearings, const LtvCalibration &c, const Eigen::Vector3d &ba,
                             const Eigen::Vector3d &bg, uint64_t version, const ltv::FeaturePipelineContext *feature_context) {
  if (readiness_)
    return processHardened(t, bearings, c, ba, bg, version, feature_context);
  if (feature_pipeline_ && (options_.enable_gravity || options_.enable_velocity))
    throw std::runtime_error("Passive feature path cannot submit G/V");
  if (feature_pipeline_ && (!feature_context || !std::isfinite(feature_context->time) ||
                            std::abs(feature_context->time - (t + c.offset)) > 1e-9 || feature_context->version != version))
    throw std::invalid_argument("Missing or stale feature context");
  if (!std::isfinite(t) || !std::isfinite(c.offset) || !ba.allFinite() || !bg.allFinite() || !c.Da.allFinite() || !c.Dw.allFinite() ||
      !c.Tg.allFinite() || !c.R_ACCtoIMU.allFinite() || !c.R_GYROtoIMU.allFinite() || !c.R_BC.allFinite() || !c.p_BC.allFinite())
    throw std::invalid_argument("non-finite LTV context");
  if (t <= last_camera_)
    return frame(t, version, "duplicate_or_backward_camera");
  last_camera_ = t;
  last_version_ = version;
  const double target = t + c.offset;
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
  if (paused_) {
    ++epoch_;
    observer_.start(target);
    if (feature_pipeline_) {
      feature_pipeline_->reset(epoch_);
      feature_frame_ = ltv::FeaturePipelineFrame();
      if (!observer_.enableControlledFeatures(epoch_))
        throw std::runtime_error("Cannot enable controlled feature admission");
    }
    cursor_ = target;
    paused_ = false;
    ids_.clear();
    next_id_ = 0;
  }
  for (size_t i = 1; i < samples.size(); ++i) {
    auto a = correct(samples[i - 1], c, ba, bg), b = correct(samples[i], c, ba, bg);
    double dt = b.timestamp - a.timestamp;
    observer_.propagateImu(dt, 0.5 * (a.am + b.am), 0.5 * (a.wm + b.wm), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    ++steps_;
    seconds_ += dt;
    if (!observer_.started())
      return pause(t, "core_imu_reset", version);
  }
  cursor_ = target;
  std::vector<LtvBearing> managed_bearings;
  ltv::LtvControlledFeatures control;
  if (feature_pipeline_) {
    auto context = *feature_context;
    context.epoch = epoch_;
    feature_frame_ = feature_pipeline_->process(context);
    if (!feature_frame_.management.accepted_input)
      throw std::runtime_error("Invalid feature pipeline context: " + feature_frame_.management.reason);
    for (const auto &o : feature_frame_.management.observations)
      if (o.camera_id == 0)
        managed_bearings.push_back({o.feature_id, o.bearing});
    control.epoch = epoch_;
    control.imu_timestamp = target;
  }
  std::vector<ltv::LtvFeatureObservation> features;
  for (const auto &bearing : feature_pipeline_ ? managed_bearings : bearings) {
    if (!bearing.coordinate.allFinite() || bearing.coordinate.norm() < 1e-12)
      return pause(t, "invalid_bearing", version);
    if (!ids_.count(bearing.id)) {
      if (next_id_ == INT_MAX)
        return pause(t, "feature_id_overflow", version);
      ids_.emplace(bearing.id, next_id_++);
    }
    ltv::LtvFeatureObservation f;
    f.feature_id = ids_.at(bearing.id);
    f.normalized_coordinate = bearing.coordinate;
    features.push_back(f);
  }
  if (feature_pipeline_) {
    for (size_t id : feature_frame_.management.retained_ids)
      control.retained_ids.push_back(ids_.at(id));
    for (const auto &birth : feature_frame_.management.births) {
      if (birth.seed.epoch != epoch_ || std::abs(birth.seed.time - target) > 1e-9)
        throw std::runtime_error("Stale seed execution context");
      ltv::LtvLandmarkSeed seed;
      seed.feature_id = ids_.at(birth.feature_id);
      seed.apply_mean = birth.apply_seed;
      seed.mean_body = birth.seed.landmark_B;
      control.births.push_back(seed);
    }
    const auto result = observer_.updateFeaturesControlled(t, target, features, c.R_BC, c.p_BC, control);
    if (!result.accepted && observer_.started())
      throw std::runtime_error("Controlled feature rejection: " + result.reason);
  } else {
    observer_.updateFeatures(t, target, features, c.R_BC, c.p_BC);
  }
  if (!observer_.started())
    return pause(t, "core_camera_reset", version);
  ++sequence_;
  auto f = frame(t, version, "candidate");
  f.imu_time = target;
  f.available = true;
  return f;
}
bool LtvAdapter::claim(const LtvFrame &f) {
  if (!f.available || paused_ || f.epoch != epoch_ || f.sequence != sequence_ || f.sequence <= claimed_ ||
      f.camera_ns != std::llround(last_camera_ * 1e9) || f.version != last_version_ || f.camera_time != last_camera_ ||
      f.imu_time != cursor_)
    return false;
  claimed_ = f.sequence;
  return true;
}
} // namespace ov_msckf
