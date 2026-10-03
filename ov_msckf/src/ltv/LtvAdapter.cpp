#include "LtvAdapter.h"
#include <climits>
namespace ov_msckf {
LtvAdapter::LtvAdapter(const LtvOptions &options) : options_(options) {
  auto config = options.observer;
  config.enable = options.enabled;
  observer_.configure(config);
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
  f.integrated_steps = steps_;
  f.integrated_seconds = seconds_;
  return f;
}
LtvFrame LtvAdapter::pause(double t, const std::string &reason, uint64_t version) {
  if (!paused_) {
    if (observer_.started())
      observer_.reset();
    paused_ = true;
    ids_.clear();
    next_id_ = 0;
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
                             const Eigen::Vector3d &bg, uint64_t version) {
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
  std::vector<ltv::LtvFeatureObservation> features;
  for (const auto &bearing : bearings) {
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
  observer_.updateFeatures(t, target, features, c.R_BC, c.p_BC);
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
