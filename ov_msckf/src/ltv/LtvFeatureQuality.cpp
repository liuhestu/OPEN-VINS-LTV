#include "LtvFeatureQuality.h"
#include <cmath>
#include <stdexcept>
namespace ltv {
const char *toString(FeatureSeedSource s) {
  switch (s) {
  case FeatureSeedSource::TemporalPose:
    return "TEMPORAL_POSE";
  case FeatureSeedSource::Stereo:
    return "STEREO";
  case FeatureSeedSource::MsckfGeometry:
    return "MSCKF_GEOMETRY";
  case FeatureSeedSource::StereoThenTemporal:
    return "STEREO_THEN_TEMPORAL";
  }
  return "UNKNOWN";
}
LtvFeatureQuality::LtvFeatureQuality(const FeatureQualityConfig &c) : config_(c) {
  if (!c.min_history_frames || !(c.min_history_span >= 0) || !(c.min_ray > 0) || !(c.max_condition > 0) || !(c.min_parallax_rad >= 0) ||
      !(c.max_residual_rad > 0) || !(c.max_relative_risk > 0))
    throw std::invalid_argument("invalid feature quality configuration");
}
FeatureQualityDecision LtvFeatureQuality::evaluate(uint64_t epoch, double time, const FeatureTrackHistory &h,
                                                   const FeatureSeedCandidate *s) const {
  FeatureQualityDecision d;
  auto reject = [&](const char *reason) {
    d.reason = reason;
    return d;
  };
  if (!h.visible || h.last_time != time)
    return reject("not_visible");
  if (!h.identity_valid)
    return reject("identity_invalid");
  if (h.samples.size() < config_.min_history_frames || h.samples.empty() ||
      time - h.samples.front().time + 1e-12 < config_.min_history_span)
    return reject("history_insufficient");
  // Opportunity denominator is fixed before the risk gate, including solver failure.
  d.opportunity = true;
  if (!s)
    return reject("seed_unavailable");
  if (s->epoch != epoch || s->time != time)
    return reject("seed_stale");
  if (!s->geometry_valid || !s->landmark_B.allFinite() || !std::isfinite(s->min_ray) || !std::isfinite(s->condition) ||
      !std::isfinite(s->parallax_rad) || !std::isfinite(s->residual_rad) || !std::isfinite(s->relative_risk))
    return reject("geometry_invalid");
  d.risk = s->relative_risk;
  if (s->min_ray < config_.min_ray)
    return reject("nonpositive_or_near_ray");
  if (s->condition < 1 || s->condition > config_.max_condition)
    return reject("condition");
  if (s->parallax_rad < config_.min_parallax_rad)
    return reject("parallax");
  if (s->residual_rad < 0 || s->residual_rad > config_.max_residual_rad)
    return reject("angular_residual");
  if (s->relative_risk < 0 || s->relative_risk > config_.max_relative_risk)
    return reject("distance_risk");
  if (config_.require_independent_check && !s->independent_check_available)
    return reject("independent_check_missing");
  if (s->independent_check_available &&
      (!std::isfinite(s->check_residual_rad) || s->check_residual_rad < 0 || s->check_residual_rad > config_.max_residual_rad))
    return reject("independent_check");
  d.accepted = true;
  d.reason = "accepted";
  return d;
}
} // namespace ltv
