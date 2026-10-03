#pragma once
#include "LtvFeatureHistory.h"
#include <string>
namespace ltv {
enum class FeatureSeedSource { TemporalPose, Stereo, MsckfGeometry, StereoThenTemporal };
const char *toString(FeatureSeedSource source);
struct FeatureSeedCandidate {
  size_t feature_id = 0;
  uint64_t epoch = 0;
  double time = 0;
  Eigen::Vector3d landmark_B = Eigen::Vector3d::Zero();
  FeatureSeedSource source = FeatureSeedSource::TemporalPose;
  bool geometry_valid = false, independent_check_available = false;
  double min_ray = 0, condition = 0, parallax_rad = 0, residual_rad = 0;
  double relative_risk = 0, check_residual_rad = 0;
};
struct FeatureQualityConfig {
  size_t min_history_frames = 3;
  double min_history_span = 0.10;
  double min_ray = 0.1, max_condition = 1e8, min_parallax_rad = 0.008726646259971648;
  double max_residual_rad = 0.008726646259971648, max_relative_risk = 0.05;
  bool require_independent_check = false;
};
struct FeatureQualityDecision {
  bool opportunity = false, accepted = false;
  std::string reason;
  double risk = 0;
};
class LtvFeatureQuality {
public:
  explicit LtvFeatureQuality(const FeatureQualityConfig &config = {});
  FeatureQualityDecision evaluate(uint64_t epoch, double time, const FeatureTrackHistory &history,
                                  const FeatureSeedCandidate *candidate) const;

private:
  FeatureQualityConfig config_;
};
} // namespace ltv
