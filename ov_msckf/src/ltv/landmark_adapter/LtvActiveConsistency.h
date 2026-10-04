#pragma once
#include "ltv/landmark_adapter/LtvFeatureHistory.h"
#include <limits>
#include <string>

namespace ltv {
struct FeaturePipelineContext;
struct ActiveConsistencyConfig {
  bool enabled = false;
  size_t min_history_frames = 5;
  double min_history_span_s = .20;
  double max_history_residual_rad = .04, max_holdout_residual_rad = .04;
  int retire_after_consecutive_failures = 2;
  // Select from the existing history; never create a second observation store.
  double history_window_s = 1.0;
};
enum class ActiveConsistencyReason {
  Disabled,
  InsufficientHistory,
  MissingPoseSupport,
  FitFailed,
  HistoryInconsistent,
  HoldoutInconsistent,
  Pass
};
const char *toString(ActiveConsistencyReason reason);
struct ActiveConsistencyResult {
  size_t feature_id = 0;
  bool evaluable = false, pass = true;
  size_t history_count = 0, missing_pose_count = 0;
  double history_span_s = 0;
  double history_max_residual_rad = std::numeric_limits<double>::quiet_NaN();
  double holdout_residual_rad = std::numeric_limits<double>::quiet_NaN();
  bool fit_valid = false;
  Eigen::Vector3d point_W = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double fit_condition = std::numeric_limits<double>::quiet_NaN();
  std::string fit_reason;
  ActiveConsistencyReason reason = ActiveConsistencyReason::Disabled;
};
// Pure geometry only. Caller provides history from the current context epoch;
// FeatureTrackHistory itself has no epoch tag. No state/P or failure-count writes.
class LtvActiveConsistency {
public:
  explicit LtvActiveConsistency(const ActiveConsistencyConfig &config = {});
  ActiveConsistencyResult evaluate(size_t feature_id, double current_time, const FeatureTrackHistory &history,
                                   const FeaturePipelineContext &ctx) const;

private:
  ActiveConsistencyConfig config_;
};
} // namespace ltv
