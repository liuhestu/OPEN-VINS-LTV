#pragma once
#include "ltv/features/LtvFeaturePool.h"
#include "ltv/features/LtvLandmarkHistory.h"
#include "ltv/features/LtvSeedGeometry.h"

namespace ltv {
// Geometry checks are evaluated before the lifecycle manager commits an event.
class LtvActiveConsistency {
public:
  explicit LtvActiveConsistency(const ActiveConsistencyConfig &config = {});
  ActiveConsistencyResult evaluate(size_t feature_id, double current_time, const FeatureTrackHistory &history,
                                   const LtvFeatureContext &ctx) const;

private:
  ActiveConsistencyConfig config_;
};

class LtvFeatureManager {
public:
  explicit LtvFeatureManager(const LtvFeatureConfig &config = {});
  void reset(uint64_t epoch);
  LtvFeatureFrame process(const LtvFeatureContext &context, bool allow_admission = true);
  const LtvFeaturePool &pool() const { return manager_; }
  const LtvFeatureFrame &frame() const { return frame_; }
  const std::map<size_t, int> &ids() const { return ids_; }
  const std::vector<LtvLandmarkCandidate> &candidates() const { return anchors_.candidates(); }
  struct ObserverInput {
    LtvControlledFeatures control;
    std::vector<LtvFeatureObservation> observations;
    std::vector<double> prediction_angles;
  };
  ObserverInput prepare(const LtvFeatureContext &, const LtvObserver &, bool ready_V, bool admit, bool landmarks);
  void commit(const LtvFeatureContext &, const LtvObserver &, bool landmarks);

private:
  LtvFeatureFrame frame_;
  std::map<size_t, int> ids_;
  int next_id_ = 0;
  LtvLandmarkHistory anchors_;
  LtvFeatureConfig config_;
  LtvFeaturePool manager_;
  LtvActiveConsistency active_consistency_;
};
} // namespace ltv
