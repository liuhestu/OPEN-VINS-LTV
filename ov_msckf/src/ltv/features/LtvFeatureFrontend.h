#pragma once
#include "ltv/features/LtvFeatureManager.h"
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

class LtvFeatureFrontend {
public:
  explicit LtvFeatureFrontend(const LtvFeatureConfig &config = {});
  void reset(uint64_t epoch);
  LtvFeatureFrame process(const LtvFeatureContext &context, bool allow_admission = true);
  const LtvFeatureManager &manager() const { return manager_; }

private:
  LtvFeatureConfig config_;
  LtvFeatureManager manager_;
  LtvActiveConsistency active_consistency_;
};
} // namespace ltv
