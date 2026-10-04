#pragma once
#include "LtvActiveConsistency.h"
#include "LtvFeatureQuality.h"
#include "LtvIdentityGuard.h"
#include <map>
#include <set>
namespace ltv {
enum class LandmarkPhase { Candidate, SeedReady, Active, Coasting, Retired };
const char *toString(LandmarkPhase phase);
struct LandmarkManagerConfig {
  FeatureHistoryConfig history;
  FeatureQualityConfig quality;
  size_t max_active = 30, min_features = 15;
  int max_missed_frames = 2;
  double maturity_seconds = 0.10;
  bool apply_seed = true;
  bool bounded_memory = false;
};
struct ManagedLandmark {
  LandmarkPhase phase = LandmarkPhase::Candidate;
  double first_seen = 0, entered = -1, seeded = -1, last_seen = 0;
  int missed_frames = 0;
  int consistency_fail_count = 0;
  uint64_t consistency_reject_count = 0;
  bool ever_opportunity = false, seed_written = false;
  std::string reason = "candidate";
};
struct LandmarkBirth {
  size_t feature_id = 0;
  bool apply_seed = true;
  FeatureSeedCandidate seed;
};
enum class LandmarkRetirementKind { ActiveRetired, NeverAdmittedCandidateTtl };
struct LandmarkRetirement {
  size_t feature_id = 0;
  LandmarkRetirementKind kind = LandmarkRetirementKind::ActiveRetired;
  double time = 0;
  ManagedLandmark record;
};
struct ActiveConsistencyDiagnostic {
  ActiveConsistencyResult result;
  int fail_count = 0;
  uint64_t reject_count = 0;
  std::string action = "UPDATE";
};
struct LandmarkManagerFrame {
  bool active_consistency_enabled = false;
  size_t consistency_active_count = 0, consistency_evaluable_count = 0, consistency_pass_count = 0;
  size_t consistency_skip_count = 0, consistency_retire_count = 0;
  std::vector<ActiveConsistencyDiagnostic> active_consistency;
  bool accepted_input = false;
  std::string reason;
  uint64_t epoch = 0;
  double time = 0;
  std::vector<size_t> retained_ids;
  std::vector<HistoryObservation> observations;
  std::vector<LandmarkBirth> births;
  std::vector<size_t> retired_ids;
  size_t mature_visible = 0, opportunity_tracks = 0, admitted_tracks = 0, eligible_seeds = 0;
  bool enough_mature = false;
  // Bounded mode emits one-time typed transitions instead of storing tombstones.
  std::vector<LandmarkRetirement> retirement_events;
  std::vector<size_t> identity_guard_rejected_ids, capacity_rejected_ids;
  uint64_t active_retired_total = 0, candidate_ttl_total = 0, identity_guard_rejections_total = 0;
  uint64_t identity_guard_insertions = 0;
  size_t identity_guard_set_bits = 0, exact_records = 0;
};
// Pure causal state machine; it cannot access observer x/P or modify the tracker.
// Returned births must be installed by the controlled core before correction.
class LtvLandmarkManager {
public:
  explicit LtvLandmarkManager(const LandmarkManagerConfig &config = {});
  void reset(uint64_t epoch);
  LandmarkManagerFrame step(uint64_t epoch, double time, uint64_t context_version, const std::vector<HistoryObservation> &observations,
                            const std::vector<FeatureSeedCandidate> &candidates, bool allow_admission = true,
                            const std::map<size_t, ActiveConsistencyResult> *consistency = nullptr, int retire_after = 2);
  const LtvFeatureHistory &history() const { return history_; }
  const std::set<size_t> &retained_ids() const { return retained_; }
  const std::map<size_t, ManagedLandmark> &landmarks() const { return landmarks_; }

private:
  LandmarkManagerConfig config_;
  LtvFeatureHistory history_;
  LtvFeatureQuality quality_;
  std::map<size_t, ManagedLandmark> landmarks_;
  std::set<size_t> retained_, retired_;
  uint64_t epoch_ = 0;
  // Empty in legacy mode; vector copies provide independent transactional bits.
  std::vector<LtvIdentityGuard> identity_guard_;
  uint64_t opportunity_total_ = 0, admitted_total_ = 0, active_retired_total_ = 0, candidate_ttl_total_ = 0;
  uint64_t guard_rejections_total_ = 0;
};
} // namespace ltv
