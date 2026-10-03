#pragma once
#include "LtvFeatureQuality.h"
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
};
struct ManagedLandmark {
  LandmarkPhase phase = LandmarkPhase::Candidate;
  double first_seen = 0, entered = -1, seeded = -1, last_seen = 0;
  int missed_frames = 0;
  bool ever_opportunity = false, seed_written = false;
  std::string reason = "candidate";
};
struct LandmarkBirth {
  size_t feature_id = 0;
  bool apply_seed = true;
  FeatureSeedCandidate seed;
};
struct LandmarkManagerFrame {
  bool accepted_input = false;
  std::string reason;
  uint64_t epoch = 0;
  double time = 0;
  std::vector<size_t> retained_ids;
  std::vector<HistoryObservation> observations;
  std::vector<LandmarkBirth> births;
  std::vector<size_t> retired_ids;
  size_t mature_visible = 0, opportunity_tracks = 0, admitted_tracks = 0;
  bool enough_mature = false;
};
// Pure causal state machine; it cannot access observer x/P or modify the tracker.
// Returned births must be installed by the controlled core before correction.
class LtvLandmarkManager {
public:
  explicit LtvLandmarkManager(const LandmarkManagerConfig &config = {});
  void reset(uint64_t epoch);
  LandmarkManagerFrame step(uint64_t epoch, double time, uint64_t context_version, const std::vector<HistoryObservation> &observations,
                            const std::vector<FeatureSeedCandidate> &candidates);
  const LtvFeatureHistory &history() const { return history_; }
  const std::map<size_t, ManagedLandmark> &landmarks() const { return landmarks_; }

private:
  LandmarkManagerConfig config_;
  LtvFeatureHistory history_;
  LtvFeatureQuality quality_;
  std::map<size_t, ManagedLandmark> landmarks_;
  std::set<size_t> retained_, retired_;
  uint64_t epoch_ = 0;
};
} // namespace ltv
