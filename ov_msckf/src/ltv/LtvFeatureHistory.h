#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <vector>

namespace ltv {
struct HistoryObservation {
  size_t feature_id = 0;
  int camera_id = 0;
  Eigen::Vector3d bearing = Eigen::Vector3d::Zero();
  bool match_valid = true;
};
struct HistorySample {
  double time = 0;
  uint64_t context_version = 0;
  std::vector<HistoryObservation> observations;
};
struct FeatureTrackHistory {
  double first_time = 0, last_time = 0;
  size_t total_frames = 0;
  bool visible = false, identity_valid = true;
  std::deque<HistorySample> samples;
};
struct FeatureHistoryConfig {
  size_t max_candidates = 256;
  double history_window = 1.0, candidate_ttl = 2.0;
};
// Stores observations only. A caller must resolve all poses in one coherent context;
// history does not pretend means from different optimization versions share a joint P.
class LtvFeatureHistory {
public:
  explicit LtvFeatureHistory(const FeatureHistoryConfig &config = {});
  bool update(uint64_t epoch, double time, uint64_t context_version, const std::vector<HistoryObservation> &observations,
              const std::set<size_t> &protected_ids = {});
  void reset(uint64_t epoch);
  const std::map<size_t, FeatureTrackHistory> &tracks() const { return tracks_; }
  const FeatureTrackHistory *find(size_t id) const;
  uint64_t epoch() const { return epoch_; }
  double time() const { return time_; }

private:
  FeatureHistoryConfig config_;
  uint64_t epoch_ = 0;
  double time_ = -1;
  std::map<size_t, FeatureTrackHistory> tracks_;
};
} // namespace ltv
