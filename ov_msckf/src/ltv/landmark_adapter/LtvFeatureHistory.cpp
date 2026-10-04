#include "ltv/landmark_adapter/LtvFeatureHistory.h"
#include <cmath>
#include <stdexcept>
namespace ltv {
LtvFeatureHistory::LtvFeatureHistory(const FeatureHistoryConfig &config) : config_(config) {
  if (!config.max_candidates || !std::isfinite(config.history_window) || config.history_window <= 0 ||
      !std::isfinite(config.candidate_ttl) || config.candidate_ttl <= 0)
    throw std::invalid_argument("invalid LTV history configuration");
  if (config.bounded_memory && (!config.max_samples_per_track || !config.max_observations_per_packet || !config.max_camera_count))
    throw std::invalid_argument("invalid bounded LTV history capacity");
}
void LtvFeatureHistory::reset(uint64_t epoch) {
  epoch_ = epoch;
  time_ = -1;
  tracks_.clear();
}
const FeatureTrackHistory *LtvFeatureHistory::find(size_t id) const {
  auto it = tracks_.find(id);
  return it == tracks_.end() ? nullptr : &it->second;
}
bool LtvFeatureHistory::forget(size_t id) { return tracks_.erase(id) != 0; }
bool LtvFeatureHistory::update(uint64_t epoch, double time, uint64_t version, const std::vector<HistoryObservation> &observations,
                               const std::set<size_t> &protected_ids) {
  // Caller explicitly resets an epoch; stale/future epochs never clear live history.
  if (epoch != epoch_ || !std::isfinite(time) || time < 0 || time <= time_)
    return false;
  if (config_.bounded_memory &&
      (observations.size() > config_.max_observations_per_packet || protected_ids.size() > config_.max_candidates))
    return false;
  std::map<size_t, std::vector<HistoryObservation>> grouped;
  std::set<std::pair<size_t, int>> unique;
  for (const auto &o : observations) {
    if (o.camera_id < 0 || (config_.bounded_memory && static_cast<size_t>(o.camera_id) >= config_.max_camera_count) ||
        !o.bearing.allFinite() || o.bearing.norm() < 1e-12 || !unique.emplace(o.feature_id, o.camera_id).second)
      return false;
    auto normalized = o;
    normalized.bearing.normalize();
    grouped[o.feature_id].push_back(normalized);
  }
  // Preflight before mutating time/visibility: a protected identity may not
  // bypass the total bound. Reserve its slot before admitting ordinary inputs.
  size_t pending_protected = 0;
  if (config_.bounded_memory) {
    size_t survivors = 0;
    for (const auto &entry : tracks_)
      if (protected_ids.count(entry.first) || time - entry.second.last_time <= config_.candidate_ttl)
        ++survivors;
    for (const auto &entry : grouped)
      if (protected_ids.count(entry.first) && !tracks_.count(entry.first))
        ++pending_protected;
    if (survivors > config_.max_candidates || pending_protected > config_.max_candidates - survivors)
      return false;
  }
  std::set<size_t> previously_visible;
  for (const auto &entry : tracks_)
    if (entry.second.visible)
      previously_visible.insert(entry.first);
  time_ = time;
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    it->second.visible = false;
    if (!protected_ids.count(it->first) && time - it->second.last_time > config_.candidate_ttl)
      it = tracks_.erase(it);
    else
      ++it;
  }
  for (const auto &entry : grouped) {
    auto it = tracks_.find(entry.first);
    if (it == tracks_.end()) {
      if (config_.bounded_memory) {
        if (!protected_ids.count(entry.first) && tracks_.size() + pending_protected >= config_.max_candidates)
          continue;
        if (protected_ids.count(entry.first))
          --pending_protected;
      }
      if (tracks_.size() >= config_.max_candidates && !protected_ids.count(entry.first))
        continue;
      FeatureTrackHistory h;
      h.first_time = time;
      it = tracks_.emplace(entry.first, h).first;
    }
    auto &h = it->second;
    // A missing frame breaks a candidate's contiguous history. ACTIVE lifecycle
    // retention is handled separately; no synthetic observation fills the gap.
    if (!h.samples.empty() && !previously_visible.count(entry.first) && !protected_ids.count(entry.first)) {
      h.samples.clear();
      h.first_time = time;
      h.total_frames = 0;
      h.identity_valid = true;
    }
    h.last_time = time;
    h.visible = true;
    ++h.total_frames;
    for (const auto &o : entry.second)
      h.identity_valid = h.identity_valid && o.match_valid;
    if (config_.bounded_memory)
      while (h.samples.size() >= config_.max_samples_per_track)
        h.samples.pop_front();
    h.samples.push_back({time, version, entry.second});
    while (!h.samples.empty() && time - h.samples.front().time > config_.history_window)
      h.samples.pop_front();
  }
  return true;
}
} // namespace ltv
