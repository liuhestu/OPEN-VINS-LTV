#include "LtvLandmarkManager.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace ltv {
const char *toString(LandmarkPhase s) {
  switch (s) {
  case LandmarkPhase::Candidate:
    return "CANDIDATE";
  case LandmarkPhase::SeedReady:
    return "SEED_READY";
  case LandmarkPhase::Active:
    return "ACTIVE";
  case LandmarkPhase::Coasting:
    return "COASTING";
  case LandmarkPhase::Retired:
    return "RETIRED";
  }
  return "UNKNOWN";
}
LtvLandmarkManager::LtvLandmarkManager(const LandmarkManagerConfig &c) : config_(c), history_(c.history), quality_(c.quality) {
  if (!c.max_active || c.max_active > 30 || c.min_features < 15 || c.min_features > c.max_active || c.max_missed_frames < 0 ||
      !std::isfinite(c.maturity_seconds) || c.maturity_seconds < 0 || c.history.max_candidates < c.max_active)
    throw std::invalid_argument("invalid LTV manager configuration");
}
void LtvLandmarkManager::reset(uint64_t epoch) {
  epoch_ = epoch;
  history_.reset(epoch);
  landmarks_.clear();
  retained_.clear();
  retired_.clear();
}
LandmarkManagerFrame LtvLandmarkManager::step(uint64_t epoch, double time, uint64_t version,
                                              const std::vector<HistoryObservation> &observations,
                                              const std::vector<FeatureSeedCandidate> &candidates) {
  LandmarkManagerFrame out;
  out.epoch = epoch;
  out.time = time;
  if (epoch != epoch_) {
    out.reason = "epoch_mismatch";
    return out;
  }
  std::map<size_t, const FeatureSeedCandidate *> seeds;
  for (const auto &s : candidates) {
    if (!seeds.emplace(s.feature_id, &s).second) {
      out.reason = "duplicate_seed_candidate";
      return out;
    }
  }
  if (!history_.update(epoch, time, version, observations, retained_)) {
    out.reason = "invalid_history_input";
    return out;
  }
  out.accepted_input = true;
  out.reason = "processed";
  std::map<size_t, HistoryObservation> visible;
  for (const auto &o : observations)
    if (o.camera_id == 0)
      visible.emplace(o.feature_id, o);
  for (auto it = retained_.begin(); it != retained_.end();) {
    auto &m = landmarks_.at(*it);
    auto v = visible.find(*it);
    if (v != visible.end() && v->second.match_valid) {
      m.phase = LandmarkPhase::Active;
      m.missed_frames = 0;
      m.last_seen = time;
    } else {
      ++m.missed_frames;
      m.phase = LandmarkPhase::Coasting;
    }
    if (m.missed_frames > config_.max_missed_frames || (v != visible.end() && !v->second.match_valid)) {
      m.phase = LandmarkPhase::Retired;
      m.reason = "lost_or_identity_invalid";
      retired_.insert(*it);
      out.retired_ids.push_back(*it);
      it = retained_.erase(it);
    } else
      ++it;
  }
  struct Ready {
    size_t id;
    const FeatureSeedCandidate *seed;
    int sector;
    double risk;
  };
  std::vector<Ready> ready;
  auto sector = [](const Eigen::Vector3d &b) { return (b.x() >= 0 ? 1 : 0) + (b.y() >= 0 ? 2 : 0); };
  for (const auto &entry : visible) {
    const size_t id = entry.first;
    if (retired_.count(id) || retained_.count(id))
      continue;
    const auto *h = history_.find(id);
    if (!h)
      continue;
    auto inserted = landmarks_.emplace(id, ManagedLandmark{});
    auto &m = inserted.first->second;
    if (inserted.second)
      m.first_seen = time;
    m.last_seen = time;
    const auto si = seeds.find(id);
    const auto *s = si == seeds.end() ? nullptr : si->second;
    const auto q = quality_.evaluate(epoch, time, *h, s);
    m.ever_opportunity = m.ever_opportunity || q.opportunity;
    m.reason = q.reason;
    if (q.accepted) {
      m.phase = LandmarkPhase::SeedReady;
      ready.push_back({id, s, sector(entry.second.bearing), q.risk});
    } else
      m.phase = LandmarkPhase::Candidate;
  }
  std::array<size_t, 4> sectors{{0, 0, 0, 0}};
  for (size_t id : retained_)
    if (visible.count(id))
      ++sectors[sector(visible.at(id).bearing)];
  // Existing states never get displaced to improve image distribution. New points
  // prefer sparsely occupied quadrants, then lower risk, then deterministic ID.
  while (retained_.size() < config_.max_active && !ready.empty()) {
    auto best = std::min_element(ready.begin(), ready.end(), [&](const Ready &a, const Ready &b) {
      if (sectors[a.sector] != sectors[b.sector])
        return sectors[a.sector] < sectors[b.sector];
      if (a.risk != b.risk)
        return a.risk < b.risk;
      return a.id < b.id;
    });
    auto &m = landmarks_.at(best->id);
    m.phase = LandmarkPhase::Active;
    m.entered = time;
    m.reason = "admitted";
    m.seed_written = config_.apply_seed;
    m.seeded = config_.apply_seed ? time : -1;
    retained_.insert(best->id);
    ++sectors[best->sector];
    out.births.push_back({best->id, config_.apply_seed, *best->seed});
    ready.erase(best);
  }
  for (const auto &r : ready)
    landmarks_.at(r.id).reason = "capacity";
  // Expired candidates keep lightweight lifetime/denominator records but no image
  // history. Admitted retired identities remain tombstoned until explicit reset.
  for (auto &entry : landmarks_) {
    auto &m = entry.second;
    if (!retained_.count(entry.first) && !retired_.count(entry.first) && !history_.find(entry.first)) {
      m.phase = LandmarkPhase::Retired;
      m.reason = "candidate_ttl";
      retired_.insert(entry.first);
    }
    if (m.ever_opportunity)
      ++out.opportunity_tracks;
    if (m.entered >= 0)
      ++out.admitted_tracks;
  }
  for (size_t id : retained_) {
    out.retained_ids.push_back(id);
    if (visible.count(id) && visible.at(id).match_valid) {
      out.observations.push_back(visible.at(id));
      if (time - landmarks_.at(id).entered + 1e-12 >= config_.maturity_seconds)
        ++out.mature_visible;
    }
  }
  out.enough_mature = out.mature_visible >= config_.min_features;
  return out;
}
} // namespace ltv
