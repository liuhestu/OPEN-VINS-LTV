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
LtvLandmarkManager::LtvLandmarkManager(const LandmarkManagerConfig &c)
    : config_(c), history_([&c]() {
        auto h = c.history;
        h.bounded_memory = c.bounded_memory;
        return h;
      }()),
      quality_(c.quality) {
  if (!c.max_active || c.max_active > 30 || c.min_features < 15 || c.min_features > c.max_active || c.max_missed_frames < 0 ||
      !std::isfinite(c.maturity_seconds) || c.maturity_seconds < 0 || (c.bounded_memory && c.history.max_candidates < c.max_active))
    throw std::invalid_argument("invalid LTV manager configuration");
  if (c.bounded_memory)
    identity_guard_.emplace_back();
}
void LtvLandmarkManager::reset(uint64_t epoch) {
  if (config_.bounded_memory)
    identity_guard_.front().reset(epoch);
  opportunity_total_ = admitted_total_ = active_retired_total_ = candidate_ttl_total_ = guard_rejections_total_ = 0;
  epoch_ = epoch;
  history_.reset(epoch);
  landmarks_.clear();
  retained_.clear();
  retired_.clear();
}
LandmarkManagerFrame LtvLandmarkManager::step(uint64_t epoch, double time, uint64_t version,
                                              const std::vector<HistoryObservation> &observations,
                                              const std::vector<FeatureSeedCandidate> &candidates, bool allow_admission) {
  LandmarkManagerFrame out;
  out.epoch = epoch;
  out.time = time;
  if (epoch != epoch_) {
    out.reason = "epoch_mismatch";
    return out;
  }
  if (config_.bounded_memory && candidates.size() > config_.history.max_observations_per_packet) {
    out.reason = "seed_candidate_packet_limit";
    return out;
  }
  std::map<size_t, const FeatureSeedCandidate *> seeds;
  for (const auto &s : candidates) {
    if (!seeds.emplace(s.feature_id, &s).second) {
      out.reason = "duplicate_seed_candidate";
      return out;
    }
  }
  std::vector<HistoryObservation> filtered;
  std::set<size_t> expired_candidates;
  const auto *effective_observations = &observations;
  if (config_.bounded_memory) {
    // Validate the whole original packet; a guarded ID cannot hide bad input.
    if (observations.size() > config_.history.max_observations_per_packet) {
      out.reason = "invalid_history_input";
      return out;
    }
    // Capture expiry against the previous history before update can erase and
    // recreate a same-ID observation after a long camera gap. No mutation until
    // the entire history input has been accepted below.
    for (const auto &entry : landmarks_) {
      if (retained_.count(entry.first) || entry.second.entered >= 0)
        continue;
      const auto *previous = history_.find(entry.first);
      if (!previous || time - previous->last_time > config_.history.candidate_ttl)
        expired_candidates.insert(entry.first);
    }
    std::set<std::pair<size_t, int>> unique;
    std::set<size_t> rejected;
    for (const auto &o : observations) {
      if (o.camera_id < 0 || static_cast<size_t>(o.camera_id) >= config_.history.max_camera_count || !o.bearing.allFinite() ||
          o.bearing.norm() < 1e-12 || !unique.emplace(o.feature_id, o.camera_id).second) {
        out.reason = "invalid_history_input";
        return out;
      }
      if (expired_candidates.count(o.feature_id) ||
          (!landmarks_.count(o.feature_id) && identity_guard_.front().potentiallySeen(o.feature_id))) {
        rejected.insert(o.feature_id);
      } else {
        filtered.push_back(o);
      }
    }
    out.identity_guard_rejected_ids.assign(rejected.begin(), rejected.end());
    effective_observations = &filtered;
  }
  // Historical cache occupancy is input-driven, independent of admission guard.
  // Retired/rejected observations retain the legacy ordering and TTL competition.
  if (!history_.update(epoch, time, version, observations, retained_)) {
    out.reason = "invalid_history_input";
    return out;
  }
  out.accepted_input = true;
  if (config_.bounded_memory)
    guard_rejections_total_ += out.identity_guard_rejected_ids.size();
  out.reason = "processed";
  std::map<size_t, HistoryObservation> visible;
  for (const auto &o : *effective_observations)
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
      if (config_.bounded_memory) {
        out.retirement_events.push_back({*it, LandmarkRetirementKind::ActiveRetired, time, m});
        ++active_retired_total_;
      } else {
        retired_.insert(*it);
      }
      out.retired_ids.push_back(*it);
      it = retained_.erase(it);
    } else
      ++it;
  }
  if (config_.bounded_memory) {
    for (auto it = landmarks_.begin(); it != landmarks_.end();) {
      if (retained_.count(it->first)) {
        ++it;
        continue;
      }
      if (it->second.phase == LandmarkPhase::Retired || expired_candidates.count(it->first) || !history_.find(it->first)) {
        if (it->second.entered < 0) {
          it->second.phase = LandmarkPhase::Retired;
          it->second.reason = "candidate_ttl";
          identity_guard_.front().insert(it->first);
          ++candidate_ttl_total_;
          out.retirement_events.push_back({it->first, LandmarkRetirementKind::NeverAdmittedCandidateTtl, time, it->second});
        }
        it = landmarks_.erase(it);
      } else {
        ++it;
      }
    }
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
    if (config_.bounded_memory && !landmarks_.count(id) && identity_guard_.front().potentiallySeen(id))
      continue; // Includes an identity retired earlier in this same event.
    const auto *h = history_.find(id);
    if (!h) {
      if (config_.bounded_memory)
        out.capacity_rejected_ids.push_back(id);
      continue;
    }
    if (config_.bounded_memory && !landmarks_.count(id) && landmarks_.size() >= config_.history.max_candidates) {
      out.capacity_rejected_ids.push_back(id);
      continue;
    }
    auto inserted = landmarks_.emplace(id, ManagedLandmark{});
    auto &m = inserted.first->second;
    if (inserted.second)
      m.first_seen = time;
    m.last_seen = time;
    const auto si = seeds.find(id);
    const auto *s = si == seeds.end() ? nullptr : si->second;
    const auto q = quality_.evaluate(epoch, time, *h, s);
    if (config_.bounded_memory && q.opportunity && !m.ever_opportunity)
      ++opportunity_total_;
    m.ever_opportunity = m.ever_opportunity || q.opportunity;
    m.reason = q.reason;
    if (q.accepted) {
      m.phase = LandmarkPhase::SeedReady;
      ready.push_back({id, s, sector(entry.second.bearing), q.risk});
    } else
      m.phase = LandmarkPhase::Candidate;
  }
  out.eligible_seeds = ready.size();
  std::array<size_t, 4> sectors{{0, 0, 0, 0}};
  for (size_t id : retained_)
    if (visible.count(id))
      ++sectors[sector(visible.at(id).bearing)];
  // Existing states never get displaced to improve image distribution. New points
  // prefer sparsely occupied quadrants, then lower risk, then deterministic ID.
  while (allow_admission && retained_.size() < config_.max_active && !ready.empty()) {
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
    if (config_.bounded_memory) {
      identity_guard_.front().insert(best->id);
      ++admitted_total_;
    }
    ++sectors[best->sector];
    out.births.push_back({best->id, config_.apply_seed, *best->seed});
    ready.erase(best);
  }
  for (const auto &r : ready)
    landmarks_.at(r.id).reason = allow_admission ? "capacity" : "waiting_observer_start";
  // Expired candidates keep lightweight lifetime/denominator records but no image
  // history. Admitted retired identities remain tombstoned until explicit reset.
  if (!config_.bounded_memory)
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
  if (config_.bounded_memory) {
    out.opportunity_tracks = opportunity_total_;
    out.admitted_tracks = admitted_total_;
    out.active_retired_total = active_retired_total_;
    out.candidate_ttl_total = candidate_ttl_total_;
    out.identity_guard_rejections_total = guard_rejections_total_;
    out.identity_guard_insertions = identity_guard_.front().insertionCalls();
    out.identity_guard_set_bits = identity_guard_.front().setBitCount();
    out.exact_records = landmarks_.size();
  }
  out.enough_mature = out.mature_visible >= config_.min_features;
  return out;
}
} // namespace ltv
