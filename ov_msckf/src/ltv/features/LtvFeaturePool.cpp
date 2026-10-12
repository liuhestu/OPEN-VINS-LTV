#include "ltv/features/LtvFeaturePool.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// 1. Maintain causal observation history.
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

// 2. Evaluate seed admission quality.
namespace ltv {
const char *toString(FeatureSeedSource s) {
  switch (s) {
  case FeatureSeedSource::TemporalPose:
    return "TEMPORAL_POSE";
  case FeatureSeedSource::Stereo:
    return "STEREO";
  case FeatureSeedSource::MsckfGeometry:
    return "MSCKF_GEOMETRY";
  case FeatureSeedSource::StereoThenTemporal:
    return "STEREO_THEN_TEMPORAL";
  }
  return "UNKNOWN";
}
LtvFeatureQuality::LtvFeatureQuality(const FeatureQualityConfig &c) : config_(c) {
  if (!c.min_history_frames || !(c.min_history_span >= 0) || !(c.min_ray > 0) || !(c.max_condition > 0) || !(c.min_parallax_rad >= 0) ||
      !(c.max_residual_rad > 0) || !(c.max_relative_risk > 0))
    throw std::invalid_argument("invalid feature quality configuration");
}
FeatureQualityDecision LtvFeatureQuality::evaluate(uint64_t epoch, double time, const FeatureTrackHistory &h,
                                                   const FeatureSeedCandidate *s) const {
  FeatureQualityDecision d;
  auto reject = [&](const char *reason) {
    d.reason = reason;
    return d;
  };
  if (!h.visible || h.last_time != time)
    return reject("not_visible");
  if (!h.identity_valid)
    return reject("identity_invalid");
  if (h.samples.size() < config_.min_history_frames || h.samples.empty() ||
      time - h.samples.front().time + 1e-12 < config_.min_history_span)
    return reject("history_insufficient");
  // Opportunity denominator is fixed before the risk gate, including solver failure.
  d.opportunity = true;
  if (!s)
    return reject("seed_unavailable");
  if (s->epoch != epoch || s->time != time)
    return reject("seed_stale");
  if (!s->geometry_valid || !s->landmark_B.allFinite() || !std::isfinite(s->min_ray) || !std::isfinite(s->condition) ||
      !std::isfinite(s->parallax_rad) || !std::isfinite(s->residual_rad) || !std::isfinite(s->relative_risk))
    return reject("geometry_invalid");
  d.risk = s->relative_risk;
  if (s->min_ray < config_.min_ray)
    return reject("nonpositive_or_near_ray");
  if (s->condition < 1 || s->condition > config_.max_condition)
    return reject("condition");
  if (s->parallax_rad < config_.min_parallax_rad)
    return reject("parallax");
  if (s->residual_rad < 0 || s->residual_rad > config_.max_residual_rad)
    return reject("angular_residual");
  if (s->relative_risk < 0 || s->relative_risk > config_.max_relative_risk)
    return reject("distance_risk");
  if (config_.require_independent_check && !s->independent_check_available)
    return reject("independent_check_missing");
  if (s->independent_check_available &&
      (!std::isfinite(s->check_residual_rad) || s->check_residual_rad < 0 || s->check_residual_rad > config_.max_residual_rad))
    return reject("independent_check");
  d.accepted = true;
  d.reason = "accepted";
  return d;
}
} // namespace ltv

// 3. Commit feature births, retention and retirement.
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
LtvFeaturePool::LtvFeaturePool(const LtvFeatureManagerConfig &c)
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
void LtvFeaturePool::reset(uint64_t epoch) {
  if (config_.bounded_memory)
    identity_guard_.front().reset(epoch);
  opportunity_total_ = admitted_total_ = active_retired_total_ = candidate_ttl_total_ = guard_rejections_total_ = 0;
  epoch_ = epoch;
  history_.reset(epoch);
  landmarks_.clear();
  retained_.clear();
  retired_.clear();
}
LandmarkManagerFrame LtvFeaturePool::step(uint64_t epoch, double time, uint64_t version,
                                          const std::vector<HistoryObservation> &observations,
                                          const std::vector<FeatureSeedCandidate> &candidates, bool allow_admission,
                                          const std::map<size_t, ActiveConsistencyResult> *consistency, int retire_after) {
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
  if (consistency) {
    if (retire_after < 1) {
      out.reason = "invalid_consistency_config";
      return out;
    }
    for (const auto &entry : *consistency) {
      const auto &r = entry.second;
      if (entry.first != r.feature_id || !retained_.count(entry.first) || (!r.evaluable && !r.pass) ||
          (r.evaluable && (!std::isfinite(r.history_max_residual_rad) || (r.pass && !std::isfinite(r.holdout_residual_rad))))) {
        out.reason = "invalid_consistency_input";
        return out;
      }
    }
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
  std::set<size_t> consistency_skipped;
  out.active_consistency_enabled = consistency != nullptr;
  out.consistency_active_count = retained_.size();
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
    bool consistency_retire = false;
    if (consistency && v != visible.end() && v->second.match_valid) {
      const auto found = consistency->find(*it);
      if (found != consistency->end()) {
        const auto &r = found->second;
        if (r.evaluable) {
          ++out.consistency_evaluable_count;
          if (r.pass) {
            m.consistency_fail_count = 0;
            ++out.consistency_pass_count;
          } else {
            ++m.consistency_fail_count;
            ++m.consistency_reject_count;
            consistency_skipped.insert(*it);
            consistency_retire = m.consistency_fail_count >= retire_after;
            if (consistency_retire)
              ++out.consistency_retire_count;
            else
              ++out.consistency_skip_count;
          }
        }
        out.active_consistency.push_back({r, m.consistency_fail_count, m.consistency_reject_count,
                                          consistency_retire               ? "RETIRE"
                                          : consistency_skipped.count(*it) ? "SKIP"
                                                                           : "UPDATE"});
      }
    }
    if (m.missed_frames > config_.max_missed_frames || (v != visible.end() && !v->second.match_valid) || consistency_retire) {
      m.phase = LandmarkPhase::Retired;
      m.reason = consistency_retire ? "active_consistency" : "lost_or_identity_invalid";
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
    if (visible.count(id) && visible.at(id).match_valid && !consistency_skipped.count(id)) {
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
