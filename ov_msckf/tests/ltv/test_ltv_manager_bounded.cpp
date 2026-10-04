#include "ltv/LtvLandmarkManager.h"
#include <iostream>
#include <stdexcept>
using namespace ltv;
static void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
static std::vector<HistoryObservation> observations(size_t first, size_t count) {
  std::vector<HistoryObservation> result;
  for (size_t id = first; id < first + count; ++id) {
    HistoryObservation o;
    o.feature_id = id;
    o.camera_id = 0;
    o.bearing = Eigen::Vector3d(.1, .1, 1).normalized();
    result.push_back(o);
  }
  return result;
}
static std::vector<FeatureSeedCandidate> seeds(uint64_t epoch, double time, size_t first, size_t count) {
  std::vector<FeatureSeedCandidate> result;
  for (size_t id = first; id < first + count; ++id) {
    FeatureSeedCandidate s;
    s.feature_id = id;
    s.epoch = epoch;
    s.time = time;
    s.geometry_valid = true;
    s.landmark_B = Eigen::Vector3d(.3, .3, 3);
    s.min_ray = 3;
    s.condition = 10;
    s.parallax_rad = .1;
    s.relative_risk = .01;
    result.push_back(s);
  }
  return result;
}
int main() {
  try {
    LandmarkManagerConfig config;
    config.bounded_memory = true;
    config.history.max_candidates = 30;
    config.max_active = 15;
    LtvLandmarkManager manager(config);
    manager.reset(1);
    LandmarkManagerFrame f;
    for (int k = 0; k < 3; ++k)
      f = manager.step(1, k * .05, k, observations(0, 40), seeds(1, k * .05, 0, 15));
    require(f.births.size() == 15 && f.exact_records == 30 && f.identity_guard_insertions == 15,
            "bounded live candidates, admitted identities protected");
    require(f.capacity_rejected_ids.size() == 10, "overflow has explicit receipt");
    // A copied staged transaction has independent identity protection and maps.
    auto staged = manager;
    for (int k = 3; k <= 5; ++k)
      f = staged.step(1, k * .05, k, {}, {});
    require(f.retirement_events.size() == 15 && f.active_retired_total == 15 && f.candidate_ttl_total == 0,
            "admitted removal is typed separately from candidate TTL");
    require(manager.landmarks().at(0).phase == LandmarkPhase::Active, "staging cannot mutate original manager");
    f = staged.step(1, 2.2, 6, {}, {});
    require(f.candidate_ttl_total == 15 && f.active_retired_total == 15 && f.exact_records == 0,
            "expired candidates erased with independent cumulative count");
    require(f.identity_guard_insertions == 30 && f.retirement_events.size() == 15, "overflow IDs never enter identity guard");
    f = staged.step(1, 2.25, 7, observations(0, 40), seeds(1, 2.25, 0, 40));
    require(f.identity_guard_rejected_ids.size() == 30 && f.births.empty() && f.exact_records == 10,
            "retired IDs rejected but previously unaccepted capacity IDs may try again");
    for (int k = 1; k <= 2; ++k) {
      const double t = 2.25 + .05 * k;
      f = staged.step(1, t, 7 + k, observations(30, 10), seeds(1, t, 30, 10));
    }
    require(f.births.size() == 10 && f.admitted_tracks == 25 && f.identity_guard_insertions == 40,
            "capacity retry admits once with exact cumulative counts");
    const auto before = f.identity_guard_insertions;
    require(!staged.step(1, 2.4, 10, {}, seeds(1, 2.4, 1000, config.history.max_observations_per_packet + 1)).accepted_input,
            "oversized candidate vector rejected before map construction");
    auto invalid = observations(100, 1);
    invalid.push_back(invalid.front());
    require(!staged.step(1, 2.4, 10, invalid, {}).accepted_input, "invalid packet rejected before mutation");
    f = staged.step(1, 2.4, 10, observations(30, 10), seeds(1, 2.4, 30, 10));
    require(f.accepted_input && f.identity_guard_insertions == before && f.births.empty(), "rejection consumes neither time nor bits");
    bool rejected = false;
    try {
      staged.reset(1);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "same populated epoch cannot clear protection");
    staged.reset(2);
    for (int k = 0; k < 3; ++k)
      f = staged.step(2, k * .05, k, observations(0, 15), seeds(2, k * .05, 0, 15));
    require(f.births.size() == 15 && f.admitted_tracks == 15 && f.active_retired_total == 0, "explicit new epoch permits one seed again");
    // Large nonmonotonic IDs do not raise a frontend-ID high water mark.
    LtvLandmarkManager nonmonotonic(config);
    nonmonotonic.reset(1);
    for (int k = 0; k < 3; ++k)
      nonmonotonic.step(1, k * .05, k, observations(100000, 1), seeds(1, k * .05, 100000, 1));
    for (int k = 3; k < 6; ++k)
      f = nonmonotonic.step(1, k * .05, k, observations(1, 1), seeds(1, k * .05, 1, 1));
    require(f.births.size() == 1 && f.births.front().feature_id == 1, "legal low frontend ID remains admissible");
    LtvLandmarkManager gap_expiry(config);
    gap_expiry.reset(1);
    f = gap_expiry.step(1, 0, 0, observations(77, 1), {});
    require(f.exact_records == 1 && f.identity_guard_insertions == 0, "unadmitted candidate initially cached");
    auto invalid_return = observations(77, 1);
    invalid_return.push_back(invalid_return.front());
    require(!gap_expiry.step(1, 2.1, 1, invalid_return, {}).accepted_input, "invalid returning packet cannot consume candidate TTL");
    require(gap_expiry.landmarks().size() == 1 && gap_expiry.history().find(77)->last_time == 0,
            "failed gap packet preserves exact record and previous history time");
    f = gap_expiry.step(1, 2.1, 1, observations(77, 1), seeds(1, 2.1, 77, 1));
    require(f.accepted_input && f.candidate_ttl_total == 1 && f.active_retired_total == 0 && f.retirement_events.size() == 1 &&
                f.retirement_events.front().kind == LandmarkRetirementKind::NeverAdmittedCandidateTtl,
            "same-event returning expired candidate emits permanent typed TTL");
    require(f.identity_guard_insertions == 1 && f.identity_guard_rejected_ids == std::vector<size_t>{77} && f.exact_records == 0 &&
                !gap_expiry.history().find(77) && f.births.empty(),
            "long-gap return cannot recreate expired identity history");
    f = gap_expiry.step(1, 2.15, 2, observations(77, 1), {});
    require(f.retirement_events.empty() && f.identity_guard_insertions == 1 && f.candidate_ttl_total == 1 && f.exact_records == 0,
            "TTL event and guard insertion occur exactly once");
    // Ten minutes of candidate turnover remains finite; no full observer simulation.
    LtvLandmarkManager churn(config);
    churn.reset(1);
    for (int k = 0; k < 12000; ++k) {
      f = churn.step(1, k * .05, k, observations(1000 + 30 * k, 30), {});
      require(f.accepted_input && churn.landmarks().size() <= 30 && f.identity_guard_insertions <= 30u * (k + 1),
              "candidate turnover never accumulates exact tombstones");
    }
    require(f.candidate_ttl_total > 0 && f.active_retired_total == 0, "churn never confuses unadmitted TTL with active retirement");
    std::cout << "bounded manager PASS typed retirement/identity guard/capacity retry/copy/epoch/nonmonotonic/10min churn\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
