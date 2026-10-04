#include "ltv/landmark_adapter/LtvLandmarkManager.h"
#include <iostream>
#include <stdexcept>
using namespace ltv;
static void require(bool b, const char *s) {
  if (!b)
    throw std::runtime_error(s);
}
static std::vector<HistoryObservation> observations(size_t first) {
  std::vector<HistoryObservation> v;
  for (size_t id = first; id < first + 15; ++id) {
    HistoryObservation o;
    o.feature_id = id;
    o.camera_id = 0;
    o.bearing = Eigen::Vector3d(.1, .1, 1).normalized();
    v.push_back(o);
  }
  return v;
}
static std::vector<FeatureSeedCandidate> seeds(double t, size_t first) {
  std::vector<FeatureSeedCandidate> v;
  for (size_t id = first; id < first + 15; ++id) {
    FeatureSeedCandidate s;
    s.feature_id = id;
    s.epoch = 1;
    s.time = t;
    s.geometry_valid = true;
    s.landmark_B = Eigen::Vector3d(.3, .3, 3);
    s.min_ray = 3;
    s.condition = 10;
    s.parallax_rad = .1;
    s.relative_risk = .01;
    v.push_back(s);
  }
  return v;
}
int main() {
  try {
    LandmarkManagerConfig c;
    c.max_active = c.history.max_candidates = 15;
    LtvLandmarkManager old(c);
    c.bounded_memory = true;
    LtvLandmarkManager bounded(c);
    old.reset(1);
    bounded.reset(1);
    auto paired = [&](double t, const std::vector<HistoryObservation> &obs, const std::vector<FeatureSeedCandidate> &seed) {
      auto a = old.step(1, t, 1, obs, seed), b = bounded.step(1, t, 1, obs, seed);
      require(a.accepted_input && b.accepted_input, "both packets accepted");
      require(a.retained_ids == b.retained_ids && a.births.size() == b.births.size(), "same controlled lifecycle selection");
      for (size_t j = 0; j < a.births.size(); ++j)
        require(a.births[j].feature_id == b.births[j].feature_id, "same birth ordering");
      const auto &x = old.history().tracks();
      const auto &y = bounded.history().tracks();
      require(x.size() == y.size(), "retirement must not release historical capacity early");
      for (const auto &entry : x) {
        auto it = y.find(entry.first);
        require(it != y.end(), "same historical capacity competitors");
        require(entry.second.first_time == it->second.first_time && entry.second.last_time == it->second.last_time &&
                    entry.second.samples.size() == it->second.samples.size(),
                "same causal history timing and sample count");
      }
      return b;
    };
    for (int k = 0; k < 3; ++k)
      paired(.05 * k, observations(0), seeds(.05 * k, 0));
    for (int k = 3; k < 6; ++k)
      paired(.05 * k, {}, {});
    require(bounded.landmarks().empty(), "retired exact manager records still removed");
    paired(.30, observations(100), seeds(.30, 100));
    require(!bounded.history().find(100), "new candidate cannot bypass old cached retired occupancy");
    auto returning = paired(.35, observations(0), seeds(.35, 0));
    require(returning.births.empty() && returning.identity_guard_rejected_ids.size() == 15,
            "retired input history may refresh but cannot revive seed");
    paired(2.40, observations(100), seeds(2.40, 100));
    paired(2.45, observations(100), seeds(2.45, 100));
    auto admitted = paired(2.50, observations(100), seeds(2.50, 100));
    require(admitted.births.size() == 15, "new candidates enter normally after original history TTL expires");
    std::cout << "history competition PASS old/bounded parity with retired input history\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
