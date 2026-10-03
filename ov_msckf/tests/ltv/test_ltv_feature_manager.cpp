#include "ltv/LtvLandmarkManager.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace ltv;
static void check(bool value, const char *name) {
  if (!value)
    throw std::runtime_error(name);
}
static std::vector<HistoryObservation> observations(size_t first = 0, size_t count = 16) {
  std::vector<HistoryObservation> result;
  for (size_t id = first; id < first + count; ++id) {
    HistoryObservation o;
    o.feature_id = id;
    o.bearing = Eigen::Vector3d(0.1 * (id % 3), 0.2, 1);
    result.push_back(o);
  }
  return result;
}
static std::vector<FeatureSeedCandidate> seeds(double time, uint64_t epoch = 0, size_t first = 0, size_t count = 16) {
  std::vector<FeatureSeedCandidate> result;
  for (size_t id = first; id < first + count; ++id) {
    FeatureSeedCandidate s;
    s.feature_id = id;
    s.time = time;
    s.epoch = epoch;
    s.geometry_valid = true;
    s.landmark_B = Eigen::Vector3d(1, 2, 10);
    s.min_ray = 10;
    s.condition = 100;
    s.parallax_rad = 0.1;
    s.residual_rad = 0.001;
    s.relative_risk = 0.02;
    result.push_back(s);
  }
  return result;
}
int main() {
  try {
    LtvLandmarkManager m;
    check(m.step(0, 0, 1, observations(), seeds(0)).births.empty(), "new candidates cannot pollute observer");
    check(m.step(0, .05, 2, observations(), seeds(.05)).births.empty(), "minimum causal history");
    auto bad = seeds(.1);
    for (auto &s : bad)
      s.relative_risk = .5;
    const auto rejected = m.step(0, .1, 3, observations(), bad);
    check(rejected.births.empty() && rejected.opportunity_tracks == 16, "risk rejection preserves denominator");
    auto accepted = m.step(0, .15, 4, observations(), seeds(.15));
    check(accepted.births.size() == 16 && accepted.retained_ids.size() == 16 && !accepted.enough_mature, "seed admission before maturity");
    check(!m.step(0, .15, 4, observations(), seeds(.15)).accepted_input, "duplicate event rejected");
    check(!m.step(1, .16, 4, observations(), seeds(.16, 1)).accepted_input, "stale epoch cannot reset manager");
    auto repeat = m.step(0, .25, 5, observations(), seeds(.25));
    check(repeat.births.empty() && repeat.enough_mature, "exactly once and mature");
    auto coast = m.step(0, .3, 6, {}, {});
    check(coast.retained_ids.size() == 16 && coast.observations.empty() && !coast.enough_mature, "coasting no fabricated bearing");
    auto back = m.step(0, .35, 7, observations(), seeds(.35));
    check(back.births.empty() && back.observations.size() == 16, "retained reappearance uses old state");
    m.step(0, .4, 8, {}, {});
    m.step(0, .45, 9, {}, {});
    auto retired = m.step(0, .5, 10, {}, {});
    check(retired.retained_ids.empty() && retired.retired_ids.size() == 16, "third missing frame retires");
    m.step(0, .55, 11, observations(), seeds(.55));
    m.step(0, .6, 12, observations(), seeds(.6));
    check(m.step(0, .65, 13, observations(), seeds(.65)).births.empty(), "retired ID not resurrected");
    m.reset(1);
    m.step(1, 0, 1, observations(), seeds(0, 1));
    m.step(1, .05, 2, observations(), seeds(.05, 1));
    check(m.step(1, .1, 3, observations(), seeds(.1, 1)).births.size() == 16, "explicit epoch reset permits new seed");
    LtvLandmarkManager capped;
    auto many = observations(0, 40);
    capped.step(0, 0, 1, many, seeds(0, 0, 0, 40));
    capped.step(0, .05, 2, many, seeds(.05, 0, 0, 40));
    check(capped.step(0, .1, 3, many, seeds(.1, 0, 0, 40)).births.size() == 30, "capacity capped");
    LandmarkManagerConfig cfg;
    cfg.apply_seed = false;
    LtvLandmarkManager sel(cfg);
    sel.step(0, 0, 1, observations(), seeds(0));
    sel.step(0, .05, 2, observations(), seeds(.05));
    auto selected = sel.step(0, .1, 3, observations(), seeds(.1));
    check(selected.births.size() == 16 && !selected.births.front().apply_seed, "SEL zero initialization is explicit");
    LtvFeatureHistory h;
    h.update(0, 0, 0, observations(0, 1));
    h.update(0, .05, 1, {});
    h.update(0, .1, 2, observations(0, 1));
    check(h.find(0)->samples.size() == 1 && h.find(0)->first_time == .1, "candidate continuity resets after disappearance");
    auto dupe = observations(0, 1);
    dupe.push_back(dupe.front());
    check(!h.update(0, .15, 3, dupe) && h.time() == .1, "duplicate camera observation rejected atomically");
    auto invalid = observations(0, 1);
    invalid.front().bearing.x() = std::numeric_limits<double>::quiet_NaN();
    check(!h.update(0, .15, 3, invalid) && h.time() == .1, "nonfinite rejected atomically");
    bool min_guard = false;
    try {
      LandmarkManagerConfig c;
      c.min_features = 1;
      LtvLandmarkManager invalid_manager(c);
    } catch (const std::invalid_argument &) {
      min_guard = true;
    }
    check(min_guard, "cannot lower minimum features");
    std::cout << "feature_manager PASS: causal history, once seed, coasting, retirement, epochs, capacity, quality denominator\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
