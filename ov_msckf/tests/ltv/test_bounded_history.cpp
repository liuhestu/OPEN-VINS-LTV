#include "ltv/LtvFeatureHistory.h"
#include <cassert>
#include <cmath>
#include <iostream>

ltv::HistoryObservation observation(size_t id, int camera = 0) { return {id, camera, Eigen::Vector3d(0, 0, 1), true}; }

int main() {
  ltv::FeatureHistoryConfig config;
  config.bounded_memory = true;
  config.max_candidates = 3;
  config.max_samples_per_track = 21;
  config.max_observations_per_packet = 6;
  ltv::LtvFeatureHistory history(config);
  history.reset(1);
  // 10kHz updates within one second cannot grow the per-track sample window.
  for (int k = 0; k < 10000; ++k) {
    assert(history.update(1, k * .0001, k + 1, {observation(10), observation(10, 1)}));
    assert(history.find(10)->samples.size() <= 21);
    assert(history.tracks().size() == 1);
  }
  assert(history.find(10)->total_frames == 10000);
  const double old_time = history.time();
  const size_t old_samples = history.find(10)->samples.size();
  assert(!history.update(1, 1.0, 10001, std::vector<ltv::HistoryObservation>(7, observation(20))));
  assert(!history.update(1, 1.0, 10001, {observation(20, 2)}));
  assert(!history.update(1, 1.0, 10001, {observation(20)}, {1, 2, 3, 4}));
  assert(history.time() == old_time && history.find(10)->samples.size() == old_samples);
  assert(history.update(1, 1.0, 10001, {observation(1), observation(2), observation(3)}, {3}));
  assert(history.tracks().size() == 3 && history.find(3));
  // Full live table plus unknown protected identity rejects atomically.
  assert(!history.update(1, 1.1, 10002, {observation(99)}, {99}));
  assert(history.time() == 1.0 && !history.find(99));
  assert(history.forget(1));
  assert(!history.forget(1));
  assert(history.time() == 1.0 && history.epoch() == 1 && history.find(10));
  assert(history.update(1, 1.1, 10002, {observation(0), observation(99)}, {99}));
  assert(history.find(99) && !history.find(0) && history.tracks().size() == 3);
  // Explicit reset is the only operation here that clears all tracks/time.
  history.reset(2);
  assert(history.tracks().empty() && history.time() == -1);
  assert(!history.update(1, 2.0, 1, {observation(1)}));
  assert(history.update(2, 2.0, 1, {observation(1)}));
  assert(history.update(2, 5.0, 2, {observation(2)}));
  assert(!history.find(1) && history.find(2));
  ltv::LtvFeatureHistory boundary(config);
  for (int k = 0; k <= 20; ++k)
    assert(boundary.update(0, k / 20.0, k + 1, {observation(1)}));
  assert(boundary.find(1)->samples.size() == 21 && boundary.find(1)->samples.front().time == 0);
  assert(boundary.update(0, std::nextafter(1.0, 2.0), 22, {observation(1)}));
  assert(boundary.find(1)->samples.size() == 21 && boundary.find(1)->samples.front().time == .05);
  // Default/OFF preserves the old unbounded-by-count sample behavior and
  // protected-identity capacity exception; new validation is opt-in only.
  config.bounded_memory = false;
  config.max_candidates = 1;
  config.max_samples_per_track = 0;
  config.max_observations_per_packet = 0;
  config.max_camera_count = 0;
  ltv::LtvFeatureHistory legacy(config);
  for (int k = 0; k < 30; ++k)
    assert(legacy.update(0, k * .001, k + 1, {observation(10, 3)}));
  assert(legacy.find(10)->samples.size() == 30);
  assert(legacy.update(0, .031, 31, {observation(20)}, {20}));
  assert(legacy.tracks().size() == 2);
  std::cout << "PASS bounded samples/candidates/input/protected preflight/forget and OFF legacy semantics\n";
}
