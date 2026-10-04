#include "ltv/observer/ltv_observer.h"
#include <cassert>
#include <climits>
#include <iostream>
#include <limits>

int main() {
  ltv::LtvConfig cfg;
  cfg.enable = true;
  ltv::LtvObserver legacy, bounded;
  for (auto *o : {&legacy, &bounded}) {
    o->configure(cfg);
    o->start(0);
  }
  assert(legacy.enableControlledFeatures(4));
  assert(bounded.enableControlledFeatures(4, true));
  const auto R = Eigen::Matrix3d::Identity().eval();
  const auto p = Eigen::Vector3d::Zero().eval();
  double frame = 0;
  auto update = [&](ltv::LtvObserver &o, const std::vector<int> &retained, const std::vector<int> &births, uint64_t epoch = 4) {
    ltv::LtvControlledFeatures control;
    control.epoch = epoch;
    control.retained_ids = retained;
    std::vector<ltv::LtvFeatureObservation> observations;
    for (int id : retained) {
      ltv::LtvFeatureObservation z;
      z.feature_id = id;
      z.normalized_coordinate = Eigen::Vector3d(.2, .1, 1).normalized();
      observations.push_back(z);
    }
    for (int id : births) {
      ltv::LtvLandmarkSeed b;
      b.feature_id = id;
      b.mean_body = Eigen::Vector3d(1, 2, 3);
      control.births.push_back(b);
    }
    return o.updateFeaturesControlled(frame, 0, observations, R, p, control).accepted;
  };
  // Out-of-order batch checks every birth against the old, committed watermark.
  assert(update(legacy, {2, 1}, {2, 1}));
  assert(update(bounded, {2, 1}, {2, 1}));
  assert(bounded.admissionHighWaterMark() == 2 && bounded.admissionHistorySize() == 0);
  assert(legacy.admissionHistorySize() == 2);
  for (int i = 3; i < 1000; ++i) {
    frame += .001;
    assert(update(legacy, {1, i}, {i}));
    assert(update(bounded, {1, i}, {i}));
    assert((legacy.state() - bounded.state()).norm() == 0);
    assert((legacy.covariance() - bounded.covariance()).norm() == 0);
    assert(bounded.admissionHistorySize() == 0 && bounded.admissionHighWaterMark() == i);
    assert(bounded.slotForFeature(1) == legacy.slotForFeature(1));
  }
  frame += .001;
  const auto x = bounded.state().eval();
  const auto P = bounded.covariance().eval();
  assert(!update(bounded, {1, 2}, {2})); // retired ID cannot return
  assert(!update(bounded, {1, 1001, 1000}, {1001, 1000}, 3));
  assert(bounded.admissionHighWaterMark() == 999);
  assert((x - bounded.state()).norm() == 0 && (P - bounded.covariance()).norm() == 0);
  assert(update(bounded, {1, 1001}, {1001}));
  frame += .001;
  assert(!update(bounded, {1, 1000}, {1000})); // unused gap <= watermark is conservatively rejected
  assert(update(bounded, {1, INT_MAX}, {INT_MAX}));
  frame += .001;
  assert(!update(bounded, {1, INT_MAX}, {INT_MAX}));
  bounded.reset();
  bounded.start(0);
  assert(bounded.admissionHighWaterMark() == -1);
  assert(bounded.enableControlledFeatures(5, true));
  assert(update(bounded, {1}, {1}, 5));
  std::cout << "PASS bounded controlled IDs: exact x/P, unordered births, retained low IDs, retired/gap rejection, failed epoch, INT_MAX "
               "and reset\n";
}
