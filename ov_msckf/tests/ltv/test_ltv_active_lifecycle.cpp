#include "ltv/landmark_adapter/LtvLandmarkManager.h"
#include "ltv/observer/ltv_observer.h"
#include <iostream>
#include <stdexcept>
using namespace ltv;
static void check(bool x, const char *why) {
  if (!x)
    throw std::runtime_error(why);
}
static HistoryObservation obs() {
  HistoryObservation o;
  o.feature_id = 42;
  o.bearing = Eigen::Vector3d(.1, .2, 1).normalized();
  return o;
}
static FeatureSeedCandidate seed(double t) {
  FeatureSeedCandidate s;
  s.feature_id = 42;
  s.time = t;
  s.geometry_valid = true;
  s.landmark_B = Eigen::Vector3d(1, 2, 10);
  s.min_ray = 10;
  s.condition = 100;
  s.parallax_rad = .1;
  s.residual_rad = .001;
  s.relative_risk = .02;
  return s;
}
static std::map<size_t, ActiveConsistencyResult> result(bool evaluable, bool pass) {
  ActiveConsistencyResult r;
  r.feature_id = 42;
  r.evaluable = evaluable;
  r.pass = pass;
  if (evaluable) {
    r.history_max_residual_rad = pass ? .001 : .1;
    r.holdout_residual_rad = .001;
  }
  r.reason = !evaluable ? ActiveConsistencyReason::InsufficientHistory
             : pass     ? ActiveConsistencyReason::Pass
                        : ActiveConsistencyReason::HistoryInconsistent;
  return {{42, r}};
}
static LtvControlledResult camera(LtvObserver &core, const LandmarkManagerFrame &frame) {
  LtvControlledFeatures c;
  c.imu_timestamp = frame.time;
  c.epoch = frame.epoch;
  for (auto id : frame.retained_ids)
    c.retained_ids.push_back(static_cast<int>(id));
  for (const auto &b : frame.births) {
    LtvLandmarkSeed s;
    s.feature_id = static_cast<int>(b.feature_id);
    s.apply_mean = b.apply_seed;
    s.mean_body = b.seed.landmark_B;
    c.births.push_back(s);
  }
  std::vector<LtvFeatureObservation> z;
  for (const auto &o : frame.observations) {
    LtvFeatureObservation a;
    a.feature_id = static_cast<int>(o.feature_id);
    a.normalized_coordinate = o.bearing;
    z.push_back(a);
  }
  return core.updateFeaturesControlled(frame.time, frame.time, z, Eigen::Matrix3d::Identity(), Eigen::Vector3d(.05, -.02, .03), c);
}
int main() {
  for (bool bounded : {false, true}) {
    LandmarkManagerConfig c;
    c.bounded_memory = bounded;
    LtvLandmarkManager m(c);
    for (int k = 0; k < 2; ++k)
      check(m.step(0, .05 * k, k + 1, {obs()}, {seed(.05 * k)}).births.empty(), "causal history before birth");
    auto born = m.step(0, .1, 3, {obs()}, {seed(.1)});
    check(born.births.size() == 1, "one birth");
    auto pass = result(true, true), fail = result(true, false), unknown = result(false, true);
    auto accepted = m.step(0, .15, 4, {obs()}, {}, true, &pass);
    check(accepted.observations.size() == 1 && m.landmarks().at(42).consistency_fail_count == 0, "pass updates");
    auto skipped = m.step(0, .2, 5, {obs()}, {}, true, &fail);
    check(skipped.retained_ids.size() == 1 && skipped.observations.empty() && skipped.births.empty(),
          "first fail keeps slot skips correction");
    check(skipped.active_consistency.size() == 1 && skipped.active_consistency[0].action == "SKIP" &&
              skipped.active_consistency[0].fail_count == 1,
          "typed skip count");
    check(m.landmarks().at(42).missed_frames == 0 && m.history().find(42)->samples.back().time == .2, "reject remains raw visible history");
    check((m.history().find(42)->samples.back().observations[0].bearing - obs().bearing.normalized()).norm() == 0, "rejected raw ray preserved");
    auto unavailable = m.step(0, .25, 6, {obs()}, {}, true, &unknown);
    check(unavailable.observations.size() == 1 && m.landmarks().at(42).consistency_fail_count == 1,
          "unevaluable preserves count allows observation");
    m.step(0, .3, 7, {obs()}, {}, true, &pass);
    check(m.landmarks().at(42).consistency_fail_count == 0, "pass clears consecutive failure");
    m.step(0, .35, 8, {obs()}, {}, true, &fail);
    auto retired = m.step(0, .4, 9, {obs()}, {}, true, &fail);
    check(retired.retained_ids.empty() && retired.observations.empty() && retired.retired_ids.size() == 1, "second fail retires");
    check(retired.active_consistency[0].action == "RETIRE" && retired.active_consistency[0].fail_count == 2, "typed retirement");
    for (int k = 9; k < 15; ++k) {
      auto retry = m.step(0, .05 * k, k + 1, {obs()}, {seed(.05 * k)});
      check(retry.accepted_input && retry.births.empty(), "same epoch cannot readmit");
    }
    // Exercise actual controlled core with the manager's first reject packet.
    LtvConfig coreconfig;
    coreconfig.enable = true;
    LtvObserver core;
    core.configure(coreconfig);
    core.start(.1);
    check(core.enableControlledFeatures(0), "controlled enable");
    check(camera(core, born).accepted, "controlled birth");
    for (int k = 0; k < 20; ++k)
      core.propagateImu(.005, Eigen::Vector3d(.3, -.1, -9.81), Eigen::Vector3d(.1, .2, -.1), Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
    const Eigen::VectorXd x = core.state();
    const Eigen::MatrixXd P = core.covariance();
    check(P.block<3, 6>(0, 3).norm() > 1e-4, "nonzero point shared cross blocks exist");
    auto core_skip = camera(core, skipped);
    check(core_skip.accepted && core.slotForFeature(42) == 0, "skip retained actual slot");
    check((core.state() - x).norm() == 0, "no external mean patch and no skipped correction");
    check((core.covariance() - P).norm() < 1e-10 * std::max(1., P.norm()),
          "full covariance including cross blocks preserved within sanitizer roundoff");
    check(core_skip.snapshot.camera_substeps == 0, "no actual observation correction for all-skipped packet");
  }
  std::cout << "active consistency lifecycle PASS\n";
}
