#include "ltv/LtvActiveConsistency.h"
#include "ltv/LtvFeaturePipeline.h"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
using namespace ltv;
static void check(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
struct Fixture {
  FeaturePipelineContext ctx;
  FeatureTrackHistory history;
  Eigen::Vector3d point{.6, .4, 4.};
  Fixture() {
    ctx.time = 1.;
    ctx.epoch = 3;
    ctx.version = 7;
    SeedCamera cam;
    cam.p_BC = Eigen::Vector3d(.1, -.02, .03);
    cam.R_BC = Eigen::AngleAxisd(.17, Eigen::Vector3d::UnitY()).toRotationMatrix();
    ctx.cameras.push_back(cam);
    for (int k = 0; k < 7; ++k) {
      SeedPose p;
      p.t = .25 + .125 * k;
      p.p_WB = Eigen::Vector3d(.1 * k, .02 * k * k, 0);
      p.R_WB = Eigen::AngleAxisd(.02 * k, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      ctx.poses.push_back(p);
      HistoryObservation o;
      o.feature_id = 42;
      o.bearing = (cam.R_BC.transpose() * (p.R_WB.transpose() * (point - p.p_WB) - cam.p_BC)).normalized();
      HistorySample s;
      s.time = p.t;
      s.context_version = k;
      s.observations.push_back(o);
      history.samples.push_back(s);
      if (k == 6)
        ctx.observations.push_back(o);
    }
    ctx.execution_pose_index = 6;
  }
};
int main() {
  ActiveConsistencyConfig c;
  c.enabled = true;
  LtvActiveConsistency evaluator(c);
  Fixture f;
  auto good = evaluator.evaluate(42, 1., f.history, f.ctx);
  check(good.evaluable && good.pass && good.history_count == 6, "clean past model");
  check((good.point_W - f.point).norm() < 1e-10, "nonzero extrinsic geometry");
  Fixture bad = f;
  bad.ctx.observations[0].bearing = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitY()) * bad.ctx.observations[0].bearing;
  auto held = evaluator.evaluate(42, 1., bad.history, bad.ctx);
  check(held.evaluable && !held.pass && held.reason == ActiveConsistencyReason::HoldoutInconsistent, "current holdout reject");
  check((held.point_W - good.point_W).norm() == 0, "current bearing excluded from fit");
  bad.history.samples.back().observations[0].bearing = Eigen::Vector3d::UnitX();
  HistorySample future = bad.history.samples.back();
  future.time = 1.1;
  bad.history.samples.push_back(future);
  auto future_test = evaluator.evaluate(42, 1., bad.history, bad.ctx);
  check((future_test.point_W - held.point_W).norm() == 0, "future and current history excluded");
  Fixture inconsistent = f;
  for (size_t j = 0; j < 6; ++j)
    inconsistent.history.samples[j].observations[0].bearing =
        Eigen::AngleAxisd(j % 2 ? .15 : -.15, Eigen::Vector3d::UnitY()) * inconsistent.history.samples[j].observations[0].bearing;
  auto history_bad = evaluator.evaluate(42, 1., inconsistent.history, inconsistent.ctx);
  check(history_bad.evaluable && !history_bad.pass && history_bad.reason == ActiveConsistencyReason::HistoryInconsistent,
        "valid high residual not fitfailed");
  Fixture short_history = f;
  short_history.history.samples.erase(short_history.history.samples.begin(), short_history.history.samples.begin() + 3);
  auto insufficient = evaluator.evaluate(42, 1., short_history.history, short_history.ctx);
  check(!insufficient.evaluable && insufficient.pass && insufficient.reason == ActiveConsistencyReason::InsufficientHistory,
        "insufficient is neutral");
  Fixture missing = f;
  missing.ctx.poses.erase(missing.ctx.poses.begin(), missing.ctx.poses.begin() + 3);
  missing.ctx.execution_pose_index = 3;
  auto unsupported = evaluator.evaluate(42, 1., missing.history, missing.ctx);
  check(!unsupported.evaluable && unsupported.pass && unsupported.reason == ActiveConsistencyReason::MissingPoseSupport,
        "missing clones neutral");
  Fixture gauge = f;
  Eigen::Matrix3d Q = Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  Eigen::Vector3d tr(10, -20, 3);
  for (auto &p : gauge.ctx.poses) {
    p.R_WB = Q * p.R_WB;
    p.p_WB = Q * p.p_WB + tr;
  }
  auto transformed = evaluator.evaluate(42, 1., gauge.history, gauge.ctx);
  check(transformed.pass && transformed.evaluable && (transformed.point_W - (Q * good.point_W + tr)).norm() < 1e-9,
        "world gauge invariant");
  check(std::abs(transformed.holdout_residual_rad - good.holdout_residual_rad) < 1e-7, "gauge angle invariant");
  Fixture degenerate = f;
  for (auto &p : degenerate.ctx.poses) {
    p.R_WB.setIdentity();
    p.p_WB.setZero();
  }
  for (auto &s : degenerate.history.samples)
    s.observations[0].bearing = Eigen::Vector3d::UnitZ();
  auto invalid = evaluator.evaluate(42, 1., degenerate.history, degenerate.ctx);
  check(!invalid.evaluable && invalid.pass && invalid.reason == ActiveConsistencyReason::FitFailed, "degenerate neutral");
  Fixture duplicate = f;
  duplicate.ctx.poses.push_back(duplicate.ctx.poses.front());
  check(!evaluator.evaluate(42, 1., duplicate.history, duplicate.ctx).evaluable, "duplicate pose rejected");
  ActiveConsistencyConfig off_config;
  off_config.history_window_s = .1;
  LtvActiveConsistency off(off_config);
  auto disabled = off.evaluate(42, 1., f.history, f.ctx);
  check(disabled.pass && !disabled.evaluable && disabled.reason == ActiveConsistencyReason::Disabled, "default off");
  std::cout << "active consistency tests PASS\n";
}
