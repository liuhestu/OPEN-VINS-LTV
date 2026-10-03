#include "ltv/LtvFeaturePipeline.h"
#include <iostream>
#include <stdexcept>
using namespace ltv;
static void check(bool pass, const char *reason) {
  if (!pass)
    throw std::runtime_error(reason);
}
static FeaturePipelineContext context(int tick, bool stereo) {
  FeaturePipelineContext c;
  c.time = .05 * tick;
  c.version = tick;
  for (int j = 0; j <= tick; ++j) {
    SeedPose p;
    p.t = .05 * j;
    p.p_WB = Eigen::Vector3d(.1 * j, 0, 0);
    c.poses.push_back(p);
  }
  c.execution_pose_index = tick;
  c.pose_covariance = Eigen::MatrixXd::Zero(6 * (tick + 1), 6 * (tick + 1));
  // Shared errors retain singular cross blocks; do not diagonalize these clones.
  for (int i = 0; i <= tick; ++i)
    for (int j = 0; j <= tick; ++j)
      c.pose_covariance.block<6, 6>(6 * i, 6 * j) = .0001 * Eigen::Matrix<double, 6, 6>::Identity();
  c.cameras.push_back(SeedCamera{});
  if (stereo) {
    SeedCamera right;
    right.p_BC = Eigen::Vector3d(.2, 0, 0);
    c.cameras.push_back(right);
  }
  for (size_t k = 0; k < c.cameras.size(); ++k) {
    HistoryObservation o;
    o.feature_id = 42;
    o.camera_id = static_cast<int>(k);
    o.bearing = (Eigen::Vector3d(.3, .2, 3) - c.poses.back().p_WB - c.cameras[k].p_BC).normalized();
    c.observations.push_back(o);
  }
  return c;
}
int main() {
  try {
    FeaturePipelineConfig cfg;
    cfg.source = FeatureSeedSource::Stereo;
    LtvFeaturePipeline stereo(cfg);
    check(stereo.process(context(0, true)).management.births.empty(), "stereo history wait");
    stereo.process(context(1, true));
    auto s = stereo.process(context(2, true));
    check(s.management.births.size() == 1, "stereo causal seed accepted");
    check(s.seeds[0].heldout_check_available && s.seeds[0].heldout_check_observations == 2 && s.seeds[0].heldout_max_residual_rad < 1e-7,
          "stereo past observations held out from seed");
    check(!s.seeds[0].candidate.independent_check_available, "holdout diagnostic does not silently change candidate gate");
    check(s.seeds.size() == 1 && s.seeds[0].input.poses.size() == 1, "stereo shares execution pose");
    check(s.seeds[0].uncertainty.jacobian.leftCols(6).norm() < 1e-10, "common stereo pose cancels");
    check((s.management.births[0].seed.landmark_B - Eigen::Vector3d(.1, .2, 3)).norm() < 1e-10, "execution body frame");
    check(s.management.observations.size() == 1 && s.management.observations[0].camera_id == 0, "right camera seed only");
    check(stereo.process(context(3, true)).seeds.empty(), "no repeated seed geometry for active point");
    LtvFeaturePipeline temporal;
    temporal.process(context(0, false));
    temporal.process(context(1, false));
    auto t = temporal.process(context(2, false));
    check(t.management.births.size() == 1 && t.seeds[0].input.poses.size() == 3, "temporal current context resolves history");
    const auto &cov = t.seeds[0].input_covariance;
    check((cov.block<6, 6>(0, 6) - .0001 * Eigen::Matrix<double, 6, 6>::Identity()).norm() == 0, "full pose cross block retained");
    auto duplicate = context(2, false);
    check(!temporal.process(duplicate).management.accepted_input, "duplicate event rejected");
    auto future = context(3, false);
    future.poses[0].t = .2;
    check(!temporal.process(future).management.accepted_input, "future pose prohibited");
    auto bad = context(3, false);
    bad.pose_covariance(0, 0) = -1;
    check(!temporal.process(bad).management.accepted_input, "invalid covariance cannot mutate history");
    check(temporal.manager().history().time() == .1, "rejected input preserves event cursor");
    LtvFeaturePipeline missing;
    missing.process(context(0, false));
    missing.process(context(1, false));
    auto shortctx = context(2, false);
    SeedPose now = shortctx.poses.back();
    shortctx.poses = {now};
    shortctx.execution_pose_index = 0;
    shortctx.pose_covariance = .0001 * Eigen::Matrix<double, 6, 6>::Identity();
    auto m = missing.process(shortctx);
    check(m.management.births.empty() && m.seeds[0].input.observations.size() == 1, "missing historical pose not fabricated");
    FeaturePipelineConfig hybrid_config;
    hybrid_config.source = FeatureSeedSource::StereoThenTemporal;
    LtvFeaturePipeline hybrid_near(hybrid_config), stereo_near(cfg);
    auto exact_seed = [&](const FeatureSeedDiagnostic &a, const FeatureSeedDiagnostic &b) {
      check(a.candidate.source == b.candidate.source, "hybrid selected concrete source");
      check((a.estimate.landmark_B.array() == b.estimate.landmark_B.array()).all(), "route exact seed mean");
      check((a.uncertainty.covariance_B.array() == b.uncertainty.covariance_B.array()).all(), "route exact uncertainty");
      check(a.uncertainty.relative_parallel_risk == b.uncertainty.relative_parallel_risk, "route exact risk");
      check(a.input_covariance.rows() == b.input_covariance.rows() && (a.input_covariance.array() == b.input_covariance.array()).all(),
            "route exact full input covariance");
    };
    for (int tick = 0; tick < 3; ++tick) {
      auto input = context(tick, true);
      auto hybrid = hybrid_near.process(input), original = stereo_near.process(input);
      check(hybrid.seeds.size() == 1 && original.seeds.size() == 1, "near route diagnostics");
      exact_seed(hybrid.seeds[0], original.seeds[0]);
      check(hybrid.management.births.size() == original.management.births.size(), "near route same admission");
    }
    LtvFeaturePipeline hybrid_far(hybrid_config), temporal_far;
    for (int tick = 0; tick < 3; ++tick) {
      auto input = context(tick, true);
      for (size_t j = 0; j < input.poses.size(); ++j)
        input.poses[j].p_WB.x() = .5 * j;
      input.cameras[1].p_BC.x() = .1;
      for (auto &o : input.observations)
        o.bearing = (Eigen::Vector3d(.3, .2, 30) - input.poses.back().p_WB - input.cameras[o.camera_id].p_BC).normalized();
      auto hybrid = hybrid_far.process(input), original = temporal_far.process(input);
      check(hybrid.seeds.size() == 1 && original.seeds.size() == 1, "far route diagnostics");
      exact_seed(hybrid.seeds[0], original.seeds[0]);
      check(hybrid.seeds[0].candidate.source == FeatureSeedSource::TemporalPose, "far stereo fallback source");
      check(hybrid.management.births.size() == original.management.births.size(), "far route same admission");
      if (tick == 2)
        check(hybrid.management.births.size() == 1, "far temporal fallback actual admission");
    }
    std::cout << "feature_pipeline PASS: temporal cross covariance, stereo pose cancellation, causal context and once-only seed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
