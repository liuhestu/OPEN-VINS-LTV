// Reuse only the fixture constructor; assertions and rejection oracle are independent.
#define main original_adapter_fixture_main
#include "../../../ov_msckf/tests/ltv/test_ltv_adapter_hardened.cpp"
#undef main
#include <functional>
#include <limits>

static LtvOptions opts() {
  LtvOptions o;
  o.enabled = true;
  o.feature_readiness_enabled = true;
  o.feature_seed_source = "STEREO";
  o.passive_hardening_enabled = true;
  return o;
}
static void warm(Harness &h) {
  for (int k = 0; k <= 7; ++k)
    h.frame(k, true);
  require(h.adapter.core().started(), "fixture started");
}
static void reject_before_prediction(const std::function<void(LtvCalibration &, ltv::FeaturePipelineContext &)> &mutate) {
  Harness h(opts());
  warm(h);
  h.feed_to(.4);
  auto ctx = h.context(8, true);
  auto c = h.cal;
  mutate(c, ctx);
  auto x = h.adapter.core().state();
  auto P = h.adapter.core().covariance();
  auto ids = h.adapter.feature_ids();
  const auto history = h.adapter.feature_pipeline()->manager().history().time();
  bool rejected = false;
  try {
    h.adapter.process(.4, {}, c, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 9, &ctx);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "invalid calibration/context not rejected");
  require((x.array() == h.adapter.core().state().array()).all() && (P.array() == h.adapter.core().covariance().array()).all(),
          "prevalidation rejection mutated full x/P");
  require(ids == h.adapter.feature_ids() && history == h.adapter.feature_pipeline()->manager().history().time(),
          "prevalidation consumed IDs/history");
}
int main() {
  try {
    reject_before_prediction([](LtvCalibration &, ltv::FeaturePipelineContext &c) { c.version = 8; });
    reject_before_prediction([](LtvCalibration &, ltv::FeaturePipelineContext &c) { c.time = .41; });
    reject_before_prediction([](LtvCalibration &c, ltv::FeaturePipelineContext &) { c.R_BC(0, 0) = 2; });
    reject_before_prediction([](LtvCalibration &, ltv::FeaturePipelineContext &c) { c.cameras[0].p_BC.x() += .001; });
    reject_before_prediction([](LtvCalibration &c, ltv::FeaturePipelineContext &) { c.offset = std::numeric_limits<double>::quiet_NaN(); });
    Harness h(opts());
    warm(h);
    h.feed_to(.4);
    auto ctx = h.context(8, true);
    // Invalid joint covariance is rejected by staged pipeline after legitimate IMU prediction.
    ctx.pose_covariance(0, 0) = -1;
    auto expected = h.adapter.core();
    auto before = expected.snapshot();
    auto ids = h.adapter.feature_ids();
    const auto manager_before = h.adapter.feature_frame().management;
    const auto history = h.adapter.feature_pipeline()->manager().history().time();
    double last = before.imu_timestamp;
    for (int tick = 71; tick <= 80; ++tick) {
      double time = tick * .005;
      expected.propagateImu(time - last, Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero());
      last = time;
    }
    bool rejected = false;
    try {
      h.adapter.process(.4, {}, h.cal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 9, &ctx);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid pipeline context accepted");
    require((expected.state().array() == h.adapter.core().state().array()).all(), "rejected camera changed predicted x");
    require((expected.covariance().array() == h.adapter.core().covariance().array()).all(), "rejected camera changed predicted full P");
    auto after = h.adapter.core().snapshot();
    require(after.healthy_camera_updates == before.healthy_camera_updates && after.state_features == before.state_features,
            "rejected camera consumed correction/slot");
    require(ids == h.adapter.feature_ids() && history == h.adapter.feature_pipeline()->manager().history().time(),
            "rejected camera consumed ID/history");
    const auto &management = h.adapter.feature_frame().management;
    require(management.time == manager_before.time && management.admitted_tracks == manager_before.admitted_tracks &&
                management.births.size() == manager_before.births.size(),
            "rejected camera consumed admission/seed");
    std::cout << "PASS independent calibration/context prevalidation and postprediction camera transaction rejection\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
