#include "ltv/diagnostics/LtvLandmarkShadow.h"
#include "ltv/diagnostics/LtvPassiveCache.h"
#include <iostream>
using namespace ov_msckf;
int main(int argc, char **argv) {
  try {
    if (argc != 5 && !(argc == 6 && std::string(argv[5]) == "PAST_GEOMETRY"))
      throw std::runtime_error("usage: replay_ltv_landmark_shadow config.yaml cache.bin predictions.csv exact_report.json [PAST_GEOMETRY]");
    auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
    LtvOptions options;
    options.load(parser);
    if (options.enable_gravity || options.enable_velocity || !options.passive_hardening_enabled || !options.feature_readiness_enabled ||
        options.hardening_readiness.ready_confirm_frames != 20 || options.hardening_readiness.experimental_velocity_confirm_frames != 0)
      throw std::runtime_error("shadow requires frozen hardened Passive configuration and default20");
    options.passive_cache_path.clear();
    options.log_enabled = options.value_diagnostics_enabled = false;
    LtvLandmarkShadow shadow(argv[3], options.feature_manager.maturity_seconds, argc == 6);
    uint64_t before = 0, after = 0;
    auto result = replayLtvPassiveCache(
        argv[2], options, [&](bool pre, const LtvAdapter &a, const ltv::FeaturePipelineContext &ctx, const LtvCalibration &c) {
          shadow.event(pre, a, ctx, c);
          pre ? ++before : ++after;
        });
    if (!before || !shadow.rows())
      throw std::runtime_error("shadow has no causal prediction events");
    shadow.finish();
    std::ofstream report(argv[4]);
    report << "{\"status\":\"PASS\",\"compared_events\":" << result.compared_events << ",\"before_events\":" << before
           << ",\"after_events\":" << after << ",\"prediction_rows\":" << shadow.rows()
           << ",\"comparison\":\"exact_original_cache_complete_x_P_inputs_ID_lifecycle_readiness\"}\n";
    if (!report)
      throw std::runtime_error("shadow result write failed");
    std::cout << "shadow exact replay PASS rows=" << shadow.rows() << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
