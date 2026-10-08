#include "ltv/diagnostics/LtvPassiveCache.h"
#include <fstream>
#include <iostream>
#include <iterator>
using namespace ov_msckf;
static void require(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}
static void selftest(const std::string &prefix) {
  const std::string path = prefix + "_complete.bin";
  LtvOptions options;
  options.enabled = true;
  options.observer.enable = true;
  options.feature_readiness_enabled = true;
  options.feature_seed_source = "STEREO";
  LtvAdapter adapter(options);
  LtvPassiveCacheWriter writer(path);
  LtvCalibration calibration;
  for (int i = 0; i <= 32; ++i) {
    ov_core::ImuData s;
    s.timestamp = -.01 + i * .005;
    s.wm.setZero();
    s.am = Eigen::Vector3d(0, 0, 9.81);
    adapter.feed_imu(s);
    writer.imu(s);
  }
  for (int i = 0; i < 3; ++i) {
    ltv::FeaturePipelineContext c;
    c.time = i * .05;
    c.version = i + 1;
    ltv::SeedPose pose;
    pose.t = c.time;
    c.poses = {pose};
    c.pose_covariance = .001 * Eigen::Matrix<double, 6, 6>::Identity();
    c.cameras.resize(2);
    c.cameras[1].p_BC = Eigen::Vector3d(.2, 0, 0);
    std::vector<LtvBearing> bearings;
    for (size_t id = 0; id < 16; ++id) {
      Eigen::Vector3d point(.05 * id, .1, 3);
      for (int cam = 0; cam < 2; ++cam) {
        ltv::HistoryObservation o;
        o.feature_id = id;
        o.camera_id = cam;
        o.bearing = (point - c.cameras[cam].p_BC).normalized();
        c.observations.push_back(o);
        if (!cam)
          bearings.push_back({id, o.bearing});
      }
    }
    auto f = adapter.process(c.time, bearings, calibration, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), c.version, &c);
    if (f.available)
      require(adapter.claim(f), "selftest claim");
    writer.process(c.time, bearings, calibration, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), c.version, &c, f, adapter);
  }
  require(adapter.feature_frame().management.births.size() == 16, "selftest must exercise seed births");
  auto f = adapter.pause(.15, "test_pause", 4);
  writer.pause(.15, "test_pause", 4, f, adapter);
  writer.finish();
  auto r = replayLtvPassiveCache(path, options);
  require(r.process_events == 3 && r.pause_events == 1, "selftest events");
  std::ifstream in(path, std::ios::binary);
  std::string bytes((std::istreambuf_iterator<char>(in)), {});
  const auto truncated = prefix + "_truncated.bin";
  std::ofstream out(truncated, std::ios::binary);
  out.write(bytes.data(), bytes.size() - 1);
  out.close();
  bool rejected = false;
  try {
    replayLtvPassiveCache(truncated, options);
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, "truncation must fail");
  auto changed = options;
  changed.feature_apply_seed = false;
  rejected = false;
  try {
    replayLtvPassiveCache(path, changed);
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, "different seed behavior must fail");
  rejected = false;
  try {
    LtvPassiveCacheWriter overwrite(path);
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, "cache overwrite must fail");
  std::cout << "cache selftest PASS exact input/state/P/slots/seeds, truncation, changed method, overwrite protection\n";
}
int main(int argc, char **argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--self-test") {
      selftest(argv[2]);
      return 0;
    }
    if (argc != 4)
      throw std::runtime_error("usage: replay_ltv_feature_cache config.yaml cache.bin report.json OR --self-test unique_output_prefix");
    auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
    LtvOptions options;
    options.load(parser);
    if (options.enable_gravity || options.enable_velocity)
      throw std::runtime_error("cache replay must be Passive");
    options.passive_cache_path.clear();
    options.log_enabled = false;
    options.value_diagnostics_enabled = false;
    auto result = replayLtvPassiveCache(argv[2], options);
    std::ofstream report(argv[3]);
    if (!report)
      throw std::runtime_error("cannot open result JSON");
    report << "{\"status\":\"PASS\",\"imu_events\":" << result.imu_events << ",\"process_events\":" << result.process_events
           << ",\"pause_events\":" << result.pause_events << ",\"compared_events\":" << result.compared_events
           << ",\"comparison\":\"exact_non_wallclock\"}\n";
    std::cout << "cache replay PASS events=" << result.compared_events << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
