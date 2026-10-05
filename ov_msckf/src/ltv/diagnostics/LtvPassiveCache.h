#pragma once
#include "ltv/observer/LtvAdapter.h"
#include <fstream>
namespace ov_msckf {
// Local lossless IEEE754/native-little-endian cache. Not a dataset/GT format.
// Explicit finish marker distinguishes a complete stream from a crashed capture.
class LtvPassiveCacheWriter {
public:
  explicit LtvPassiveCacheWriter(const std::string &path);
  void imu(const ov_core::ImuData &sample);
  void process(double time, const std::vector<LtvBearing> &bearings, const LtvCalibration &calibration, const Eigen::Vector3d &ba,
               const Eigen::Vector3d &bg, uint64_t version, const ltv::FeaturePipelineContext *context, const LtvFrame &expected,
               const LtvAdapter &adapter);
  void pause(double time, const std::string &reason, uint64_t version, const LtvFrame &expected, const LtvAdapter &adapter);
  void finish();

private:
  std::ofstream out_;
  bool enabled_ = false, finished_ = false;
  void record(uint8_t type, const std::string &payload);
};
struct LtvPassiveCacheResult {
  uint64_t imu_events = 0, process_events = 0, pause_events = 0;
  uint64_t compared_events = 0;
  double first_imu_time = 0, last_imu_time = 0;
};
// Throws on malformed/truncated stream, any exact-value mismatch, or trailing bytes.
using LtvCacheDiagnostic =
    std::function<void(bool before, const LtvAdapter &, const ltv::FeaturePipelineContext &, const LtvCalibration &)>;
LtvPassiveCacheResult replayLtvPassiveCache(const std::string &path, const LtvOptions &options, const LtvCacheDiagnostic &diagnostic = {});
} // namespace ov_msckf
