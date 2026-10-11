#pragma once
#include "ltv/config/LtvOptions.h"
#include "ltv/features/LtvFeatureFrontend.h"
#include <deque>
#include <map>
#include <tuple>
namespace ov_msckf {
class LtvRuntime;
struct LtvCalibration;
struct LtvLandmarkCandidate {
  uint64_t epoch;
  size_t feature_id;
  int core_id;
  double entered;
  Eigen::Vector3d point, anchor_point;
  double anchor_time, span;
};
class LtvLandmarkHistory {
public:
  explicit LtvLandmarkHistory(const LtvOptions &options) : options_(options) {}
  void reset() {
    anchors_.clear();
    candidates_.clear();
    epoch_ = 0;
  }
  void before(const LtvRuntime &, const ltv::LtvFeatureContext &, const LtvCalibration &);
  void after(const LtvRuntime &, ltv::LtvFeatureContext, const LtvCalibration &, uint64_t);
  const std::vector<LtvLandmarkCandidate> &candidates() const { return candidates_; }
  static bool b3_gate(bool mature, bool ready, double span, const Eigen::Vector3d &l, const Eigen::Vector3d &main);

private:
  using Key = std::tuple<uint64_t, size_t, int, double>;
  struct Anchor {
    double time;
    Eigen::Vector3d point;
  };
  static const ltv::SeedPose *find_pose(const ltv::LtvFeatureContext &, double);
  LtvOptions options_;
  uint64_t epoch_ = 0;
  double camera_time_ = -1;
  std::map<Key, std::deque<Anchor>> anchors_;
  std::vector<LtvLandmarkCandidate> candidates_;
};
} // namespace ov_msckf
