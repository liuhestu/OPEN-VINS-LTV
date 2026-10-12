#pragma once
#include "ltv/LtvObserver.h"
#include "ltv/features/LtvFeatureTypes.h"
#include <deque>
#include <map>
#include <tuple>
namespace ltv {
class LtvFeatureManager;
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
  explicit LtvLandmarkHistory(double maturity = .5) : maturity_(maturity) {}
  void reset() {
    anchors_.clear();
    candidates_.clear();
    epoch_ = 0;
  }
  void before(const LtvFeatureManager &, const LtvObserver &, const LtvFeatureContext &, bool ready);
  void after(const LtvFeatureManager &, const LtvObserver &, const LtvFeatureContext &);
  const std::vector<LtvLandmarkCandidate> &candidates() const { return candidates_; }
  static bool b3_gate(bool mature, bool ready, double span, const Eigen::Vector3d &l, const Eigen::Vector3d &main);

private:
  using Key = std::tuple<uint64_t, size_t, int, double>;
  struct Anchor {
    double time;
    Eigen::Vector3d point;
  };
  static const ltv::SeedPose *find_pose(const ltv::LtvFeatureContext &, double);
  double maturity_;
  uint64_t epoch_ = 0;
  double event_time_ = -1;
  std::map<Key, std::deque<Anchor>> anchors_;
  std::vector<LtvLandmarkCandidate> candidates_;
};
} // namespace ltv
