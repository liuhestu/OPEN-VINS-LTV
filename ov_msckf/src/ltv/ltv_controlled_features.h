#pragma once

#include "ltv_types.h"
#include <cstdint>
#include <string>
#include <vector>

namespace ltv {
// Value-only contract between lifecycle policy and the mathematical observer.
struct LtvLandmarkSeed {
  int feature_id = -1;
  bool apply_mean = true;
  Eigen::Vector3d mean_body = Eigen::Vector3d::Zero();
};

// Manager supplies the exact retained set, including unobserved coasting slots.
// Every new ID has exactly one birth; old IDs can never be reseeded in an epoch.
struct LtvControlledFeatures {
  uint64_t epoch = 0;
  double imu_timestamp = 0.0;
  std::vector<int> retained_ids;
  std::vector<LtvLandmarkSeed> births;
};

struct LtvControlledResult {
  bool accepted = false;
  std::string reason;
  LtvSnapshot snapshot;
};

} // namespace ltv
