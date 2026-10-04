#pragma once

#include "ltv/landmark_adapter/LtvFeatureHistory.h"
#include "ltv/landmark_adapter/LtvSeedEstimator.h"

namespace ltv {
// Immutable caller-supplied geometry for one causal camera transaction.
// Contains no tracker, estimator state handle, observer, or readiness policy.
struct FeaturePipelineContext {
  uint64_t epoch = 0, version = 0;
  double time = 0;
  std::vector<SeedPose> poses;
  // Joint right-body-rotation/world-position errors; includes all cross blocks.
  Eigen::MatrixXd pose_covariance;
  std::vector<SeedCamera> cameras;
  size_t execution_pose_index = 0;
  std::vector<HistoryObservation> observations;
};
} // namespace ltv
