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
  // Optional actual main-error selector; rows follow poses, columns follow State.
  // Never infer the current/history cross by treating cloned poses independent.
  Eigen::MatrixXd pose_error_selector;
  double pixel_noise_variance = 1.;
  std::vector<SeedCamera> cameras;
  size_t execution_pose_index = 0;
  std::vector<HistoryObservation> observations;
};
} // namespace ltv
