#pragma once
#include "LtvLandmarkManager.h"
#include "LtvSeedUncertainty.h"
namespace ltv {
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
struct FeaturePipelineConfig {
  LandmarkManagerConfig manager;
  FeatureSeedSource source = FeatureSeedSource::TemporalPose;
  double bearing_sigma_rad = 0.0008726646259971648;
};
struct FeatureSeedDiagnostic {
  FeatureSeedCandidate candidate;
  SeedInput input;
  Eigen::MatrixXd input_covariance;
  SeedEstimate estimate;
  SeedUncertainty uncertainty;
  // Diagnostic only: never changes current registered candidate admission.
  bool heldout_check_available = false;
  size_t heldout_check_observations = 0;
  double heldout_max_residual_rad = 0;
  std::string covariance_assumption = "FULL_POSE_CROSS_BLOCKS_FIXED_EXTRINSICS_INDEPENDENT_BEARING_APPROXIMATION";
};
struct FeaturePipelineFrame {
  LandmarkManagerFrame management;
  std::vector<FeatureSeedDiagnostic> seeds;
};
class LtvFeaturePipeline {
public:
  explicit LtvFeaturePipeline(const FeaturePipelineConfig &config = {});
  void reset(uint64_t epoch);
  FeaturePipelineFrame process(const FeaturePipelineContext &context);
  const LtvLandmarkManager &manager() const { return manager_; }

private:
  FeaturePipelineConfig config_;
  LtvLandmarkManager manager_;
};
} // namespace ltv
