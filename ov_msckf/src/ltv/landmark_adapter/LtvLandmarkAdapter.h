#pragma once
#include "ltv/landmark_adapter/LtvLandmarkContext.h"
#include "ltv/landmark_adapter/LtvLandmarkManager.h"
#include "ltv/landmark_adapter/LtvSeedUncertainty.h"
namespace ltv {
struct FeaturePipelineConfig {
  LandmarkManagerConfig manager;
  ActiveConsistencyConfig active_consistency;
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
// Owns history, identity, initialization, admission/lifecycle and active consistency.
// Copyable so LtvAdapter can stage and atomically commit a complete transaction.
// Never owns or mutates observer x/P, readiness, or the OpenVINS estimator.
class LtvLandmarkAdapter {
public:
  explicit LtvLandmarkAdapter(const FeaturePipelineConfig &config = {});
  void reset(uint64_t epoch);
  FeaturePipelineFrame process(const FeaturePipelineContext &context, bool allow_admission = true);
  const LtvLandmarkManager &manager() const { return manager_; }

private:
  FeaturePipelineConfig config_;
  LtvLandmarkManager manager_;
  LtvActiveConsistency active_consistency_;
};
} // namespace ltv
