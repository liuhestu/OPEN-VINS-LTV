#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ltv {
// LtvFeatureHistory value types
struct HistoryObservation {
  size_t feature_id = 0;
  int camera_id = 0;
  Eigen::Vector3d bearing = Eigen::Vector3d::Zero();
  bool match_valid = true;
  std::string pixel_source_key;
  Eigen::Matrix2d pixel_to_tangent = Eigen::Matrix2d::Zero();
};
struct HistorySample {
  double time = 0;
  uint64_t context_version = 0;
  std::vector<HistoryObservation> observations;
};
struct FeatureTrackHistory {
  double first_time = 0, last_time = 0;
  size_t total_frames = 0;
  bool visible = false, identity_valid = true;
  std::deque<HistorySample> samples;
};
struct FeatureHistoryConfig {
  size_t max_candidates = 256;
  double history_window = 1.0, candidate_ttl = 2.0;
  bool bounded_memory = false;
  size_t max_samples_per_track = 21, max_observations_per_packet = 512, max_camera_count = 2;
};
// Stores observations only. A caller must resolve all poses in one coherent context;
// history does not pretend means from different optimization versions share a joint P.

// LtvSeedEstimator value types
// All rotations map the second frame into the first; perturbations are right multiplicative.
struct SeedPose {
  Eigen::Matrix3d R_WB = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_WB = Eigen::Vector3d::Zero();
  double t = 0.0;
};
struct SeedCamera {
  Eigen::Matrix3d R_BC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_BC = Eigen::Vector3d::Zero();
};
struct SeedObservation {
  size_t pose_index = 0;
  size_t camera_index = 0;
  Eigen::Vector3d bearing_C = Eigen::Vector3d::UnitZ();
  std::string pixel_source_key;
  Eigen::Matrix2d pixel_to_tangent = Eigen::Matrix2d::Zero();
};
struct SeedInput {
  std::vector<SeedPose> poses;
  std::vector<SeedCamera> cameras;
  std::vector<SeedObservation> observations;
  size_t execution_pose_index = 0;
  size_t anchor_observation = 0;
};
struct SeedEstimate {
  bool valid = false;
  std::string reason = "UNCOMPUTED";
  Eigen::Vector3d point_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d landmark_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchor_ray_B = Eigen::Vector3d::Zero();
  Eigen::Matrix3d information_geometry = Eigen::Matrix3d::Zero();
  double condition = 0.0;
  double max_angle_rad = 0.0;
  double max_residual_rad = 0.0;
  double min_ray = 0.0;
  double range = 0.0;   // Euclidean distance from anchor camera, not optical depth.
  double depth_z = 0.0; // Anchor camera optical-axis coordinate; may be negative for signed bearings.
};

struct SeedUncertainty {
  bool valid = false;
  std::string reason = "UNCOMPUTED";
  Eigen::Matrix3d covariance_B = Eigen::Matrix3d::Zero();
  Eigen::MatrixXd jacobian;
  double sigma_parallel = 0.0;
  double relative_parallel_risk = 0.0;
  double covariance_min_eigenvalue = 0.0;
};

// LtvFeatureQuality value types
enum class FeatureSeedSource { TemporalPose, Stereo, MsckfGeometry, StereoThenTemporal };
const char *toString(FeatureSeedSource source);
struct FeatureSeedCandidate {
  size_t feature_id = 0;
  uint64_t epoch = 0;
  double time = 0;
  Eigen::Vector3d landmark_B = Eigen::Vector3d::Zero();
  FeatureSeedSource source = FeatureSeedSource::TemporalPose;
  bool geometry_valid = false, independent_check_available = false;
  double min_ray = 0, condition = 0, parallax_rad = 0, residual_rad = 0;
  double relative_risk = 0, check_residual_rad = 0;
};
struct FeatureQualityConfig {
  size_t min_history_frames = 3;
  double min_history_span = 0.10;
  double min_ray = 0.1, max_condition = 1e8, min_parallax_rad = 0.008726646259971648;
  double max_residual_rad = 0.008726646259971648, max_relative_risk = 0.05;
  bool require_independent_check = false;
};
struct FeatureQualityDecision {
  bool opportunity = false, accepted = false;
  std::string reason;
  double risk = 0;
};

// LtvActiveConsistency value types
struct LtvFeatureContext;
struct ActiveConsistencyConfig {
  bool enabled = false;
  size_t min_history_frames = 5;
  double min_history_span_s = .20;
  double max_history_residual_rad = .04, max_holdout_residual_rad = .04;
  int retire_after_consecutive_failures = 2;
  // Select from the existing history; never create a second observation store.
  double history_window_s = 1.0;
};
enum class ActiveConsistencyReason {
  Disabled,
  InsufficientHistory,
  MissingPoseSupport,
  FitFailed,
  HistoryInconsistent,
  HoldoutInconsistent,
  Pass
};
const char *toString(ActiveConsistencyReason reason);
struct ActiveConsistencyResult {
  size_t feature_id = 0;
  bool evaluable = false, pass = true;
  size_t history_count = 0, missing_pose_count = 0;
  double history_span_s = 0;
  double history_max_residual_rad = std::numeric_limits<double>::quiet_NaN();
  double holdout_residual_rad = std::numeric_limits<double>::quiet_NaN();
  bool fit_valid = false;
  Eigen::Vector3d point_W = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double fit_condition = std::numeric_limits<double>::quiet_NaN();
  std::string fit_reason;
  ActiveConsistencyReason reason = ActiveConsistencyReason::Disabled;
};
// Pure geometry only. Caller provides history from the current context epoch;
// FeatureTrackHistory itself has no epoch tag. No state/P or failure-count writes.

// LtvFeatureManager value types
enum class LandmarkPhase { Candidate, SeedReady, Active, Coasting, Retired };
const char *toString(LandmarkPhase phase);
struct LtvFeatureManagerConfig {
  FeatureHistoryConfig history;
  FeatureQualityConfig quality;
  size_t max_active = 30, min_features = 15;
  int max_missed_frames = 2;
  double maturity_seconds = 0.10;
  bool apply_seed = true;
  bool bounded_memory = false;
};
struct ManagedLandmark {
  LandmarkPhase phase = LandmarkPhase::Candidate;
  double first_seen = 0, entered = -1, seeded = -1, last_seen = 0;
  int missed_frames = 0;
  int consistency_fail_count = 0;
  uint64_t consistency_reject_count = 0;
  bool ever_opportunity = false, seed_written = false;
  std::string reason = "candidate";
};
struct LandmarkBirth {
  size_t feature_id = 0;
  bool apply_seed = true;
  FeatureSeedCandidate seed;
};
enum class LandmarkRetirementKind { ActiveRetired, NeverAdmittedCandidateTtl };
struct LandmarkRetirement {
  size_t feature_id = 0;
  LandmarkRetirementKind kind = LandmarkRetirementKind::ActiveRetired;
  double time = 0;
  ManagedLandmark record;
};
struct ActiveConsistencyDiagnostic {
  ActiveConsistencyResult result;
  int fail_count = 0;
  uint64_t reject_count = 0;
  std::string action = "UPDATE";
};
struct LandmarkManagerFrame {
  bool active_consistency_enabled = false;
  size_t consistency_active_count = 0, consistency_evaluable_count = 0, consistency_pass_count = 0;
  size_t consistency_skip_count = 0, consistency_retire_count = 0;
  std::vector<ActiveConsistencyDiagnostic> active_consistency;
  bool accepted_input = false;
  std::string reason;
  uint64_t epoch = 0;
  double time = 0;
  std::vector<size_t> retained_ids;
  std::vector<HistoryObservation> observations;
  std::vector<LandmarkBirth> births;
  std::vector<size_t> retired_ids;
  size_t mature_visible = 0, opportunity_tracks = 0, admitted_tracks = 0, eligible_seeds = 0;
  bool enough_mature = false;
  // Bounded mode emits one-time typed transitions instead of storing tombstones.
  std::vector<LandmarkRetirement> retirement_events;
  std::vector<size_t> identity_guard_rejected_ids, capacity_rejected_ids;
  uint64_t active_retired_total = 0, candidate_ttl_total = 0, identity_guard_rejections_total = 0;
  uint64_t identity_guard_insertions = 0;
  size_t identity_guard_set_bits = 0, exact_records = 0;
};
// Pure causal state machine; it cannot access observer x/P or modify the tracker.
// Returned births must be installed by the controlled core before correction.

// LtvLandmarkContext value types
// Immutable caller-supplied geometry for one causal camera transaction.
// Contains no tracker, estimator state handle, observer, or readiness policy.
struct LtvFeatureContext {
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

// LtvFeatureManager value types
struct LtvFeatureConfig {
  LtvFeatureManagerConfig manager;
  ActiveConsistencyConfig active_consistency;
  FeatureSeedSource source = FeatureSeedSource::TemporalPose;
  double bearing_sigma_rad = 0.0008726646259971648;
};
struct FeatureSeedResult {
  FeatureSeedCandidate candidate;
  SeedInput input;
  Eigen::MatrixXd input_covariance;
  SeedEstimate estimate;
  SeedUncertainty uncertainty;
  // e_seed(est-true)=-J_main delta_x(true-est)+J_bearing epsilon.
  Eigen::MatrixXd main_jacobian;
  Eigen::MatrixXd bearing_jacobian;
  // Diagnostic only: never changes current registered candidate admission.
  bool heldout_check_available = false;
  size_t heldout_check_observations = 0;
  double heldout_max_residual_rad = 0;
  std::string covariance_assumption = "FULL_POSE_CROSS_BLOCKS_FIXED_EXTRINSICS_INDEPENDENT_BEARING_APPROXIMATION";
};
struct LtvFeatureFrame {
  LandmarkManagerFrame management;
  std::vector<FeatureSeedResult> seeds;
};
// Owns history, identity, initialization, admission/lifecycle and active consistency.
// Copyable so the host can stage and atomically commit a feature transaction.
// Never owns or mutates observer x/P, readiness, or the OpenVINS estimator.
} // namespace ltv
