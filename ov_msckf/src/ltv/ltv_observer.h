#pragma once

#include "ltv_types.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ltv {

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

class LtvObserver {
public:
  LtvObserver();

  void configure(const LtvConfig &config);
  void reset(LtvResetReason reason = LtvResetReason::EstimatorReset);
  void start(double imu_timestamp);

  void propagateImu(double dt, const Eigen::Vector3d &acc_measurement, const Eigen::Vector3d &gyro_measurement,
                    const Eigen::Vector3d &accel_bias, const Eigen::Vector3d &gyro_bias);

  LtvSnapshot updateFeatures(double frame_timestamp, double imu_timestamp, const std::vector<LtvFeatureObservation> &observations,
                             const Eigen::Matrix3d &rotation_body_camera, const Eigen::Vector3d &position_body_camera);

  // Explicit opt-in after start(), before any camera event. reset() disables it.
  bool enableControlledFeatures(uint64_t epoch);
  LtvControlledResult updateFeaturesControlled(double frame_timestamp, double imu_timestamp,
                                               const std::vector<LtvFeatureObservation> &observations,
                                               const Eigen::Matrix3d &rotation_body_camera, const Eigen::Vector3d &position_body_camera,
                                               const LtvControlledFeatures &control);

  LtvSnapshot snapshot(double frame_timestamp = 0.0) const;
  bool enabled() const;
  bool started() const;

  // Read-only accessors are intentionally provided for formula and lifecycle tests.
  const Eigen::VectorXd &state() const;
  const Eigen::MatrixXd &covariance() const;
  int slotForFeature(int feature_id) const;

private:
  LtvSnapshot updateFeaturesImpl(double frame_timestamp, double imu_timestamp, const std::vector<LtvFeatureObservation> &observations,
                                 const Eigen::Matrix3d &rotation_body_camera, const Eigen::Vector3d &position_body_camera,
                                 const LtvControlledFeatures *control);
  void initializeBaseState();
  void updateFeatureLifecycle(const std::vector<LtvFeatureObservation> &observations);
  void rebuildState(const std::vector<int> &feature_ids);
  bool sanitizeCovariance();
  bool stateFinite() const;
  Eigen::MatrixXd processNoise() const;
  int velocityOffset() const;
  int gravityOffset() const;

  LtvConfig config_;
  bool configured_ = false;
  bool started_ = false;
  double imu_timestamp_ = 0.0;
  double last_camera_imu_timestamp_ = -1.0;
  double last_frame_timestamp_ = -1.0;
  int healthy_camera_updates_ = 0;
  int observed_features_ = 0;
  double innovation_norm_ = 0.0;
  double update_time_ms_ = 0.0;
  int camera_substeps_ = 0;
  LtvResetReason last_reset_reason_ = LtvResetReason::None;

  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
  std::unordered_map<int, int> feature_to_slot_;
  std::unordered_map<int, int> missed_frames_;
  std::unordered_map<int, int> candidate_age_;
  bool controlled_features_ = false;
  uint64_t controlled_epoch_ = 0;
  std::unordered_set<int> admitted_ids_;
};

} // namespace ltv
