#pragma once
#include "ltv/config/LtvOptions.h"
#include "ltv/features/LtvFeatureFrontend.h"
#include "ltv/observer/LtvObserver.h"
#include "ltv/runtime/LtvLandmarkHistory.h"
#include "utils/sensor_data.h"
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
namespace ov_msckf {
struct LtvCalibration {
  Eigen::Matrix3d Da = Eigen::Matrix3d::Identity(), Dw = Eigen::Matrix3d::Identity(), Tg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_ACCtoIMU = Eigen::Matrix3d::Identity(), R_GYROtoIMU = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_BC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_BC = Eigen::Vector3d::Zero();
  double offset = 0;
};
struct LtvBearing {
  size_t id;
  Eigen::Vector3d coordinate;
};
struct LtvFrame {
  uint64_t epoch = 0, version = 0, sequence = 0;
  int64_t camera_ns = 0;
  double camera_time = 0, imu_time = 0, cursor = 0;
  bool available = false;
  bool ready_G = false, ready_V = false;
  size_t mature_features = 0;
  bool hardened = false, raw_current = false;
  ltv::LtvAvailabilityOutput health;
  double prediction_angle_p95_rad = 0, velocity_correction_rate = 0, gravity_correction_rate = 0;
  bool correction_diagnostics_valid = false;
  uint64_t actual_corrections = 0;
  double compute_time_ms = 0; // Diagnostic only; excluded from exact numeric/cache comparisons.
  std::string reason = "not_started";
  ltv::LtvSnapshot snapshot;
  std::vector<LtvLandmarkCandidate> landmarks;
  uint64_t integrated_steps = 0;
  double integrated_seconds = 0;
};
class LtvRuntime {
public:
  explicit LtvRuntime(const LtvOptions &options);
  void feed_imu(const ov_core::ImuData &sample);
  LtvFrame process(double camera_time, const std::vector<LtvBearing> &bearings, const LtvCalibration &calibration,
                   const Eigen::Vector3d &ba, const Eigen::Vector3d &bg, uint64_t version,
                   const ltv::LtvFeatureContext *feature_context = nullptr);
  LtvFrame pause(double camera_time, const std::string &reason, uint64_t version);
  void reset();
  bool claim(const LtvFrame &frame);
  bool observer_started() const { return observer_started_; }
  const ltv::LtvObserver &observer() const { return observer_; }
  const ltv::LtvAvailabilityOutput &availability() const { return availability_status_; }
  const ltv::LtvFeatureFrame &feature_frame() const { return feature_frame_; }
  const ltv::LtvFeatureFrontend *feature_frontend() const { return feature_frontend_.get(); }
  const std::map<size_t, int> &feature_ids() const { return feature_to_observer_id_; }
  size_t bufferedImuSamples() {
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_.size();
  }
  static ov_core::ImuData correct(const ov_core::ImuData &, const LtvCalibration &, const Eigen::Vector3d &, const Eigen::Vector3d &);

private:
  LtvLandmarkHistory landmark_history_;
  bool observer_started_ = false, corrected_current_ = false;
  int healthy_camera_updates_ = 0;
  LtvOptions options_;
  std::unique_ptr<ltv::LtvAvailabilityPolicy> availability_;
  ltv::LtvAvailabilityOutput availability_status_;
  bool new_epoch_pending_ = false, initial_warm_attempted_ = false;
  double last_correction_time_ = -1, camera_imu_offset_ = 0;
  uint64_t actual_corrections_ = 0;
  double prediction_angle_ = 0, velocity_correction_rate_ = 0, gravity_correction_rate_ = 0;
  bool correction_diagnostics_valid_ = false;
  LtvFrame processManaged(double, const std::vector<LtvBearing> &, const LtvCalibration &, const Eigen::Vector3d &, const Eigen::Vector3d &,
                          uint64_t, const ltv::LtvFeatureContext *);
  LtvFrame pauseManaged(double, const std::string &, uint64_t);
  void suspendManaged(double target);
  ltv::LtvObserver observer_;
  // OpenVINS bridge owns the transaction; the landmark adapter owns feature policy.
  std::unique_ptr<ltv::LtvFeatureFrontend> feature_frontend_;
  ltv::LtvFeatureFrame feature_frame_;
  std::mutex mutex_;
  std::deque<ov_core::ImuData> imu_;
  bool imu_input_fault_ = false, suspended_ = true;
  double integrated_until_ = 0, last_camera_time_ = -1;
  uint64_t epoch_ = 0, sequence_ = 0, last_claimed_sequence_ = 0, integrated_imu_steps_ = 0, last_version_ = 0;
  double integrated_imu_seconds_ = 0;
  std::map<size_t, int> feature_to_observer_id_;
  int next_observer_id_ = 0;
  LtvFrame frame(double t, uint64_t version, const std::string &reason) const;
};
} // namespace ov_msckf
