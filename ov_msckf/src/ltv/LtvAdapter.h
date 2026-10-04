#pragma once
#include "LtvFeaturePipeline.h"
#include "LtvOptions.h"
#include "ltv_observer.h"
#include "utils/sensor_data.h"
#include <deque>
#include <map>
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
  ltv::LtvReadinessOutput health;
  double prediction_angle_p95_rad = 0, velocity_correction_rate = 0, gravity_correction_rate = 0;
  bool correction_diagnostics_valid = false;
  uint64_t actual_corrections = 0;
  std::string reason = "not_started";
  ltv::LtvSnapshot snapshot;
  uint64_t integrated_steps = 0;
  double integrated_seconds = 0;
};
class LtvAdapter {
public:
  explicit LtvAdapter(const LtvOptions &options);
  void feed_imu(const ov_core::ImuData &sample);
  LtvFrame process(double camera_time, const std::vector<LtvBearing> &bearings, const LtvCalibration &calibration,
                   const Eigen::Vector3d &ba, const Eigen::Vector3d &bg, uint64_t version,
                   const ltv::FeaturePipelineContext *feature_context = nullptr);
  LtvFrame pause(double camera_time, const std::string &reason, uint64_t version);
  void reset();
  bool claim(const LtvFrame &frame);
  const ltv::LtvObserver &core() const { return observer_; }
  const ltv::FeaturePipelineFrame &feature_frame() const { return feature_frame_; }
  const ltv::LtvFeaturePipeline *feature_pipeline() const { return feature_pipeline_.get(); }
  const std::map<size_t, int> &feature_ids() const { return ids_; }
  static ov_core::ImuData correct(const ov_core::ImuData &, const LtvCalibration &, const Eigen::Vector3d &, const Eigen::Vector3d &);

private:
  LtvOptions options_;
  std::unique_ptr<ltv::LtvReadiness> readiness_;
  ltv::LtvReadinessOutput health_;
  bool hardening_new_epoch_pending_ = false, initial_warm_attempted_ = false;
  double last_correction_time_ = -1, hardening_offset_ = 0;
  uint64_t actual_corrections_ = 0;
  double prediction_angle_ = 0, velocity_correction_rate_ = 0, gravity_correction_rate_ = 0;
  bool correction_diagnostics_valid_ = false;
  LtvFrame processHardened(double, const std::vector<LtvBearing> &, const LtvCalibration &, const Eigen::Vector3d &,
                           const Eigen::Vector3d &, uint64_t, const ltv::FeaturePipelineContext *);
  LtvFrame pauseHardened(double, const std::string &, uint64_t);
  void suspendHardened(double target);
  ltv::LtvObserver observer_;
  std::unique_ptr<ltv::LtvFeaturePipeline> feature_pipeline_;
  ltv::FeaturePipelineFrame feature_frame_;
  std::mutex mutex_;
  std::deque<ov_core::ImuData> imu_;
  bool input_fault_ = false, paused_ = true;
  double cursor_ = 0, last_camera_ = -1;
  uint64_t epoch_ = 0, sequence_ = 0, claimed_ = 0, steps_ = 0, last_version_ = 0;
  double seconds_ = 0;
  std::map<size_t, int> ids_;
  int next_id_ = 0;
  LtvFrame frame(double t, uint64_t version, const std::string &reason) const;
};
} // namespace ov_msckf
