#pragma once
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
                   const Eigen::Vector3d &ba, const Eigen::Vector3d &bg, uint64_t version);
  LtvFrame pause(double camera_time, const std::string &reason, uint64_t version);
  void reset();
  bool claim(const LtvFrame &frame);
  const ltv::LtvObserver &core() const { return observer_; }
  static ov_core::ImuData correct(const ov_core::ImuData &, const LtvCalibration &, const Eigen::Vector3d &, const Eigen::Vector3d &);

private:
  LtvOptions options_;
  ltv::LtvObserver observer_;
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
