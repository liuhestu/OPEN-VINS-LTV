#pragma once
#include "ltv/landmark_adapter/LtvLandmarkAdapter.h"
#include "ltv/observer/LtvJointFactors.h"
#include "ltv/observer/ltv_observer.h"
#include <fstream>
#include <memory>
namespace ov_msckf {
struct LtvCalibration;
}
namespace ltv {
// Explicit finite-state diagnostic model: one independently configured native
// Observer slot, fixed calibration/clock, constant physical biases, whitened
// raw IMU endpoint and pixel sources, conditional main EKF H/K schedule.
// It cannot provide a 30-point production covariance by dropping other points.
class LtvFiniteJointShadow {
public:
  static LtvFiniteJointShadow &instance();
  bool enabled() const { return enabled_; }
  void initialize(const void *owner, const Eigen::MatrixXd &p);
  void reset(const void *owner, const Eigen::MatrixXd &p);
  void mainMap(const void *owner, const Eigen::MatrixXd &p, const Eigen::MatrixXd &map, const std::string &event);
  void freezeBias(const Eigen::MatrixXd &selector);
  void rawImuReceipt(const std::vector<double> &timestamps, double sigma_acc, double sigma_gyro);
  void mainImu(const Eigen::MatrixXd &f, const Eigen::MatrixXd &b_left, const Eigen::MatrixXd &b_right, double t_left, double t_right);
  void mainVisual(const Eigen::MatrixXd &h, const Eigen::MatrixXd &k, const Eigen::MatrixXd &r, const Eigen::MatrixXd &pixel_projection);
  void errorReset(const Eigen::MatrixXd &map);
  void observerImu(double dt, double t_left, double t_right, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                   const ov_msckf::LtvCalibration &calibration);
  void camera(const FeaturePipelineContext &ctx, const FeaturePipelineFrame &accepted, const LtvConfig &config,
              const ov_msckf::LtvCalibration &calibration);
  const std::shared_ptr<LtvJointFactors> &factors() const { return factors_; }

private:
  LtvFiniteJointShadow();
  Eigen::MatrixXd endpoint(double time);
  Eigen::MatrixXd pixel(const std::string &key, double variance);
  void record(double time, const char *event);
  bool enabled_ = false;
  const void *owner_ = nullptr;
  std::shared_ptr<LtvJointFactors> factors_;
  std::shared_ptr<LtvObserverSourceSensitivity> sensitivity_;
  std::unique_ptr<LtvObserver> observer_;
  std::vector<double> raw_times_;
  double sigma_acc_ = 0, sigma_gyro_ = 0;
  bool active_ = false;
  size_t feature_ = 0;
  int local_id_ = 0, next_id_ = 0;
  uint64_t epoch_ = 0;
  double time_ = -1;
  std::map<double, std::string> anchors_;
  std::map<std::string, double> source_times_;
  std::ofstream output_, matrix_index_, matrix_data_;
  unsigned long event_ = 0, missing_ = 0;
};
} // namespace ltv
