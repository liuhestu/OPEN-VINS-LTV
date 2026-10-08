#pragma once

#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ltv {

// A joint error map, deliberately separate from the observer's Riccati metric.
// Layout and source covariance must be supplied explicitly by the caller. The
// cross covariance D is Cov(old joint error, source); no implicit independence.
inline Eigen::MatrixXd shadowJointMap(const Eigen::MatrixXd &sigma, const Eigen::MatrixXd &map, const Eigen::MatrixXd &source_map,
                                      const Eigen::MatrixXd &source_covariance, const Eigen::MatrixXd &error_source_cross) {
  if (sigma.rows() != sigma.cols() || map.cols() != sigma.rows() || source_map.rows() != map.rows() ||
      source_map.cols() != source_covariance.rows() || source_covariance.rows() != source_covariance.cols() ||
      error_source_cross.rows() != sigma.rows() || error_source_cross.cols() != source_map.cols())
    throw std::invalid_argument("shadow joint map dimension mismatch");
  return map * sigma * map.transpose() + source_map * source_covariance * source_map.transpose() +
         map * error_source_cross * source_map.transpose() + source_map * error_source_cross.transpose() * map.transpose();
}

// These derivatives hold the Riccati gain schedule fixed. They describe the
// actual Euler mean map, not a statistical Sigma_L. Gain/P_R sensitivities and
// historic source/main-error blocks remain mandatory for an unconditional model.
class LtvErrorShadow {
public:
  LtvErrorShadow() {
    const char *path = std::getenv("LTV_JOINT_SHADOW_PATH");
    if (!path || !*path)
      return;
    // Hardened adapter copies/replaces observers transactionally. Copies share
    // the sink; constructing an epoch replacement must not truncate its log.
    static std::map<std::string, std::weak_ptr<std::ofstream>> sinks;
    output_ = sinks[path].lock();
    if (output_)
      return;
    std::ifstream existing(path);
    if (existing.good())
      throw std::runtime_error("refusing joint shadow overwrite");
    output_ = std::make_shared<std::ofstream>(path);
    sinks[path] = output_;
    if (!*output_)
      throw std::runtime_error("cannot open joint shadow output");
    *output_ << std::setprecision(17)
             << "event,time,epoch,dimension,imu_steps,births,retired,substeps,transition_norm,source_map_norm,"
                "conditional_unit_source_gram_trace,persistent_corrected_input_offset_norm,local_feature_ids,valid_unconditional,missing_"
                "blocks\n";
  }
  bool enabled() const { return output_ && output_->is_open(); }
  const Eigen::MatrixXd &imuSourceMap() const { return imu_b_; }
  const Eigen::MatrixXd &cameraTransition() const { return camera_f_; }
  const Eigen::MatrixXd &cameraSourceMap() const { return camera_b_; }
  void setEpoch(unsigned long epoch) { epoch_ = epoch; }
  void reset(double time) {
    if (!enabled())
      return;
    ++epoch_;
    imu_steps_ = 0;
    persistent_input_offset_ = Eigen::MatrixXd::Zero(6, 6);
    write("reset", time, 6, 0, 0, 0, 0, 0);
  }
  void imu(const Eigen::MatrixXd &f, const Eigen::MatrixXd &b) {
    if (!enabled())
      return;
    // One persistent corrected-input offset source, NOT a fresh bias draw per
    // IMU step. Birth seed is held fixed here; its true offset dependency is
    // UNKNOWN and prevents statistical validity. Units are (accel, gyro).
    if (persistent_input_offset_.rows() != f.cols())
      throw std::runtime_error("shadow input sensitivity lifecycle mismatch");
    persistent_input_offset_ = (f * persistent_input_offset_ - b).eval();
    imu_b_ = b;
    ++imu_steps_;
    last_imu_f_norm_ = f.norm();
    last_imu_b_norm_ = b.norm();
  }
  void lifecycle(double time, const Eigen::MatrixXd &map, int births, int retired) {
    if (enabled()) {
      persistent_input_offset_ = (map * persistent_input_offset_).eval();
      write("lifecycle", time, map.rows(), births, retired, 0, map.norm(), 0);
    }
  }
  void camera(double time, const Eigen::MatrixXd &f, const Eigen::MatrixXd &b, int substeps, const std::vector<int> &ids) {
    if (enabled()) {
      // The isolated same-frame source contribution is actually propagated with
      // the joint-map implementation. Unit tangent-source covariance measures
      // sensitivity only: it is explicitly NOT physical bearing uncertainty.
      // Unknown prior/source and main/source blocks are never filled with zeros
      // in an unconditional distribution.
      const Eigen::MatrixXd unit_source = Eigen::MatrixXd::Identity(b.cols(), b.cols());
      const Eigen::MatrixXd fixed_initial = Eigen::MatrixXd::Zero(f.cols(), f.cols());
      const Eigen::MatrixXd fixed_initial_cross = Eigen::MatrixXd::Zero(f.cols(), b.cols());
      persistent_input_offset_ = (f * persistent_input_offset_).eval();
      source_ids_.clear();
      for (int id : ids)
        source_ids_ += std::to_string(id) + ";";
      camera_f_ = f;
      camera_b_ = b;
      frame_source_gram_ = shadowJointMap(fixed_initial, f, b, unit_source, fixed_initial_cross);
      write("camera", time, f.rows(), 0, 0, substeps, f.norm(), b.norm());
      write("imu_summary", time, f.rows(), 0, 0, 0, last_imu_f_norm_, last_imu_b_norm_);
      output_->flush();
    }
  }

private:
  void write(const char *event, double time, int dimension, int births, int retired, int substeps, double f_norm, double b_norm) {
    *output_ << event << ',' << time << ',' << epoch_ << ',' << dimension << ',' << imu_steps_ << ',' << births << ',' << retired << ','
             << substeps << ',' << f_norm << ',' << b_norm << "," << (std::string(event) == "camera" ? frame_source_gram_.trace() : 0.0)
             << "," << persistent_input_offset_.norm() << "," << (std::string(event) == "camera" ? source_ids_ : "")
             << ",false,seed_history_joint;main_visual_reset_cross;interpolated_imu_source_cross;gain_schedule_sensitivity;independent_"
                "landmark_truth\n";
    if (!*output_)
      throw std::runtime_error("joint shadow write failed");
  }
  std::string source_ids_;
  Eigen::MatrixXd frame_source_gram_, camera_f_, camera_b_, imu_b_;
  Eigen::MatrixXd persistent_input_offset_ = Eigen::MatrixXd::Zero(6, 6);
  std::shared_ptr<std::ofstream> output_;
  unsigned long epoch_ = 0, imu_steps_ = 0;
  double last_imu_f_norm_ = 0, last_imu_b_norm_ = 0;
};

} // namespace ltv
