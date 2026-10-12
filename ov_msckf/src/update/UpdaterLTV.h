#pragma once
#include "ltv/LtvOptions.h"
#include "ltv/features/LtvLandmarkHistory.h"
namespace ltv {
enum class LtvAvailability { Disabled, Collecting, Bootstrapping, Tracking, Degraded, Dormant, Recovering };
const char *toString(LtvAvailability state);
struct LtvAvailabilityInput {
  double time = 0, observer_time = 0;
  bool clock_valid = true, physical_input_fault = false;
  size_t eligible_seeds = 0, observed_features = 0, mature_observed_features = 0;
  bool geometry_valid = false;
  bool observer_started = false, observer_finite = false;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(), gravity = Eigen::Vector3d::Zero();
  unsigned int actual_corrections = 0;
  // Prediction residual is computed from current measurement and predicted state,
  // before this packet's correction; rates refer only to correction delta / dt.
  bool correction_diagnostics_valid = false;
  double prediction_angle_p95_rad = 0, velocity_correction_rate = 0, gravity_correction_rate = 0;
  // Legacy delayed-ack marker is rejected by update(). Use same-event
  // acknowledgeBootstrap() after a successful adapter transaction.
  bool bootstrap_committed = false, bootstrap_was_physical_recovery = false;
  std::string bootstrap_source;
};
struct LtvAvailabilityOutput {
  LtvAvailability state = LtvAvailability::Disabled;
  bool accepted_time = false, pool_ready = false, observer_valid = false;
  bool ready_G = false, ready_V = false, joint_ready = false;
  bool request_bootstrap = false, bootstrap_is_physical_recovery = false;
  bool request_dormant = false;
  bool ready_soft_grace_enabled = false, grace_G = false, grace_V = false;
  unsigned int soft_failure_frames_G = 0, soft_failure_frames_V = 0;
  double last_strict_good_G = -1, last_strict_good_V = -1;
  double last_bootstrap_time = -1;
  std::string last_bootstrap_source, reason = "disabled";
  uint64_t bootstrap_count = 0, physical_fault_count = 0;
};
} // namespace ltv

#include "state/State.h"
#include "update/UpdaterHelper.h"
namespace ov_msckf {
// One camera event, containing observer values and pure-value L candidates.
struct LtvFrame {
  uint64_t epoch = 0, sequence = 0;
  double camera_time = 0, imu_time = 0, compute_time_ms = 0;
  bool available = false, valid = false, ready_G = false, ready_V = false;
  bool gravity_valid = false, velocity_valid = false;
  std::string reason = "not_started";
  ltv::LtvSnapshot snapshot;
  std::vector<ltv::LtvLandmarkCandidate> landmarks;
};
struct LtvDiagnostics {
  std::string gravity_reason = "disabled", velocity_reason = "disabled", landmark_reason = "disabled", submit_reason = "none";
  bool gravity_eligible = false, velocity_eligible = false, gravity_quality = false, velocity_quality = false, consumed = false;
  double gravity_nis = -1, velocity_nis = -1, landmark_nis = -1, gv_nis = -1, joint_nis = -1;
  double gravity_residual = 0, velocity_residual = 0, gravity_variance = 0, velocity_variance = 0;
  double prior_rotation_difference = 0, prior_velocity_difference = 0, compute_time_ms = 0;
  Eigen::VectorXd gravity_residual_vector, velocity_residual_vector;
  int landmark_rows = 0, gravity_rows = 0, velocity_rows = 0, visual_rows = 0, columns = 0, ekf_calls = 0;
};
struct LtvContext {
  Eigen::Matrix3d R_current, R_linearization;
  Eigen::Vector3d v_current, v_linearization, gamma = Eigen::Vector3d(0, 0, -1);
  Eigen::Matrix<double, 3, 2> T;
};
// Thin G/V/L measurement model. Only the host MSCKF calls EKFUpdate.
class UpdaterLTV {
public:
  explicit UpdaterLTV(const LtvOptions &options, double gravity_mag = 9.81) : options_(options), gravity_mag_(gravity_mag) {}
  LtvContext context(const State &) const;
  MeasurementBlock gravity(const State &, const LtvContext &, const ltv::LtvSnapshot &) const;
  MeasurementBlock velocity(const State &, const LtvContext &, const ltv::LtvSnapshot &) const;
  MeasurementBlock build(const std::shared_ptr<State> &, const LtvFrame &, LtvDiagnostics *diagnostic = nullptr) const;
  static Eigen::Vector3d landmark_prediction(const ov_type::PoseJPL &, const ov_type::PoseJPL &, const Eigen::Vector3d &, bool fej = false);
  static Eigen::Matrix<double, 3, 12> landmark_jacobian(const ov_type::PoseJPL &, const ov_type::PoseJPL &, const Eigen::Vector3d &, bool);

  void reset_health();
  void health_diagnostics(ltv::LtvAvailabilityOutput &) const;
  std::string update_health(const ltv::LtvAvailabilityInput &, ltv::LtvAvailabilityOutput &);

private:
  unsigned int gravity_frames_ = 0, velocity_frames_ = 0, soft_G_ = 0, soft_V_ = 0;
  double last_good_G_ = -1, last_good_V_ = -1;
  MeasurementBlock landmark(const std::shared_ptr<State> &, const LtvFrame &, MeasurementBlock, LtvDiagnostics &) const;
  LtvOptions options_;
  double gravity_mag_;
};
} // namespace ov_msckf
