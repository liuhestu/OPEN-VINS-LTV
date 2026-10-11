#pragma once
#include "ltv/runtime/LtvRuntime.h"
#include "state/State.h"
namespace ov_msckf {
struct LtvDiagnostics {
  std::string gravity_reason = "disabled", velocity_reason = "disabled", landmark_reason = "disabled", submit_reason = "none";
  bool gravity_eligible = false, velocity_eligible = false, gravity_quality = false, velocity_quality = false, consumed = false;
  double gravity_nis = -1, velocity_nis = -1, landmark_nis = -1, gv_nis = -1, joint_nis = -1;
  double gravity_residual = 0, velocity_residual = 0, gravity_variance = 0, velocity_variance = 0;
  double prior_rotation_difference = 0, prior_velocity_difference = 0, compute_time_ms = 0;
  Eigen::VectorXd gravity_residual_vector, velocity_residual_vector;
  int landmark_rows = 0, gravity_rows = 0, velocity_rows = 0, visual_rows = 0, columns = 0, ekf_calls = 0;
};
struct LtvReceipt {
  bool consumed = false;
  LtvDiagnostics diagnostics;
};
struct MeasurementBlock {
  std::vector<std::shared_ptr<ov_type::Type>> order;
  Eigen::MatrixXd H = Eigen::MatrixXd(0, 0), R = Eigen::MatrixXd(0, 0);
  Eigen::VectorXd res;
  LtvFrame token;
  std::shared_ptr<LtvReceipt> receipt;
  bool empty() const { return res.size() == 0; }
};
// Shared type-column mapping and numerical validation; no state mutation.
MeasurementBlock merge_measurements(const MeasurementBlock &, const MeasurementBlock &);
bool validate_measurement(const std::shared_ptr<State> &, const MeasurementBlock &, std::string &, double *nis_value = nullptr);
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
  MeasurementBlock build(const std::shared_ptr<State> &, const LtvFrame &, MeasurementBlock *diagnostic = nullptr) const;
  static Eigen::Vector3d landmark_prediction(const ov_type::PoseJPL &, const ov_type::PoseJPL &, const Eigen::Vector3d &, bool fej = false);
  static Eigen::Matrix<double, 3, 12> landmark_jacobian(const ov_type::PoseJPL &, const ov_type::PoseJPL &, const Eigen::Vector3d &, bool);

private:
  MeasurementBlock landmark(const std::shared_ptr<State> &, const LtvFrame &, MeasurementBlock) const;
  LtvOptions options_;
  double gravity_mag_;
};
} // namespace ov_msckf
