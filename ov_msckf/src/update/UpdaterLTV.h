#pragma once
#include "ltv/LtvAdapter.h"
#include "state/State.h"
namespace ov_msckf {
struct LtvDiagnostics {
  std::string gravity_reason = "disabled", velocity_reason = "disabled", submit_reason = "none";
  bool gravity_eligible = false, velocity_eligible = false, gravity_quality = false, velocity_quality = false;
  double gravity_nis = -1, velocity_nis = -1, gravity_residual = 0, velocity_residual = 0, update_norm = 0;
  double gravity_variance = 0, velocity_variance = 0, p_symmetry = 0, p_min_eigenvalue = 0;
  double gravity_update_norm = 0, velocity_update_norm = 0, gravity_gain_norm = 0, velocity_gain_norm = 0;
  double prior_rotation_difference = 0, prior_velocity_difference = 0;
  bool p_checked = false;
  int gravity_rows = 0, velocity_rows = 0, visual_rows = 0, columns = 0, ekf_calls = 0;
};
// Receipt owns consumption metadata only. Numerical block and frozen prior are never mutated.
struct LtvReceipt {
  bool consumed = false;
  LtvDiagnostics diagnostics;
};
struct MeasurementBlock {
  std::vector<std::shared_ptr<ov_type::Type>> order;
  Eigen::MatrixXd H, R;
  Eigen::VectorXd res;
  LtvFrame token;
  std::shared_ptr<LtvReceipt> receipt;
  Eigen::MatrixXd prior_P;
  Eigen::VectorXd prior_imu, prior_fej;
  std::map<double, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> prior_clones;
  const State *state_identity = nullptr;
  bool empty() const { return res.size() == 0; }
};
struct LtvContext {
  Eigen::Matrix3d R_current, R_linearization;
  Eigen::Vector3d v_current, v_linearization, gamma = Eigen::Vector3d(0, 0, -1);
  Eigen::Matrix<double, 3, 2> T;
};
class UpdaterLTV {
public:
  explicit UpdaterLTV(const LtvOptions &options, double gravity_mag = 9.81) : options_(options), gravity_mag_(gravity_mag) {}
  LtvContext context(const State &state) const;
  // Pure un-gated model functions; fixed T is retained for finite differences.
  MeasurementBlock gravity(const State &, const LtvContext &, const ltv::LtvSnapshot &) const;
  MeasurementBlock velocity(const State &, const LtvContext &, const ltv::LtvSnapshot &) const;
  MeasurementBlock build(const std::shared_ptr<State> &state, const LtvFrame &snapshot) const;
  static MeasurementBlock merge(const MeasurementBlock &, const MeasurementBlock &);
  static bool validate(const std::shared_ptr<State> &, const MeasurementBlock &, std::string &reason);
  // Returns false only for an already consumed receipt; never retries its auxiliary.
  static bool apply_joint(const std::shared_ptr<State> &, const MeasurementBlock &visual, const MeasurementBlock &auxiliary);

private:
  LtvOptions options_;
  double gravity_mag_;
};
} // namespace ov_msckf
