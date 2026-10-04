#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include <string>
namespace ltv {
enum class LtvAvailability { Disabled, Collecting, Bootstrapping, Tracking, Degraded, Dormant, Recovering };
const char *toString(LtvAvailability state);
struct LtvReadinessConfig {
  bool enabled = false;
  // Preserve the first finite current core trajectory, once per explicit reset.
  bool initial_unseeded_warmup = false;
  bool preserve_constrained_state = false;
  bool ready_soft_grace = false;
  size_t min_features = 15;
  unsigned int bootstrap_confirm_frames = 3, ready_confirm_frames = 20;
  unsigned int min_actual_corrections = 20;
  double coast_seconds = .20, dormant_seconds = 1.0, quality_bootstrap_interval = 30.0;
  double timestamp_tolerance = 1e-9, confirmation_max_gap = .20;
  double gravity_norm_min = 8.0, gravity_norm_max = 11.5;
  double prediction_angle_limit_rad = .02;
  double velocity_correction_rate_limit = .5, gravity_correction_rate_limit = 1.0;
};
struct LtvReadinessInput {
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
struct LtvReadinessOutput {
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
// Pure causal policy: no state/P access, no GT, no VIO-reference error. All
// mutations to the observer belong to the adapter's explicit transaction.
class LtvReadiness {
public:
  explicit LtvReadiness(const LtvReadinessConfig &config = {});
  void reset(); // Explicit parent/frontend reset only; not for low feature counts.
  LtvReadinessOutput update(const LtvReadinessInput &input);
  // Same-event ack after update() emitted request and adapter committed x/P/IDs.
  // Does not consume a second event or increment confirmation counters.
  LtvReadinessOutput acknowledgeBootstrap(const LtvReadinessInput &input);

private:
  void clearGrace();
  void validateCommit(const LtvReadinessInput &input) const;
  void commit(const LtvReadinessInput &input);
  LtvReadinessConfig config_;
  LtvAvailability state_ = LtvAvailability::Collecting;
  double last_time_ = -1, shortage_since_ = -1, last_bootstrap_ = -1, last_quality_bootstrap_ = -1;
  unsigned int supply_frames_ = 0, gravity_frames_ = 0, velocity_frames_ = 0;
  unsigned int soft_G_ = 0, soft_V_ = 0;
  double last_good_G_ = -1, last_good_V_ = -1;
  bool pending_physical_ = false, pending_initial_warmup_ = false;
  bool awaiting_bootstrap_ = false, bootstrapped_ = false, dormant_requested_ = false, physical_recovery_ = false, fault_latched_ = false;
  uint64_t bootstrap_count_ = 0, physical_fault_count_ = 0;
  std::string bootstrap_source_;
};
} // namespace ltv
