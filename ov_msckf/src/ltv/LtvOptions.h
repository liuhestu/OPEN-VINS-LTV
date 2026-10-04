#pragma once
#include "LtvLandmarkManager.h"
#include "LtvReadiness.h"
#include "ltv_types.h"
#include "state/StateOptions.h"
#include <stdexcept>
namespace ov_msckf {
struct LtvOptions {
  bool enabled = false, enable_gravity = false, enable_velocity = false, allow_correlated_pseudomeasurements = false;
  bool enable_quality_gate = false, enable_nis_gate = true, log_enabled = false;
  bool enable_huber = false;
  bool feature_readiness_enabled = false, feature_apply_seed = true, passive_audit_enabled = false;
  bool passive_hardening_enabled = false, hardening_health_readiness = true, hardening_initial_warmup = false;
  ltv::LtvReadinessConfig hardening_readiness;
  std::string feature_seed_source = "TEMPORAL_POSE";
  std::string passive_cache_path;
  double feature_bearing_sigma_rad = 0.0008726646259971648;
  ltv::LandmarkManagerConfig feature_manager;
  double huber_delta = 2.0;
  double sigma_gravity_deg = 10, sigma_velocity_mps = 1, max_gravity_angle_deg = 30, time_tolerance_s = 1e-6;
  double nis_gravity = 9.21034037197618, nis_velocity = 11.3448667301444;
  double quality_eta_error = 0.5, quality_innovation = 0.03, quality_velocity_disagreement = 0.5;
  int quality_min_features = 25;
  std::string log_path = "ltv.csv", correlation_model = "independence_approximation";
  bool value_diagnostics_enabled = false, value_diagnostics_matrices = false;
  std::string value_diagnostics_path = "value.jsonl.gz";
  ltv::LtvConfig observer;
  void load(const std::shared_ptr<ov_core::YamlParser> &p) {
    if (!p)
      return;
#define FIELD(x) p->parse_config("ltv_" #x, x, false)
    FIELD(value_diagnostics_enabled);
    FIELD(feature_readiness_enabled);
    FIELD(passive_hardening_enabled);
    FIELD(hardening_health_readiness);
    FIELD(hardening_initial_warmup);
    FIELD(feature_apply_seed);
    FIELD(passive_audit_enabled);
    FIELD(feature_seed_source);
    FIELD(passive_cache_path);
    FIELD(feature_bearing_sigma_rad);
    FIELD(value_diagnostics_matrices);
    FIELD(value_diagnostics_path);
    FIELD(enabled);
    FIELD(enable_gravity);
    FIELD(enable_velocity);
    FIELD(allow_correlated_pseudomeasurements);
    FIELD(enable_quality_gate);
    FIELD(enable_nis_gate);
    FIELD(enable_huber);
    FIELD(huber_delta);
    FIELD(log_enabled);
    FIELD(log_path);
    FIELD(correlation_model);
    FIELD(sigma_gravity_deg);
    FIELD(sigma_velocity_mps);
    FIELD(max_gravity_angle_deg);
    FIELD(time_tolerance_s);
    FIELD(nis_gravity);
    FIELD(nis_velocity);
    FIELD(quality_eta_error);
    FIELD(quality_innovation);
    FIELD(quality_velocity_disagreement);
    FIELD(quality_min_features);
#undef FIELD
    p->parse_config("ltv_hardening_prediction_angle_limit_rad", hardening_readiness.prediction_angle_limit_rad, false);
    p->parse_config("ltv_hardening_velocity_correction_rate_limit", hardening_readiness.velocity_correction_rate_limit, false);
    p->parse_config("ltv_hardening_gravity_correction_rate_limit", hardening_readiness.gravity_correction_rate_limit, false);
    p->parse_config("ltv_feature_max_relative_risk", feature_manager.quality.max_relative_risk, false);
    p->parse_config("ltv_feature_min_history_span", feature_manager.quality.min_history_span, false);
    p->parse_config("ltv_feature_maturity_seconds", feature_manager.maturity_seconds, false);
#define CORE(x) p->parse_config("ltv_observer_" #x, observer.x, false)
    CORE(max_features);
    CORE(min_features);
    CORE(max_missed_frames);
    CORE(warmup_camera_updates);
    CORE(max_imu_dt);
    CORE(reset_gap);
    CORE(timestamp_tolerance);
    CORE(q_landmark);
    CORE(v_landmark);
    CORE(v_velocity);
    CORE(v_gravity);
    CORE(initial_p_landmark);
    CORE(initial_p_velocity);
    CORE(initial_p_gravity);
    CORE(covariance_floor);
    CORE(covariance_failure_threshold);
    CORE(camera_euler_safety);
    CORE(max_camera_substeps);
    CORE(gravity_norm_min);
    CORE(gravity_norm_max);
#undef CORE
  }
  void validate(const StateOptions &state) const {
    if (passive_hardening_enabled && (!feature_readiness_enabled || !enabled || enable_gravity || enable_velocity))
      throw std::invalid_argument("Passive hardening requires managed Passive LTV");
    if (feature_readiness_enabled) {
      if (!enabled || enable_gravity || enable_velocity)
        throw std::invalid_argument("Feature readiness requires Passive LTV with G/V injection OFF");
      if (feature_seed_source != "TEMPORAL_POSE" && feature_seed_source != "STEREO" && feature_seed_source != "STEREO_THEN_TEMPORAL")
        throw std::invalid_argument("Unsupported feature seed source");
      if (feature_seed_source != "TEMPORAL_POSE" && state.num_cameras != 2)
        throw std::invalid_argument("Stereo seeds require two existing cameras");
      if (observer.q_landmark != 1e-4 || observer.v_landmark != 1e6 || observer.v_velocity != 1e6 || observer.v_gravity != 1e6 ||
          observer.initial_p_landmark != 1 || observer.initial_p_velocity != 1 || observer.initial_p_gravity != 1 ||
          observer.max_features != 30 || observer.min_features != 15)
        throw std::invalid_argument("Feature readiness study requires frozen C0 and 30/15 capacity");
      if (!std::isfinite(feature_bearing_sigma_rad) || feature_bearing_sigma_rad <= 0)
        throw std::invalid_argument("Invalid feature bearing uncertainty");
    }
    if (passive_audit_enabled && (enable_gravity || enable_velocity))
      throw std::invalid_argument("Passive audit forbids auxiliary injection");
    if ((enable_gravity || enable_velocity) && (!enabled || !allow_correlated_pseudomeasurements))
      throw std::invalid_argument("LTV auxiliary requires enabled and explicit correlated pseudomeasurement acceptance");
    if (!enabled)
      return;
    if (state.do_calib_camera_pose || state.do_calib_camera_intrinsics || state.do_calib_camera_timeoffset ||
        state.do_calib_imu_intrinsics || state.do_calib_imu_g_sensitivity)
      throw std::invalid_argument("UNSUPPORTED LTV online calibration");
    if (state.num_cameras < 1 || state.num_cameras > 2)
      throw std::invalid_argument("UNSUPPORTED LTV asynchronous/multiple cameras");
    auto positive = [](double x) { return std::isfinite(x) && x > 0; };
    if (correlation_model != "independence_approximation" || !positive(sigma_gravity_deg) || !positive(sigma_velocity_mps) ||
        !positive(time_tolerance_s) || !positive(max_gravity_angle_deg) || max_gravity_angle_deg >= 90 || !positive(nis_gravity) ||
        !positive(nis_velocity) || !positive(huber_delta))
      throw std::invalid_argument("invalid LTV fusion settings");
    if (observer.max_features < 1 || observer.min_features < 1 || observer.min_features > observer.max_features ||
        observer.max_missed_frames < 0 || observer.warmup_camera_updates < 1 || observer.max_camera_substeps < 1)
      throw std::invalid_argument("invalid LTV observer counts");
    for (double x :
         {observer.max_imu_dt, observer.reset_gap, observer.timestamp_tolerance, observer.q_landmark, observer.v_landmark,
          observer.v_velocity, observer.v_gravity, observer.initial_p_landmark, observer.initial_p_velocity, observer.initial_p_gravity,
          observer.covariance_floor, observer.camera_euler_safety, observer.gravity_norm_min, observer.gravity_norm_max})
      if (!positive(x))
        throw std::invalid_argument("invalid LTV observer scalar");
    if (!std::isfinite(observer.covariance_failure_threshold) || observer.covariance_failure_threshold >= 0 ||
        observer.gravity_norm_min >= observer.gravity_norm_max || quality_min_features < 1 || !positive(quality_eta_error) ||
        !positive(quality_innovation) || !positive(quality_velocity_disagreement))
      throw std::invalid_argument("invalid LTV spectral/quality settings");
  }
};
} // namespace ov_msckf
