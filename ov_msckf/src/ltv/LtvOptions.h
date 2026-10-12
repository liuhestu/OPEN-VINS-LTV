#pragma once
#include "ltv/LtvObserver.h"
#include "ltv/features/LtvFeatureTypes.h"
#include "state/StateOptions.h"
#include <stdexcept>
namespace ltv {
struct LtvAvailabilityConfig {
  bool enabled = true;
  // Preserve the first finite current core trajectory, once per explicit reset.
  bool initial_unseeded_warmup = true;
  bool preserve_constrained_state = true;
  bool ready_soft_grace = true;
  size_t min_features = 15;
  unsigned int bootstrap_confirm_frames = 3, ready_confirm_frames = 20;
  unsigned int min_actual_corrections = 20;
  double coast_seconds = .20, dormant_seconds = 1.0, quality_bootstrap_interval = 30.0;
  double timestamp_tolerance = 1e-9, confirmation_max_gap = .20;
  double gravity_norm_min = 8.0, gravity_norm_max = 11.5;
  double prediction_angle_limit_rad = .02;
  double velocity_correction_rate_limit = .5, gravity_correction_rate_limit = 1.0;
};
} // namespace ltv
namespace ov_msckf {
struct LtvOptions {
  std::string mode = "OFF", scene = "MH", profile_path;
  double weight_G = 1, weight_V = 1, weight_L = 1;
  bool enabled = false, enable_gravity = false, enable_velocity = false, enable_landmark_approx = false;
  bool allow_correlated_pseudomeasurements = false;
  bool enable_quality_gate = false, enable_nis_gate = true, enable_huber = false;
  bool log_enabled = false, feature_apply_seed = true;
  int landmark_approx_max_points = 2, quality_min_features = 25;
  double landmark_approx_sigma_floor_m = .5, landmark_approx_information_cap = .01;
  double feature_bearing_sigma_rad = 0.0008726646259971648;
  double huber_delta = 2, sigma_gravity_deg = 10, sigma_velocity_mps = 1, max_gravity_angle_deg = 30, time_tolerance_s = 1e-6;
  double nis_gravity = 9.21034037197618, nis_velocity = 11.3448667301444;
  double quality_eta_error = .5, quality_innovation = .03, quality_velocity_disagreement = .5;
  std::string log_path = "ltv.csv", feature_seed_source = "STEREO_THEN_TEMPORAL";
  int min_features = 15, warmup_camera_updates = 20;
  double max_imu_dt = .05, reset_gap = .2, gravity_norm_min = 7, gravity_norm_max = 12;
  ltv::LtvConfig observer;
  ltv::LtvAvailabilityConfig health_options;
  ltv::LtvFeatureManagerConfig feature_manager;
  ltv::ActiveConsistencyConfig active_consistency;
  LtvOptions() { active_consistency.enabled = true; }
  void load_profile(const std::string &path, const std::string &override_mode = "", const std::string &override_scene = "");
  void load(const std::shared_ptr<ov_core::YamlParser> &p) {
    if (!p)
      return;
#define FIELD(x) p->parse_config("ltv_" #x, x, false)
    FIELD(enable_quality_gate);
    FIELD(enable_nis_gate);
    FIELD(enable_huber);
    FIELD(huber_delta);
    FIELD(log_enabled);
    FIELD(log_path);
    FIELD(max_gravity_angle_deg);
    FIELD(time_tolerance_s);
    FIELD(nis_gravity);
    FIELD(nis_velocity);
    FIELD(quality_eta_error);
    FIELD(quality_innovation);
    FIELD(quality_velocity_disagreement);
    FIELD(quality_min_features);
    FIELD(feature_apply_seed);
    FIELD(feature_seed_source);
    FIELD(feature_bearing_sigma_rad);
    FIELD(landmark_approx_max_points);
#undef FIELD
    int history_frames = static_cast<int>(active_consistency.min_history_frames);
    p->parse_config("ltv_active_consistency_min_history_frames", history_frames, false);
    if (history_frames < 1)
      throw std::invalid_argument("Invalid active consistency history frames");
    active_consistency.min_history_frames = static_cast<size_t>(history_frames);
    p->parse_config("ltv_health_prediction_angle_limit_rad", health_options.prediction_angle_limit_rad, false);
    p->parse_config("ltv_health_velocity_correction_rate_limit", health_options.velocity_correction_rate_limit, false);
    p->parse_config("ltv_health_gravity_correction_rate_limit", health_options.gravity_correction_rate_limit, false);
    p->parse_config("ltv_active_consistency_enabled", active_consistency.enabled, false);
    p->parse_config("ltv_active_consistency_min_history_span_s", active_consistency.min_history_span_s, false);
    p->parse_config("ltv_active_consistency_max_history_residual_rad", active_consistency.max_history_residual_rad, false);
    p->parse_config("ltv_active_consistency_max_holdout_residual_rad", active_consistency.max_holdout_residual_rad, false);
    p->parse_config("ltv_active_consistency_retire_after_consecutive_failures", active_consistency.retire_after_consecutive_failures,
                    false);
    p->parse_config("ltv_feature_max_relative_risk", feature_manager.quality.max_relative_risk, false);
    p->parse_config("ltv_feature_min_history_span", feature_manager.quality.min_history_span, false);
    p->parse_config("ltv_feature_maturity_seconds", feature_manager.maturity_seconds, false);
#define CORE(x) p->parse_config("ltv_observer_" #x, observer.x, false)
    CORE(max_features);
    p->parse_config("ltv_observer_min_features", min_features, false);
    p->parse_config("ltv_observer_warmup_camera_updates", warmup_camera_updates, false);
    p->parse_config("ltv_observer_max_imu_dt", max_imu_dt, false);
    p->parse_config("ltv_observer_reset_gap", reset_gap, false);
    CORE(q_landmark);
    CORE(v_landmark);
    CORE(v_velocity);
    CORE(v_gravity);
    CORE(initial_p_landmark);
    CORE(initial_p_velocity);
    CORE(initial_p_gravity);
    p->parse_config("ltv_observer_gravity_norm_min", gravity_norm_min, false);
    p->parse_config("ltv_observer_gravity_norm_max", gravity_norm_max, false);
#undef CORE
    std::string path, override_mode, override_scene;
    p->parse_config("ltv_config", path, false);
    p->parse_config("ltv_mode", override_mode, false);
    p->parse_config("ltv_scene", override_scene, false);
    load_profile(path, override_mode, override_scene);
  }
  void validate(const StateOptions &state) const {
    if (!enabled)
      return;
    if (state.do_calib_camera_pose || state.do_calib_camera_intrinsics || state.do_calib_camera_timeoffset ||
        state.do_calib_imu_intrinsics || state.do_calib_imu_g_sensitivity)
      throw std::invalid_argument("LTV requires fixed calibration; use config:=euroc_ltv");
    if (enable_landmark_approx && state.max_slam_features != 0)
      throw std::invalid_argument("LTV landmark fusion requires max_slam=0; use config:=euroc_ltv");
    if (state.num_cameras < 1 || state.num_cameras > 2 || (feature_seed_source != "TEMPORAL_POSE" && state.num_cameras != 2))
      throw std::invalid_argument("LTV seed source is incompatible with camera count");
    if (feature_seed_source != "TEMPORAL_POSE" && feature_seed_source != "STEREO" && feature_seed_source != "STEREO_THEN_TEMPORAL")
      throw std::invalid_argument("Invalid LTV seed source");
    if (min_features < 1 || observer.max_features < min_features || warmup_camera_updates < 1 || landmark_approx_max_points < 1 ||
        landmark_approx_max_points > 2 || quality_min_features < 1)
      throw std::invalid_argument("Invalid LTV counts");
    for (double x : {sigma_gravity_deg,
                     sigma_velocity_mps,
                     time_tolerance_s,
                     max_gravity_angle_deg,
                     nis_gravity,
                     nis_velocity,
                     huber_delta,
                     feature_bearing_sigma_rad,
                     landmark_approx_sigma_floor_m,
                     landmark_approx_information_cap,
                     max_imu_dt,
                     reset_gap,
                     observer.q_landmark,
                     observer.v_landmark,
                     observer.v_velocity,
                     observer.v_gravity,
                     observer.initial_p_landmark,
                     observer.initial_p_velocity,
                     observer.initial_p_gravity,
                     gravity_norm_min,
                     gravity_norm_max,
                     quality_eta_error,
                     quality_innovation,
                     quality_velocity_disagreement,
                     health_options.prediction_angle_limit_rad,
                     health_options.velocity_correction_rate_limit,
                     health_options.gravity_correction_rate_limit})
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("Invalid LTV scalar");
    if (max_gravity_angle_deg >= 90 || gravity_norm_min >= gravity_norm_max)
      throw std::invalid_argument("Invalid LTV physical gate");
  }
  void print() const {
    PRINT_INFO("[LTV] mode=%s scene=%s weights G=%.6g V=%.6g L=%.6g\n", mode.c_str(), scene.c_str(), weight_G, weight_V, weight_L);
    if (enabled)
      PRINT_INFO("[LTV] sigma_G=%.6g deg sigma_V=%.6g m/s L_floor=%.6g m L_cap=%.6g\n", sigma_gravity_deg, sigma_velocity_mps,
                 landmark_approx_sigma_floor_m, landmark_approx_information_cap);
  }
};
} // namespace ov_msckf
