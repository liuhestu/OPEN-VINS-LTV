#include "core/VioManager.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "state/Propagator.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include <algorithm>
#include <chrono>
using namespace ov_msckf;
ltv::LtvFeatureContext VioManager::ltv_feature_context(double camera_time) const {
  const double offset = state->_calib_dt_CAMtoIMU->value()(0);
  const Eigen::Matrix3d R_BC = state->_calib_IMUtoCAM.at(0)->Rot().transpose();
  const Eigen::Vector3d p_BC = -R_BC * state->_calib_IMUtoCAM.at(0)->pos();
  const auto features = trackFEATS->get_feature_database()->features_containing(camera_time);
  ltv::LtvFeatureContext feature_context;
  feature_context.pixel_noise_variance = params.msckf_options.sigma_pix_sq;
  {
    feature_context.version = ltv_state_version;
    feature_context.time = camera_time + offset;
    std::vector<std::shared_ptr<ov_type::Type>> pose_order;
    bool execution_pose_found = false;
    for (const auto &entry : state->_clones_IMU) {
      ltv::SeedPose pose;
      pose.t = entry.first + offset;
      pose.R_WB = entry.second->Rot().transpose();
      pose.p_WB = entry.second->pos();
      if (entry.first == camera_time) {
        feature_context.execution_pose_index = feature_context.poses.size();
        execution_pose_found = true;
      }
      feature_context.poses.push_back(pose);
      pose_order.push_back(entry.second);
    }
    if (!execution_pose_found)
      throw std::runtime_error("Feature context lacks current execution clone");
    // JPL's left delta on R_BW is exp(-[dtheta]) R_BW. Thus R_WB
    // has the right positive body delta used by the geometric Jacobian.
    // PoseJPL position error is additive in world coordinates. No sign flip
    // or block-diagonal approximation: retain all clone cross-covariances.
    feature_context.pose_covariance = StateHelper::get_marginal_covariance(state, pose_order);
    for (int camera = 0; camera < state->_options.num_cameras; ++camera) {
      ltv::SeedCamera seed_camera;
      if (camera == 0) {
        seed_camera.R_BC = R_BC;
        seed_camera.p_BC = p_BC;
      } else {
        const auto &extrinsic = state->_calib_IMUtoCAM.at(camera);
        seed_camera.R_BC = extrinsic->Rot().transpose();
        seed_camera.p_BC = -seed_camera.R_BC * extrinsic->pos();
      }
      feature_context.cameras.push_back(seed_camera);
    }
  }
  for (const auto &feature : features) {
    const int camera_count = state->_options.num_cameras;
    for (int camera = 0; camera < camera_count; ++camera) {
      const auto history = feature->timestamps.find(camera);
      if (history == feature->timestamps.end())
        continue;
      const auto current = std::find(history->second.begin(), history->second.end(), camera_time);
      if (current == history->second.end())
        continue;
      const auto uv = feature->uvs_norm.at(camera).at(std::distance(history->second.begin(), current));
      const Eigen::Vector3d bearing(uv(0), uv(1), 1);
      {
        ltv::HistoryObservation observation;
        observation.feature_id = feature->featid;
        observation.camera_id = camera;
        observation.bearing = bearing;

        feature_context.observations.push_back(observation);
      }
    }
  }
  return feature_context;
}

void VioManager::initialize_ltv() {
  const auto &o = params.ltv_options;
  ltv_observer.reset(new ltv::LtvObserver());
  ltv_observer->configure(o.observer);
  ltv::LtvFeatureConfig config;
  config.manager = o.feature_manager;
  config.manager.apply_seed = o.feature_apply_seed;
  config.manager.bounded_memory = true;
  config.active_consistency = o.active_consistency;
  config.active_consistency.history_window_s = config.manager.history.history_window;
  config.source = o.feature_seed_source == "STEREO"                 ? ltv::FeatureSeedSource::Stereo
                  : o.feature_seed_source == "STEREO_THEN_TEMPORAL" ? ltv::FeatureSeedSource::StereoThenTemporal
                                                                    : ltv::FeatureSeedSource::TemporalPose;
  config.bearing_sigma_rad = o.feature_bearing_sigma_rad;
  ltv_features.reset(new ltv::LtvFeatureManager(config));
}
void VioManager::reset_ltv() {
  if (!ltv_observer)
    return;
  const auto epoch = ltv_lifecycle.epoch + 1;
  ltv_lifecycle = LtvLifecycle();
  ltv_lifecycle.epoch = epoch;
  ltv_observer->reset();
  ltv_features->reset(epoch);
  reset_ltv_lifecycle();
}
void VioManager::pause_ltv(double time, const std::string &reason) {
  if (!ltv_observer)
    return;
  ltv::LtvAvailabilityInput input;
  input.time = time + state->_calib_dt_CAMtoIMU->value()(0);
  input.physical_input_fault = reason == "invalid_or_out_of_order_imu" || reason == "missing_imu_bracket" || reason == "imu_bracket_gap" ||
                               reason == "camera_gap" || reason == "imu_gap" || reason == "out_of_order_camera" ||
                               reason == "propagation_failed";
  ltv_lifecycle.status = update_ltv_lifecycle(input);
  ltv_observer->reset();
  ltv_features->reset(ltv_lifecycle.epoch);
  ltv_lifecycle.started = false;
  ltv_lifecycle.suspended = ltv_lifecycle.new_epoch = true;
  ltv_lifecycle.healthy_frames = 0;
  ltv_lifecycle.camera_time = std::max(ltv_lifecycle.camera_time, time);
  ltv_diagnostics = LtvDiagnostics();
  LtvFrame frame;
  frame.camera_time = time;
  frame.reason = reason;
  record_ltv(frame);
}
LtvFrame VioManager::process_ltv(double t, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
  const auto begin = std::chrono::steady_clock::now();
  auto &life = ltv_lifecycle;
  const auto &options = params.ltv_options;
  const double target = t + state->_calib_dt_CAMtoIMU->value()(0);
  LtvFrame frame;
  frame.camera_time = t;
  frame.imu_time = target;
  auto fail = [&](const std::string &why) {
    pause_ltv(t, why);
    frame.reason = why;
    return frame;
  };
  if (t <= life.camera_time) {
    frame.reason = "duplicate_or_backward_camera";
    return frame;
  }
  auto interval = propagator->imu_interval(life.suspended ? target : life.cursor, target, options.max_imu_dt);
  if (interval.fault_version != life.imu_fault_version) {
    life.imu_fault_version = interval.fault_version;
    return fail("invalid_or_out_of_order_imu");
  }
  if (!interval.valid())
    return fail(interval.error);
  if (!life.suspended && (target <= life.cursor || target - life.cursor > options.reset_gap))
    return fail("camera_gap");
  life.camera_time = t;
  if (life.suspended || life.new_epoch) {
    ++life.epoch;
    ltv_observer->reset();
    ltv_features->reset(life.epoch);
    life.started = false;
    life.healthy_frames = 0;
    life.correction_time = -1;
    life.corrections = 0;
    life.cursor = target;
    life.suspended = life.new_epoch = false;
  }
  if (options.health_options.initial_unseeded_warmup && !life.initial_warm && !life.status.bootstrap_count &&
      !life.status.physical_fault_count) {
    life.initial_warm = true;
    life.started = true;
  }
  if (life.started) {
    const auto Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
    const auto Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
    const auto Tg = State::Tg(state->_calib_imu_tg->value());
    const auto &samples = interval.samples;
    for (size_t k = 1; k < samples.size(); ++k) {
      auto a = Propagator::correct_imu(samples[k - 1], Da, Dw, Tg, state->_calib_imu_ACCtoIMU->Rot(), state->_calib_imu_GYROtoIMU->Rot(),
                                       ba, bg);
      auto b =
          Propagator::correct_imu(samples[k], Da, Dw, Tg, state->_calib_imu_ACCtoIMU->Rot(), state->_calib_imu_GYROtoIMU->Rot(), ba, bg);
      const double dt = b.timestamp - a.timestamp;
      if (!ltv_observer->propagateImu(dt, .5 * (a.am + b.am), .5 * (a.wm + b.wm)))
        return fail("core_imu_failure");
      ++life.integrated_steps;
      life.integrated_seconds += dt;
    }
  }
  life.cursor = target;
  auto context = ltv_feature_context(t);
  context.epoch = life.epoch;
  const bool cold = !life.started;
  auto staged_features = *ltv_features;
  auto input = staged_features.prepare(context, *ltv_observer, life.status.ready_V, !cold, options.enable_landmark_approx);
  ltv::LtvAvailabilityInput health;
  health.time = health.observer_time = target;
  health.geometry_valid = staged_features.frame().management.accepted_input;
  health.eligible_seeds = staged_features.frame().management.eligible_seeds;
  if (cold) {
    life.status = update_ltv_lifecycle(health);
    if (!life.status.request_bootstrap) {
      *ltv_features = std::move(staged_features);
      frame.reason = life.status.reason;
      frame.epoch = life.epoch;
      frame.sequence = ++life.sequence;
      return frame;
    }
    staged_features = *ltv_features;
    input = staged_features.prepare(context, *ltv_observer, life.status.ready_V, true, options.enable_landmark_approx);
  }
  auto staged_observer = *ltv_observer;
  const auto predicted = staged_observer.snapshot();
  const double dt = life.correction_time < 0 ? 0 : target - life.correction_time;
  const auto &camera = context.cameras.at(0);
  if (!staged_observer.synchronize(input.control) || !staged_observer.correct(dt, input.observations, camera.R_BC, camera.p_BC))
    return fail("core_camera_failure");
  const auto corrected = staged_observer.snapshot();
  const bool correction = dt > 0 && !input.observations.empty();
  health.correction_diagnostics_valid = correction && input.prediction_angles.size() >= options.health_options.min_features;
  if (health.correction_diagnostics_valid) {
    auto &angles = input.prediction_angles;
    std::sort(angles.begin(), angles.end());
    health.prediction_angle_p95_rad = angles[static_cast<size_t>(std::ceil(.95 * angles.size())) - 1];
    health.velocity_correction_rate = (corrected.velocity_body - predicted.velocity_body).norm() / dt;
    health.gravity_correction_rate = (corrected.gravity_body - predicted.gravity_body).norm() / dt;
  }
  if (correction && input.observations.size() >= options.health_options.min_features)
    ++life.corrections;
  health.observer_started = true;
  health.observer_finite = true;
  health.velocity = corrected.velocity_body;
  health.gravity = corrected.gravity_body;
  health.observed_features = input.observations.size();
  health.mature_observed_features = staged_features.frame().management.mature_visible;
  health.actual_corrections = life.corrections;
  if (!cold)
    life.status = update_ltv_lifecycle(health);
  if (cold || life.status.request_bootstrap) {
    health.bootstrap_was_physical_recovery = life.status.bootstrap_is_physical_recovery;
    health.bootstrap_source = cold ? "ZERO_DELAYED_START" : "BOUNDED_INITIAL_ZERO_WARMUP";
    life.status = acknowledge_ltv_bootstrap(health);
  }
  staged_features.commit(context, staged_observer, options.enable_landmark_approx);
  *ltv_observer = std::move(staged_observer);
  *ltv_features = std::move(staged_features);
  life.started = true;
  life.correction_time = target;
  life.healthy_frames = health.observed_features >= static_cast<size_t>(options.min_features) ? life.healthy_frames + 1 : 0;
  life.new_epoch = life.status.request_dormant;
  frame.epoch = life.epoch;
  frame.sequence = ++life.sequence;
  frame.available = true;
  frame.reason = life.status.reason;
  frame.snapshot = corrected;
  frame.valid = life.healthy_frames >= options.warmup_camera_updates;
  frame.velocity_valid = frame.valid && corrected.velocity_body.allFinite();
  frame.gravity_valid =
      frame.valid && corrected.gravity_body.norm() >= options.gravity_norm_min && corrected.gravity_body.norm() <= options.gravity_norm_max;
  frame.ready_G = life.status.ready_G;
  frame.ready_V = life.status.ready_V;
  frame.landmarks = ltv_features->candidates();
  frame.compute_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return frame;
}

// Native observer lifecycle and recovery; branch confirmation lives in UpdaterLTV.
using namespace ltv;
void VioManager::reset_ltv_lifecycle() {
  ltv_lifecycle.state_ = LtvAvailability::Collecting;
  ltv_lifecycle.last_time_ = ltv_lifecycle.shortage_since_ = ltv_lifecycle.last_bootstrap_ = ltv_lifecycle.last_quality_bootstrap_ = -1;
  ltv_lifecycle.supply_frames_ = 0;
  updaterMSCKF->ltv_model().reset_health();
  ltv_lifecycle.pending_physical_ = ltv_lifecycle.pending_initial_warmup_ = false;
  ltv_lifecycle.awaiting_bootstrap_ = ltv_lifecycle.bootstrapped_ = ltv_lifecycle.dormant_requested_ = ltv_lifecycle.physical_recovery_ =
      ltv_lifecycle.fault_latched_ = false;
  ltv_lifecycle.bootstrap_count_ = ltv_lifecycle.physical_fault_count_ = 0;
  ltv_lifecycle.bootstrap_source_.clear();
}
void VioManager::validate_ltv_bootstrap(const LtvAvailabilityInput &i) const {
  if (!ltv_lifecycle.awaiting_bootstrap_ || i.bootstrap_was_physical_recovery != ltv_lifecycle.pending_physical_)
    throw std::invalid_argument("bootstrap ack has no matching pending request");
  if (ltv_lifecycle.pending_initial_warmup_ && i.bootstrap_source != "BOUNDED_INITIAL_ZERO_WARMUP")
    throw std::invalid_argument("initial warmup ack source mismatch");
  if (i.bootstrap_source.empty() || !i.observer_started || !i.observer_finite || !i.velocity.allFinite() || !i.gravity.allFinite() ||
      !std::isfinite(i.observer_time) || std::abs(i.observer_time - i.time) > params.ltv_options.health_options.timestamp_tolerance)
    throw std::invalid_argument("invalid committed readiness bootstrap");
  if (ltv_lifecycle.bootstrapped_ && !i.bootstrap_was_physical_recovery && ltv_lifecycle.last_quality_bootstrap_ >= 0 &&
      i.time - ltv_lifecycle.last_quality_bootstrap_ < params.ltv_options.health_options.quality_bootstrap_interval)
    throw std::invalid_argument("quality bootstrap cooldown violated");
  if (i.bootstrap_was_physical_recovery && !ltv_lifecycle.physical_recovery_)
    throw std::invalid_argument("undeclared physical recovery bootstrap");
}
void VioManager::commit_ltv_bootstrap(const LtvAvailabilityInput &i) {
  ltv_lifecycle.last_bootstrap_ = i.time;
  if (!i.bootstrap_was_physical_recovery)
    ltv_lifecycle.last_quality_bootstrap_ = i.time;
  ltv_lifecycle.bootstrap_source_ = i.bootstrap_source;
  ++ltv_lifecycle.bootstrap_count_;
  ltv_lifecycle.bootstrapped_ = true;
  ltv_lifecycle.awaiting_bootstrap_ = ltv_lifecycle.physical_recovery_ = ltv_lifecycle.dormant_requested_ = false;
  ltv_lifecycle.supply_frames_ = 0;
  updaterMSCKF->ltv_model().reset_health();
  ltv_lifecycle.shortage_since_ =
      ltv_lifecycle.pending_initial_warmup_ && !(i.geometry_valid && i.observed_features >= params.ltv_options.health_options.min_features)
          ? i.time
          : -1;
  ltv_lifecycle.pending_initial_warmup_ = false;
  ltv_lifecycle.state_ = LtvAvailability::Bootstrapping;
}
LtvAvailabilityOutput VioManager::acknowledge_ltv_bootstrap(const LtvAvailabilityInput &i) {
  if (!params.ltv_options.health_options.enabled || !std::isfinite(i.time) || i.time != ltv_lifecycle.last_time_ || !i.clock_valid ||
      i.physical_input_fault)
    throw std::invalid_argument("same-event bootstrap ack time/fault mismatch");
  validate_ltv_bootstrap(i);
  commit_ltv_bootstrap(i);
  LtvAvailabilityOutput o;
  o.ready_soft_grace_enabled = params.ltv_options.health_options.ready_soft_grace;
  o.accepted_time = true;
  o.state = ltv_lifecycle.state_;
  o.observer_valid = true;
  o.reason = "bootstrap_committed_confirming";
  o.last_bootstrap_time = ltv_lifecycle.last_bootstrap_;
  o.last_bootstrap_source = ltv_lifecycle.bootstrap_source_;
  o.bootstrap_count = ltv_lifecycle.bootstrap_count_;
  o.physical_fault_count = ltv_lifecycle.physical_fault_count_;
  return o;
}
LtvAvailabilityOutput VioManager::update_ltv_lifecycle(const LtvAvailabilityInput &i) {
  LtvAvailabilityOutput o;
  o.ready_soft_grace_enabled = params.ltv_options.health_options.ready_soft_grace;
  auto finish = [&](const char *reason) {
    o.state = ltv_lifecycle.state_;
    o.reason = reason;
    o.last_bootstrap_time = ltv_lifecycle.last_bootstrap_;
    o.last_bootstrap_source = ltv_lifecycle.bootstrap_source_;
    o.bootstrap_count = ltv_lifecycle.bootstrap_count_;
    o.physical_fault_count = ltv_lifecycle.physical_fault_count_;
    o.joint_ready = o.ready_G && o.ready_V;
    updaterMSCKF->ltv_model().health_diagnostics(o);
    return o;
  };
  if (!params.ltv_options.health_options.enabled)
    return o;
  if (!std::isfinite(i.time) || i.time < 0 || i.time <= ltv_lifecycle.last_time_)
    return finish("invalid_or_duplicate_event_time");
  const bool gap =
      ltv_lifecycle.last_time_ >= 0 && i.time - ltv_lifecycle.last_time_ > params.ltv_options.health_options.confirmation_max_gap +
                                                                               params.ltv_options.health_options.timestamp_tolerance;
  o.accepted_time = true;
  ltv_lifecycle.last_time_ = i.time;
  if (i.physical_input_fault || !i.clock_valid) {
    if (!ltv_lifecycle.fault_latched_)
      ++ltv_lifecycle.physical_fault_count_;
    ltv_lifecycle.awaiting_bootstrap_ = false;
    ltv_lifecycle.fault_latched_ = true;
    ltv_lifecycle.physical_recovery_ = true;
    ltv_lifecycle.supply_frames_ = 0;
    updaterMSCKF->ltv_model().reset_health();
    ltv_lifecycle.state_ = LtvAvailability::Dormant;
    if (!ltv_lifecycle.dormant_requested_) {
      o.request_dormant = true;
      ltv_lifecycle.dormant_requested_ = true;
    }
    return finish("physical_input_fault");
  }
  ltv_lifecycle.fault_latched_ = false;
  if (gap) {
    ltv_lifecycle.awaiting_bootstrap_ = false;
    ltv_lifecycle.supply_frames_ = 0;
    updaterMSCKF->ltv_model().reset_health();
  }
  const bool supply = i.geometry_valid && i.eligible_seeds >= params.ltv_options.health_options.min_features;
  ltv_lifecycle.supply_frames_ =
      supply ? std::min(ltv_lifecycle.supply_frames_ + 1, params.ltv_options.health_options.bootstrap_confirm_frames) : 0;
  o.pool_ready = i.geometry_valid && i.observed_features >= params.ltv_options.health_options.min_features &&
                 i.mature_observed_features >= params.ltv_options.health_options.min_features;
  const bool current = i.observer_started && i.observer_finite && i.velocity.allFinite() && i.gravity.allFinite() &&
                       std::isfinite(i.observer_time) &&
                       std::abs(i.observer_time - i.time) <= params.ltv_options.health_options.timestamp_tolerance;
  o.observer_valid = current && ltv_lifecycle.state_ != LtvAvailability::Dormant && ltv_lifecycle.state_ != LtvAvailability::Collecting;
  if (params.ltv_options.health_options.preserve_constrained_state && current && (i.velocity.norm() > 100.0 || i.gravity.norm() > 50.0)) {
    ltv_lifecycle.awaiting_bootstrap_ = false;
    updaterMSCKF->ltv_model().reset_health();
    ltv_lifecycle.state_ = LtvAvailability::Dormant;
    o.observer_valid = false;
    if (!ltv_lifecycle.dormant_requested_) {
      o.request_dormant = true;
      ltv_lifecycle.dormant_requested_ = true;
    }
    return finish("observer_norm_guard");
  }
  // No reliable current state exists in collection/dormancy. Merely having a
  // finite stale vector cannot promote a stopped observer back to TRACKING.
  if (ltv_lifecycle.awaiting_bootstrap_ || !ltv_lifecycle.bootstrapped_ || ltv_lifecycle.state_ == LtvAvailability::Dormant ||
      ltv_lifecycle.state_ == LtvAvailability::Collecting || (ltv_lifecycle.state_ == LtvAvailability::Recovering && !current)) {
    o.observer_valid = false;
    const bool cooldown = ltv_lifecycle.physical_recovery_ || ltv_lifecycle.last_quality_bootstrap_ < 0 ||
                          i.time - ltv_lifecycle.last_quality_bootstrap_ >= params.ltv_options.health_options.quality_bootstrap_interval;
    // This preserves an already current core; it does not manufacture one and
    // cannot be reused for later observer epochs or physical-fault recovery.
    if (params.ltv_options.health_options.initial_unseeded_warmup && !ltv_lifecycle.bootstrapped_ && !ltv_lifecycle.physical_recovery_ &&
        current) {
      o.request_bootstrap = !ltv_lifecycle.awaiting_bootstrap_;
      ltv_lifecycle.awaiting_bootstrap_ = true;
      ltv_lifecycle.pending_physical_ = false;
      ltv_lifecycle.pending_initial_warmup_ = true;
      ltv_lifecycle.state_ = LtvAvailability::Recovering;
      return finish("initial_bounded_warmup");
    }
    if (ltv_lifecycle.supply_frames_ >= params.ltv_options.health_options.bootstrap_confirm_frames && cooldown) {
      ltv_lifecycle.pending_initial_warmup_ = false;
      o.request_bootstrap = !ltv_lifecycle.awaiting_bootstrap_;
      ltv_lifecycle.awaiting_bootstrap_ = true;
      ltv_lifecycle.pending_physical_ = ltv_lifecycle.physical_recovery_;
      o.bootstrap_is_physical_recovery = ltv_lifecycle.physical_recovery_;
      ltv_lifecycle.state_ = LtvAvailability::Recovering;
      return finish("atomic_bootstrap_required");
    }
    ltv_lifecycle.awaiting_bootstrap_ = false;
    ltv_lifecycle.state_ = ltv_lifecycle.bootstrapped_ ? LtvAvailability::Dormant : LtvAvailability::Collecting;
    return finish(!supply ? "insufficient_seed_supply" : !cooldown ? "quality_bootstrap_cooldown" : "confirming_seed_supply");
  }
  const bool adequate = i.geometry_valid && i.observed_features >= params.ltv_options.health_options.min_features;
  if (params.ltv_options.health_options.preserve_constrained_state && current && i.geometry_valid && i.observed_features > 0 && !adequate) {
    // Thin current constraints permit continued estimation, never readiness.
    // Only a new complete loss starts the no-observation dormant clock.
    ltv_lifecycle.shortage_since_ = -1;
    updaterMSCKF->ltv_model().reset_health();
    ltv_lifecycle.state_ = LtvAvailability::Degraded;
    return finish("thin_observations_state_preserved_not_ready");
  }
  if (!adequate || !current) {
    updaterMSCKF->ltv_model().reset_health();
    if (ltv_lifecycle.shortage_since_ < 0)
      ltv_lifecycle.shortage_since_ = i.time;
    const double duration = i.time - ltv_lifecycle.shortage_since_;
    ltv_lifecycle.state_ = LtvAvailability::Degraded;
    if (!current || duration >= params.ltv_options.health_options.dormant_seconds) {
      ltv_lifecycle.state_ = LtvAvailability::Dormant;
      o.observer_valid = false;
      if (!ltv_lifecycle.dormant_requested_) {
        o.request_dormant = true;
        ltv_lifecycle.dormant_requested_ = true;
      }
      return finish(!current ? "observer_not_current_or_finite" : "prolonged_insufficient_geometry");
    }
    return finish(duration <= params.ltv_options.health_options.coast_seconds ? "finite_coast_not_ready"
                                                                              : "insufficient_geometry_not_ready");
  }
  ltv_lifecycle.shortage_since_ = -1;
  const auto reason = updaterMSCKF->ltv_model().update_health(i, o);
  ltv_lifecycle.state_ = (o.ready_G || o.ready_V) ? LtvAvailability::Tracking : LtvAvailability::Bootstrapping;
  return finish(reason.c_str());
}
