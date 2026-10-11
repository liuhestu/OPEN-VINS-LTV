#include "ltv/features/LtvFeatureFrontend.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

// 1. Check retained features against causal geometry.
namespace ltv {
const char *toString(ActiveConsistencyReason r) {
  switch (r) {
  case ActiveConsistencyReason::Disabled:
    return "DISABLED";
  case ActiveConsistencyReason::InsufficientHistory:
    return "INSUFFICIENT_HISTORY";
  case ActiveConsistencyReason::MissingPoseSupport:
    return "MISSING_POSE_SUPPORT";
  case ActiveConsistencyReason::FitFailed:
    return "FIT_FAILED";
  case ActiveConsistencyReason::HistoryInconsistent:
    return "HISTORY_INCONSISTENT";
  case ActiveConsistencyReason::HoldoutInconsistent:
    return "HOLDOUT_INCONSISTENT";
  case ActiveConsistencyReason::Pass:
    return "PASS";
  }
  return "UNKNOWN";
}
LtvActiveConsistency::LtvActiveConsistency(const ActiveConsistencyConfig &c) : config_(c) {
  if (!c.enabled)
    return;
  if (c.min_history_frames < 2 || c.retire_after_consecutive_failures < 1 || !std::isfinite(c.min_history_span_s) ||
      c.min_history_span_s <= 0 || !std::isfinite(c.history_window_s) || c.history_window_s < c.min_history_span_s ||
      !std::isfinite(c.max_history_residual_rad) || c.max_history_residual_rad <= 0 || c.max_history_residual_rad >= std::acos(-1.) ||
      !std::isfinite(c.max_holdout_residual_rad) || c.max_holdout_residual_rad <= 0 || c.max_holdout_residual_rad >= std::acos(-1.))
    throw std::invalid_argument("invalid active consistency configuration");
}
ActiveConsistencyResult LtvActiveConsistency::evaluate(size_t id, double time, const FeatureTrackHistory &history,
                                                       const LtvFeatureContext &ctx) const {
  ActiveConsistencyResult out;
  out.feature_id = id;
  if (!config_.enabled)
    return out;
  out.reason = ActiveConsistencyReason::FitFailed;
  out.fit_reason = "INVALID_CONTEXT";
  auto rotation_valid = [](const Eigen::Matrix3d &r) {
    return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-8 && std::abs(r.determinant() - 1) < 1e-8;
  };
  if (!std::isfinite(time) || time < 0 || !std::isfinite(ctx.time) || time != ctx.time || ctx.execution_pose_index >= ctx.poses.size() ||
      ctx.cameras.empty() || !history.identity_valid)
    return out;
  const auto &execution = ctx.poses[ctx.execution_pose_index];
  if (!std::isfinite(execution.t) || std::abs(execution.t - time) > 1e-9)
    return out;
  const auto &camera = ctx.cameras[0];
  if (!rotation_valid(camera.R_BC) || !camera.p_BC.allFinite())
    return out;
  std::map<double, size_t> poses;
  for (size_t j = 0; j < ctx.poses.size(); ++j) {
    const auto &p = ctx.poses[j];
    if (!std::isfinite(p.t) || p.t > time || !rotation_valid(p.R_WB) || !p.p_WB.allFinite() || !poses.emplace(p.t, j).second)
      return out;
  }
  const HistoryObservation *current = nullptr;
  for (const auto &o : ctx.observations)
    if (o.feature_id == id && o.camera_id == 0) {
      if (current || !o.match_valid || !o.bearing.allFinite() || o.bearing.norm() < 1e-12)
        return out;
      current = &o;
    }
  if (!current) {
    out.fit_reason = "MISSING_CURRENT_OBSERVATION";
    return out;
  }
  SeedInput input;
  input.cameras.push_back(camera);
  input.poses.push_back(execution);
  input.execution_pose_index = 0;
  std::set<double> timestamps;
  std::set<size_t> used_poses;
  size_t past_count = 0;
  double oldest = time, newest = -1;
  for (const auto &sample : history.samples) {
    if (!std::isfinite(sample.time))
      return out;
    if (sample.time >= time || sample.time < time - config_.history_window_s)
      continue;
    const HistoryObservation *past = nullptr;
    for (const auto &o : sample.observations)
      if (o.feature_id == id && o.camera_id == 0 && o.match_valid) {
        if (past || !o.bearing.allFinite() || o.bearing.norm() < 1e-12)
          return out;
        past = &o;
      }
    if (!past)
      continue;
    if (!timestamps.insert(sample.time).second) {
      out.fit_reason = "DUPLICATE_HISTORY_TIME";
      return out;
    }
    ++past_count;
    auto match = poses.lower_bound(sample.time - 1e-9);
    if (match == poses.end() || std::abs(match->first - sample.time) > 1e-9 || match->first >= time ||
        match->second == ctx.execution_pose_index) {
      ++out.missing_pose_count;
      continue;
    }
    auto next = std::next(match);
    if (next != poses.end() && std::abs(next->first - sample.time) <= 1e-9) {
      out.fit_reason = "AMBIGUOUS_POSE_TIME";
      return out;
    }
    if (!used_poses.insert(match->second).second) {
      out.fit_reason = "DUPLICATE_POSE_SUPPORT";
      return out;
    }
    input.poses.push_back(ctx.poses[match->second]);
    input.observations.push_back({input.poses.size() - 1, 0, past->bearing.normalized()});
    oldest = std::min(oldest, sample.time);
    newest = std::max(newest, sample.time);
  }
  out.history_count = input.observations.size();
  out.history_span_s = out.history_count ? newest - oldest : 0;
  if (out.history_count < config_.min_history_frames || out.history_span_s < config_.min_history_span_s) {
    out.reason = out.missing_pose_count && past_count >= config_.min_history_frames ? ActiveConsistencyReason::MissingPoseSupport
                                                                                    : ActiveConsistencyReason::InsufficientHistory;
    out.fit_reason = "NOT_ATTEMPTED";
    return out;
  }
  const auto estimate = LtvSeedEstimator::estimate(input);
  out.fit_reason = estimate.reason;
  // A failed estimator result is unavailable in the first registered policy,
  // including nonpositive signed rays; it never increments a failure count.
  if (!estimate.valid || !estimate.point_W.allFinite() || !std::isfinite(estimate.max_residual_rad) || !std::isfinite(estimate.condition))
    return out;
  out.fit_valid = estimate.valid;
  out.point_W = estimate.point_W;
  out.fit_condition = estimate.condition;
  out.history_max_residual_rad = estimate.max_residual_rad;
  if (estimate.max_residual_rad > config_.max_history_residual_rad) {
    out.evaluable = true;
    out.pass = false;
    out.reason = ActiveConsistencyReason::HistoryInconsistent;
    return out;
  }
  Eigen::Vector3d ray = camera.R_BC.transpose() * (execution.R_WB.transpose() * (estimate.point_W - execution.p_WB) - camera.p_BC);
  if (!ray.allFinite() || ray.norm() < 1e-10) {
    out.fit_reason = "CURRENT_AT_CAMERA_CENTER";
    return out;
  }
  out.holdout_residual_rad = std::acos(std::max(-1., std::min(1., ray.normalized().dot(current->bearing.normalized()))));
  out.evaluable = true;
  out.pass = out.holdout_residual_rad <= config_.max_holdout_residual_rad;
  out.reason = out.pass ? ActiveConsistencyReason::Pass : ActiveConsistencyReason::HoldoutInconsistent;
  return out;
}
} // namespace ltv

// 2. Build seed candidates and run the lifecycle transaction.
namespace ltv {
LtvFeatureFrontend::LtvFeatureFrontend(const LtvFeatureConfig &config)
    : config_(config), manager_(config.manager), active_consistency_(config.active_consistency) {
  if (!std::isfinite(config.bearing_sigma_rad) || config.bearing_sigma_rad < 0 || config.source == FeatureSeedSource::MsckfGeometry)
    throw std::invalid_argument("unsupported feature pipeline configuration");
}
void LtvFeatureFrontend::reset(uint64_t epoch) { manager_.reset(epoch); }
LtvFeatureFrame LtvFeatureFrontend::process(const LtvFeatureContext &ctx, bool allow_admission) {
  LtvFeatureFrame out;
  out.management.epoch = ctx.epoch;
  out.management.time = ctx.time;
  out.management.reason = "invalid_geometry_context";
  if (config_.manager.bounded_memory && (ctx.observations.size() > config_.manager.history.max_observations_per_packet ||
                                         ctx.cameras.size() > config_.manager.history.max_camera_count || ctx.poses.size() > 32)) {
    out.management.reason = "context_capacity_exceeded";
    return out;
  }
  if (!std::isfinite(ctx.time) || ctx.time < 0 || ctx.epoch != manager_.history().epoch() || ctx.time <= manager_.history().time() ||
      ctx.execution_pose_index >= ctx.poses.size() || ctx.cameras.empty() ||
      std::abs(ctx.poses[ctx.execution_pose_index].t - ctx.time) > 1e-9 ||
      ctx.pose_covariance.rows() != static_cast<int>(6 * ctx.poses.size()) || ctx.pose_covariance.cols() != ctx.pose_covariance.rows() ||
      !ctx.pose_covariance.allFinite())
    return out;
  const double scale = std::max(1.0, ctx.pose_covariance.norm());
  if ((ctx.pose_covariance - ctx.pose_covariance.transpose()).norm() > 1e-10 * scale)
    return out;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> psd(ctx.pose_covariance);
  if (psd.info() != Eigen::Success || psd.eigenvalues().minCoeff() < -1e-12 * scale)
    return out;
  std::map<double, size_t> pose_by_time;
  for (size_t i = 0; i < ctx.poses.size(); ++i) {
    const auto &p = ctx.poses[i];
    if (!std::isfinite(p.t) || p.t > ctx.time + 1e-9 || !p.R_WB.allFinite() || !p.p_WB.allFinite() ||
        (p.R_WB.transpose() * p.R_WB - Eigen::Matrix3d::Identity()).norm() > 1e-8 || p.R_WB.determinant() < 0 ||
        !pose_by_time.emplace(p.t, i).second)
      return out;
  }
  for (const auto &c : ctx.cameras)
    if (!c.R_BC.allFinite() || !c.p_BC.allFinite() || (c.R_BC.transpose() * c.R_BC - Eigen::Matrix3d::Identity()).norm() > 1e-8 ||
        c.R_BC.determinant() < 0)
      return out;
  std::map<size_t, std::map<int, HistoryObservation>> current;
  for (const auto &o : ctx.observations) {
    if (o.camera_id < 0 || static_cast<size_t>(o.camera_id) >= ctx.cameras.size() || !o.bearing.allFinite() || o.bearing.norm() < 1e-12 ||
        !current[o.feature_id].emplace(o.camera_id, o).second)
      return out;
  }
  std::map<size_t, ActiveConsistencyResult> consistency;
  if (config_.active_consistency.enabled) {
    for (size_t id : manager_.retained_ids()) {
      auto visible = current.find(id);
      const auto *history = manager_.history().find(id);
      if (history && visible != current.end() && visible->second.count(0) && visible->second.at(0).match_valid)
        consistency.emplace(id, active_consistency_.evaluate(id, ctx.time, *history, ctx));
    }
  }
  std::vector<FeatureSeedCandidate> candidates;
  for (const auto &feature : current) {
    if (!feature.second.count(0))
      continue;
    auto prior = manager_.landmarks().find(feature.first);
    if (prior != manager_.landmarks().end() && (prior->second.entered >= 0 || prior->second.phase == LandmarkPhase::Retired))
      continue;
    // A single pure construction path keeps each fixed-source candidate's mean,
    // covariance and arithmetic order unchanged. Hybrid evaluates a second source
    // only when current stereo geometry fails the same frozen admission limits.
    auto construct = [&](FeatureSeedSource source) {
      FeatureSeedResult d;
      d.candidate.feature_id = feature.first;
      d.candidate.epoch = ctx.epoch;
      d.candidate.time = ctx.time;
      d.candidate.source = source;
      auto &u = d.input;
      u.cameras = ctx.cameras;
      std::map<size_t, size_t> pose_map;
      std::vector<size_t> global_indices;
      auto local_pose = [&](size_t global) {
        auto it = pose_map.find(global);
        if (it != pose_map.end())
          return it->second;
        const size_t local = u.poses.size();
        u.poses.push_back(ctx.poses[global]);
        global_indices.push_back(global);
        pose_map.emplace(global, local);
        return local;
      };
      u.execution_pose_index = local_pose(ctx.execution_pose_index);
      if (source == FeatureSeedSource::TemporalPose) {
        const auto *h = manager_.history().find(feature.first);
        if (h && h->visible)
          for (const auto &sample : h->samples) {
            if (sample.time >= ctx.time || ctx.time - sample.time > config_.manager.history.history_window)
              continue;
            // Resolve in this context only. Missing/marginalized clones are not replaced
            // by stale historical means or invented independent covariance blocks.
            auto pose = pose_by_time.lower_bound(sample.time - 1e-9);
            if (pose == pose_by_time.end() || std::abs(pose->first - sample.time) > 1e-9)
              continue;
            for (const auto &o : sample.observations)
              if (o.camera_id == 0 && o.match_valid)
                u.observations.push_back({local_pose(pose->second), 0, o.bearing.normalized(), o.pixel_source_key, o.pixel_to_tangent});
          }
      }
      const auto &cam0 = feature.second.at(0);
      if (cam0.match_valid) {
        u.anchor_observation = u.observations.size();
        u.observations.push_back({u.execution_pose_index, 0, cam0.bearing.normalized(), cam0.pixel_source_key, cam0.pixel_to_tangent});
      }
      if (source == FeatureSeedSource::Stereo && feature.second.count(1) && feature.second.at(1).match_valid)
        u.observations.push_back({u.execution_pose_index, 1, feature.second.at(1).bearing.normalized(),
                                  feature.second.at(1).pixel_source_key, feature.second.at(1).pixel_to_tangent});
      d.estimate = LtvSeedEstimator::estimate(u);
      if (source == FeatureSeedSource::Stereo && d.estimate.valid) {
        const auto *h = manager_.history().find(feature.first);
        if (h && h->visible)
          for (const auto &sample : h->samples) {
            if (sample.time >= ctx.time || ctx.time - sample.time > config_.manager.history.history_window)
              continue;
            auto prior_pose = pose_by_time.lower_bound(sample.time - 1e-9);
            if (prior_pose == pose_by_time.end() || std::abs(prior_pose->first - sample.time) > 1e-9)
              continue;
            const auto &pose = ctx.poses[prior_pose->second];
            const Eigen::Vector3d predicted =
                ctx.cameras[0].R_BC.transpose() * (pose.R_WB.transpose() * (d.estimate.point_W - pose.p_WB) - ctx.cameras[0].p_BC);
            if (!predicted.allFinite() || predicted.norm() < 1e-12)
              continue;
            for (const auto &o : sample.observations)
              if (o.camera_id == 0 && o.match_valid) {
                const double dot = predicted.normalized().dot(o.bearing.normalized());
                d.heldout_max_residual_rad = std::max(d.heldout_max_residual_rad, std::acos(std::max(-1.0, std::min(1.0, dot))));
                ++d.heldout_check_observations;
              }
          }
        d.heldout_check_available = d.heldout_check_observations > 0;
      }
      d.input_covariance = Eigen::MatrixXd::Zero(LtvSeedUncertainty::inputDimension(u), LtvSeedUncertainty::inputDimension(u));
      for (size_t a = 0; a < global_indices.size(); ++a)
        for (size_t b = 0; b < global_indices.size(); ++b)
          d.input_covariance.block<6, 6>(6 * a, 6 * b) = ctx.pose_covariance.block<6, 6>(6 * global_indices[a], 6 * global_indices[b]);
      const int bearing_start = static_cast<int>(6 * u.poses.size() + 6 * u.cameras.size());
      for (size_t i = 0; i < u.observations.size(); ++i)
        d.input_covariance.block<2, 2>(bearing_start + 2 * i, bearing_start + 2 * i) =
            config_.bearing_sigma_rad * config_.bearing_sigma_rad * Eigen::Matrix2d::Identity();
      d.uncertainty = LtvSeedUncertainty::propagate(u, d.estimate, d.input_covariance);
      if (d.uncertainty.valid && ctx.pose_error_selector.rows() == static_cast<int>(6 * ctx.poses.size()) &&
          ctx.pose_error_selector.cols() > 0) {
        d.main_jacobian = Eigen::MatrixXd::Zero(3, ctx.pose_error_selector.cols());
        for (size_t a = 0; a < global_indices.size(); ++a)
          d.main_jacobian.noalias() +=
              d.uncertainty.jacobian.middleCols(6 * a, 6) * ctx.pose_error_selector.middleRows(6 * global_indices[a], 6);
        d.bearing_jacobian = d.uncertainty.jacobian.rightCols(2 * u.observations.size());
      }

      auto &s = d.candidate;
      s.geometry_valid = d.estimate.valid && d.uncertainty.valid;
      s.landmark_B = d.estimate.landmark_B;
      s.min_ray = d.estimate.min_ray;
      s.condition = d.estimate.condition;
      s.parallax_rad = d.estimate.max_angle_rad;
      s.residual_rad = d.estimate.max_residual_rad;
      s.relative_risk = d.uncertainty.relative_parallel_risk;
      return d;
    };
    auto source = config_.source == FeatureSeedSource::StereoThenTemporal ? FeatureSeedSource::Stereo : config_.source;
    auto d = construct(source);
    if (config_.source == FeatureSeedSource::StereoThenTemporal) {
      const auto &q = config_.manager.quality;
      const auto &c = d.candidate;
      const bool stereo_pass = c.geometry_valid && c.landmark_B.allFinite() && std::isfinite(c.min_ray) && std::isfinite(c.condition) &&
                               std::isfinite(c.parallax_rad) && std::isfinite(c.residual_rad) && std::isfinite(c.relative_risk) &&
                               c.min_ray >= q.min_ray && c.condition >= 1 && c.condition <= q.max_condition &&
                               c.parallax_rad >= q.min_parallax_rad && c.residual_rad >= 0 && c.residual_rad <= q.max_residual_rad &&
                               c.relative_risk >= 0 && c.relative_risk <= q.max_relative_risk;
      if (!stereo_pass)
        d = construct(FeatureSeedSource::TemporalPose);
    }
    const auto &s = d.candidate;
    candidates.push_back(s);
    out.seeds.push_back(std::move(d));
  }
  out.management = manager_.step(ctx.epoch, ctx.time, ctx.version, ctx.observations, candidates, allow_admission,
                                 config_.active_consistency.enabled ? &consistency : nullptr,
                                 config_.active_consistency.retire_after_consecutive_failures);
  return out;
}
} // namespace ltv
