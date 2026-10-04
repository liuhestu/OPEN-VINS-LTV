#include "LtvLandmarkAdapter.h"
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
namespace ltv {
LtvLandmarkAdapter::LtvLandmarkAdapter(const FeaturePipelineConfig &config)
    : config_(config), manager_(config.manager), active_consistency_(config.active_consistency) {
  if (!std::isfinite(config.bearing_sigma_rad) || config.bearing_sigma_rad < 0 || config.source == FeatureSeedSource::MsckfGeometry)
    throw std::invalid_argument("unsupported feature pipeline configuration");
}
void LtvLandmarkAdapter::reset(uint64_t epoch) { manager_.reset(epoch); }
FeaturePipelineFrame LtvLandmarkAdapter::process(const FeaturePipelineContext &ctx, bool allow_admission) {
  FeaturePipelineFrame out;
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
      FeatureSeedDiagnostic d;
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
                u.observations.push_back({local_pose(pose->second), 0, o.bearing.normalized()});
          }
      }
      const auto &cam0 = feature.second.at(0);
      if (cam0.match_valid) {
        u.anchor_observation = u.observations.size();
        u.observations.push_back({u.execution_pose_index, 0, cam0.bearing.normalized()});
      }
      if (source == FeatureSeedSource::Stereo && feature.second.count(1) && feature.second.at(1).match_valid)
        u.observations.push_back({u.execution_pose_index, 1, feature.second.at(1).bearing.normalized()});
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
