#include "ltv/landmark_adapter/LtvActiveConsistency.h"
#include "ltv/landmark_adapter/LtvLandmarkContext.h"
#include "ltv/landmark_adapter/LtvSeedEstimator.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

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
                                                       const FeaturePipelineContext &ctx) const {
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
