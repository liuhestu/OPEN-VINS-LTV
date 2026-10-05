#pragma once
#include "ltv/observer/LtvAdapter.h"
#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <tuple>
namespace ov_msckf {
// Diagnostic-only point prediction. Both predictors use the same past point.
inline Eigen::Vector3d shadowMainPoint(const ltv::SeedPose &current, const ltv::SeedPose &anchor, const Eigen::Vector3d &point_anchor) {
  return current.R_WB.transpose() * (anchor.R_WB * point_anchor + anchor.p_WB - current.p_WB);
}
inline double shadowBearingAngle(const Eigen::Vector3d &point_body, const ltv::SeedCamera &camera, const Eigen::Vector3d &bearing) {
  const Eigen::Vector3d ray = camera.R_BC.transpose() * (point_body - camera.p_BC);
  if (!ray.allFinite() || !bearing.allFinite() || ray.norm() < .1 || bearing.norm() < 1e-12 || ray.z() <= 0)
    return std::numeric_limits<double>::quiet_NaN();
  return std::acos(std::max(-1., std::min(1., ray.normalized().dot(bearing.normalized()))));
}
// Past-only geometry reference; never reads the current observation packet.
inline ltv::SeedInput shadowPastGeometryInput(const ltv::FeaturePipelineContext &ctx, const ltv::FeatureTrackHistory &history,
                                              double anchor_time) {
  ltv::SeedInput input;
  input.poses = ctx.poses;
  input.cameras = ctx.cameras;
  input.execution_pose_index = ctx.execution_pose_index;
  for (const auto &sample : history.samples) {
    if (sample.time < anchor_time - 1e-9 || sample.time >= ctx.time - 1e-9)
      continue;
    for (size_t i = 0; i < ctx.poses.size(); ++i)
      if (std::abs(ctx.poses[i].t - sample.time) <= 1e-9)
        for (const auto &o : sample.observations)
          if (o.camera_id == 0 && o.match_valid)
            input.observations.push_back({i, 0, o.bearing.normalized()});
  }
  return input;
}
class LtvLandmarkShadow {
public:
  explicit LtvLandmarkShadow(const std::string &path, double maturity_seconds, bool past_geometry = false)
      : maturity_(maturity_seconds), past_geometry_(past_geometry) {
    if (std::ifstream(path).good())
      throw std::runtime_error("refusing shadow output overwrite");
    out_.open(path);
    if (!out_)
      throw std::runtime_error("cannot open landmark shadow output");
    out_ << "camera_ns,imu_time,camera_id,feature_id,epoch,core_id,entered,anchor_time,anchor_span,past_mature,past_ready_V,match_valid,"
            "ltv_valid,main_valid,reason,ltv_angle_rad,main_angle_rad,ltv_x,ltv_y,ltv_z,main_x,main_y,main_z,anchor_x,anchor_y,anchor_z";
    if (past_geometry_)
      out_ << ",past_fit_valid,past_fit_angle_rad,past_fit_observations,past_fit_latest_time,past_fit_condition";
    out_ << '\n';
    out_ << std::setprecision(17);
  }
  // before=true runs after IMU propagation, before current geometry/lifecycle/correction.
  // after=false runs only after cache exact-value comparison, and stores past anchors.
  void event(bool before, const LtvAdapter &adapter, const ltv::FeaturePipelineContext &ctx, const LtvCalibration &cal) {
    if (epoch_ != ctx.epoch) {
      anchors_.clear();
      epoch_ = ctx.epoch;
      past_ready_ = false;
    }
    const auto *pipeline = adapter.landmark_adapter();
    if (!pipeline || ctx.execution_pose_index >= ctx.poses.size())
      throw std::runtime_error("landmark shadow requires valid managed context");
    const auto &records = pipeline->manager().landmarks();
    const auto &current = ctx.poses.at(ctx.execution_pose_index);
    if (std::abs(current.t - ctx.time) > 1e-9)
      throw std::runtime_error("shadow current clone time mismatch");
    past_ready_ = adapter.readiness_health().ready_V;
    if (!before) {
      for (const auto &item : records) {
        auto id = adapter.feature_ids().find(item.first);
        if (item.second.entered < 0 || item.second.phase == ltv::LandmarkPhase::Retired || id == adapter.feature_ids().end())
          continue;
        const int slot = adapter.core().slotForFeature(id->second);
        if (!adapter.core().started() || slot < 0)
          continue;
        auto &history = anchors_[Key(ctx.epoch, item.first, id->second, item.second.entered)];
        history.erase(std::remove_if(history.begin(), history.end(), [&](const Anchor &a) { return !findPose(ctx, a.time); }),
                      history.end());
        if (history.empty() || history.back().time < ctx.time)
          history.push_back({ctx.time, adapter.core().state().segment<3>(3 * slot)});
        if (history.size() > ctx.poses.size())
          throw std::runtime_error("unbounded shadow anchor history");
      }
      for (auto it = anchors_.begin(); it != anchors_.end();) {
        auto record = records.find(std::get<1>(it->first));
        auto id = adapter.feature_ids().find(std::get<1>(it->first));
        if (record == records.end() || record->second.phase == ltv::LandmarkPhase::Retired || id == adapter.feature_ids().end() ||
            record->second.entered != std::get<3>(it->first) || id->second != std::get<2>(it->first))
          it = anchors_.erase(it);
        else
          ++it;
      }
      return;
    }
    for (const auto &observation : ctx.observations) {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d ltv_point = Eigen::Vector3d::Constant(nan), main_point = ltv_point, anchor_point = ltv_point;
      double anchor_time = nan, entered = nan;
      int core_id = -1;
      bool mature = false;
      double ltv_error = nan, main_error = nan;
      std::string reason = "no_past_identity";
      auto record = records.find(observation.feature_id);
      auto id = adapter.feature_ids().find(observation.feature_id);
      if (record != records.end() && record->second.entered >= 0 && record->second.phase != ltv::LandmarkPhase::Retired &&
          id != adapter.feature_ids().end()) {
        core_id = id->second;
        entered = record->second.entered;
        // Qualification is from the previous transaction, not current consistency.
        mature = adapter.feature_frame().management.time - entered + 1e-12 >= maturity_;
        const int slot = adapter.core().slotForFeature(core_id);
        if (adapter.core().started() && slot >= 0)
          ltv_point = adapter.core().state().segment<3>(3 * slot);
        auto history = anchors_.find(Key(ctx.epoch, observation.feature_id, core_id, entered));
        reason = "no_live_past_anchor";
        if (history != anchors_.end())
          for (const auto &anchor : history->second) {
            const auto *pose = findPose(ctx, anchor.time);
            if (pose && anchor.time < ctx.time - 1e-9) {
              anchor_time = anchor.time;
              anchor_point = anchor.point;
              main_point = shadowMainPoint(current, *pose, anchor.point);
              reason = "ok";
              break;
            }
          }
      }
      if (observation.camera_id < 0 || size_t(observation.camera_id) >= ctx.cameras.size())
        throw std::runtime_error("invalid shadow camera");
      if (observation.match_valid) {
        ltv_error = shadowBearingAngle(ltv_point, ctx.cameras[observation.camera_id], observation.bearing);
        main_error = shadowBearingAngle(main_point, ctx.cameras[observation.camera_id], observation.bearing);
      } else
        reason = "invalid_match";
      if (reason == "ok" && (!std::isfinite(ltv_error) || !std::isfinite(main_error)))
        reason = "invalid_prediction";
      out_ << std::llround((ctx.time - cal.offset) * 1e9) << ',' << ctx.time << ',' << observation.camera_id << ','
           << observation.feature_id << ',' << ctx.epoch << ',' << core_id << ',' << entered << ',' << anchor_time << ','
           << ctx.time - anchor_time << ',' << mature << ',' << past_ready_ << ',' << observation.match_valid << ','
           << std::isfinite(ltv_error) << ',' << std::isfinite(main_error) << ',' << reason << ',' << ltv_error << ',' << main_error;
      for (const auto &point : {ltv_point, main_point, anchor_point})
        for (int i = 0; i < 3; ++i)
          out_ << ',' << point(i);
      if (past_geometry_) {
        double fit_error = nan, latest = nan, condition = nan;
        size_t observations = 0;
        if (std::isfinite(anchor_time) && observation.match_valid) {
          const auto *history = pipeline->manager().history().find(observation.feature_id);
          if (history) {
            const auto input = shadowPastGeometryInput(ctx, *history, anchor_time);
            observations = input.observations.size();
            for (const auto &o : input.observations)
              latest = std::isfinite(latest) ? std::max(latest, input.poses[o.pose_index].t) : input.poses[o.pose_index].t;
            const auto estimate = ltv::LtvSeedEstimator::estimate(input);
            condition = estimate.condition;
            if (estimate.valid)
              fit_error = shadowBearingAngle(estimate.landmark_B, ctx.cameras[observation.camera_id], observation.bearing);
          }
        }
        out_ << ',' << std::isfinite(fit_error) << ',' << fit_error << ',' << observations << ',' << latest << ',' << condition;
      }
      out_ << '\n';
      ++rows_;
    }
    if (!out_)
      throw std::runtime_error("shadow output write failed");
  }
  void finish() {
    out_.flush();
    if (!out_)
      throw std::runtime_error("shadow flush failed");
  }
  uint64_t rows() const { return rows_; }

private:
  using Key = std::tuple<uint64_t, size_t, int, double>;
  struct Anchor {
    double time;
    Eigen::Vector3d point;
  };
  static const ltv::SeedPose *findPose(const ltv::FeaturePipelineContext &ctx, double time) {
    for (const auto &pose : ctx.poses)
      if (std::abs(pose.t - time) <= 1e-9)
        return &pose;
    return nullptr;
  }
  std::ofstream out_;
  double maturity_;
  uint64_t epoch_ = UINT64_MAX, rows_ = 0;
  bool past_ready_ = false, past_geometry_ = false;
  std::map<Key, std::deque<Anchor>> anchors_;
};
} // namespace ov_msckf
