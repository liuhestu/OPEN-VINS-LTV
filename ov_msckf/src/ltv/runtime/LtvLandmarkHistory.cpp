#include "ltv/runtime/LtvLandmarkHistory.h"
#include "ltv/runtime/LtvRuntime.h"
#include <algorithm>
#include <set>
namespace ov_msckf {
bool LtvLandmarkHistory::b3_gate(bool mature, bool ready, double span, const Eigen::Vector3d &l, const Eigen::Vector3d &main) {
  return mature && ready && std::isfinite(span) && span >= .2 - 1e-6 && span <= .55 + 1e-6 && l.allFinite() && main.allFinite() &&
         (l - main).norm() / std::max(main.norm(), .1) <= .02;
}
const ltv::SeedPose *LtvLandmarkHistory::find_pose(const ltv::LtvFeatureContext &ctx, double time) {
  for (const auto &pose : ctx.poses)
    if (std::abs(pose.t - time) <= 1e-9)
      return &pose;
  return nullptr;
}
// Called after IMU prediction, before current bearing/lifecycle/correction.
void LtvLandmarkHistory::before(const LtvRuntime &adapter, const ltv::LtvFeatureContext &ctx, const LtvCalibration &cal) {
  if (epoch_ != ctx.epoch) {
    anchors_.clear();
    epoch_ = ctx.epoch;
  }
  candidates_.clear();
  camera_time_ = ctx.time - cal.offset;
  const auto *pipeline = adapter.feature_frontend();
  if (!pipeline || ctx.execution_pose_index >= ctx.poses.size())
    return;
  const auto &current = ctx.poses.at(ctx.execution_pose_index);
  const auto &records = pipeline->manager().landmarks();
  std::set<size_t> seen;
  for (const auto &observation : ctx.observations) {
    if (observation.camera_id != 0 || !seen.insert(observation.feature_id).second)
      continue;
    auto record = records.find(observation.feature_id);
    auto id = adapter.feature_ids().find(observation.feature_id);
    if (record == records.end() || id == adapter.feature_ids().end() || record->second.entered < 0 ||
        !std::isfinite(record->second.entered) || record->second.phase == ltv::LandmarkPhase::Retired)
      continue;
    const bool mature =
        adapter.feature_frame().management.time - record->second.entered + 1e-12 >= options_.feature_manager.maturity_seconds;
    const bool ready = adapter.availability().ready_V;
    const Key key(ctx.epoch, observation.feature_id, id->second, record->second.entered);
    const int slot = adapter.observer().slotForFeature(id->second);
    auto history = anchors_.find(key);
    if (!adapter.observer_started() || slot < 0 || history == anchors_.end())
      continue;
    for (const auto &anchor : history->second) {
      const auto *pose = find_pose(ctx, anchor.time);
      if (!pose || anchor.time >= ctx.time - 1e-9)
        continue;
      const Eigen::Vector3d l = adapter.observer().state().segment<3>(3 * slot);
      const Eigen::Vector3d main = current.R_WB.transpose() * (pose->R_WB * anchor.point + pose->p_WB - current.p_WB);
      const double span = ctx.time - anchor.time;
      const bool gate = b3_gate(mature, ready, span, l, main);
      const auto &camera = ctx.cameras.at(0);
      const auto ray = camera.R_BC.transpose() * (l - camera.p_BC);
      const auto main_ray = camera.R_BC.transpose() * (main - camera.p_BC);
      // Depth and association validity are available online, never error/GT gates.
      if (gate && observation.match_valid && ray.allFinite() && main_ray.allFinite() && ray.norm() >= .1 && main_ray.norm() >= .1 &&
          ray.z() > 0 && main_ray.z() > 0) {
        candidates_.push_back({ctx.epoch, observation.feature_id, id->second, record->second.entered, l, anchor.point, anchor.time, span});
      }
      break; // Same deterministic oldest-live anchor as the frozen shadow.
    }
  }
  std::sort(candidates_.begin(), candidates_.end(),
            [](const LtvLandmarkCandidate &a, const LtvLandmarkCandidate &b) { return a.feature_id < b.feature_id; });
}
// Save post-camera snapshots, matching the historical anchor convention.
void LtvLandmarkHistory::after(const LtvRuntime &adapter, ltv::LtvFeatureContext ctx, const LtvCalibration &cal, uint64_t frame_epoch) {
  if (std::abs(camera_time_ - (ctx.time - cal.offset)) > options_.time_tolerance_s) {
    candidates_.clear();
    camera_time_ = ctx.time - cal.offset;
  }
  ctx.epoch = frame_epoch;
  const auto *pipeline = adapter.feature_frontend();
  if (!pipeline || ctx.epoch != epoch_)
    return;
  for (const auto &record : pipeline->manager().landmarks()) {
    auto id = adapter.feature_ids().find(record.first);
    if (record.second.entered < 0 || record.second.phase == ltv::LandmarkPhase::Retired || id == adapter.feature_ids().end())
      continue;
    const int slot = adapter.observer().slotForFeature(id->second);
    if (!adapter.observer_started() || slot < 0)
      continue;
    auto &history = anchors_[Key(epoch_, record.first, id->second, record.second.entered)];
    history.erase(std::remove_if(history.begin(), history.end(), [&](const Anchor &a) { return !find_pose(ctx, a.time); }), history.end());
    if (history.empty() || history.back().time < ctx.time)
      history.push_back({ctx.time, adapter.observer().state().segment<3>(3 * slot)});
    if (history.size() > ctx.poses.size())
      throw std::runtime_error("unbounded approximate landmark anchors");
  }
  for (auto it = anchors_.begin(); it != anchors_.end();) {
    const auto id = adapter.feature_ids().find(std::get<1>(it->first));
    const auto record = pipeline->manager().landmarks().find(std::get<1>(it->first));
    if (record == pipeline->manager().landmarks().end() || id == adapter.feature_ids().end() ||
        record->second.phase == ltv::LandmarkPhase::Retired || record->second.entered != std::get<3>(it->first) ||
        id->second != std::get<2>(it->first))
      it = anchors_.erase(it);
    else
      ++it;
  }
  candidates_.erase(std::remove_if(candidates_.begin(), candidates_.end(),
                                   [&](const LtvLandmarkCandidate &c) {
                                     const auto id = adapter.feature_ids().find(c.feature_id);
                                     const auto record = pipeline->manager().landmarks().find(c.feature_id);
                                     return id == adapter.feature_ids().end() || id->second != c.core_id ||
                                            record == pipeline->manager().landmarks().end() || record->second.entered != c.entered ||
                                            record->second.phase == ltv::LandmarkPhase::Retired;
                                   }),
                    candidates_.end());
}

} // namespace ov_msckf
