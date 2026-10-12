#include "ltv/features/LtvLandmarkHistory.h"
#include "ltv/features/LtvFeatureManager.h"
#include <algorithm>
#include <set>
namespace ltv {
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
void LtvLandmarkHistory::before(const LtvFeatureManager &manager, const LtvObserver &observer, const LtvFeatureContext &ctx, bool ready) {
  if (epoch_ != ctx.epoch) {
    anchors_.clear();
    epoch_ = ctx.epoch;
  }
  candidates_.clear();
  event_time_ = ctx.time;
  const auto *pipeline = &manager;
  if (!pipeline || ctx.execution_pose_index >= ctx.poses.size())
    return;
  const auto &current = ctx.poses.at(ctx.execution_pose_index);
  const auto &records = pipeline->pool().landmarks();
  std::set<size_t> seen;
  for (const auto &observation : ctx.observations) {
    if (observation.camera_id != 0 || !seen.insert(observation.feature_id).second)
      continue;
    auto record = records.find(observation.feature_id);
    auto id = manager.ids().find(observation.feature_id);
    if (record == records.end() || id == manager.ids().end() || record->second.entered < 0 || !std::isfinite(record->second.entered) ||
        record->second.phase == ltv::LandmarkPhase::Retired)
      continue;
    const bool mature = manager.frame().management.time - record->second.entered + 1e-12 >= maturity_;
    const Key key(ctx.epoch, observation.feature_id, id->second, record->second.entered);
    const int slot = observer.slotForFeature(id->second);
    auto history = anchors_.find(key);
    if (slot < 0 || history == anchors_.end())
      continue;
    for (const auto &anchor : history->second) {
      const auto *pose = find_pose(ctx, anchor.time);
      if (!pose || anchor.time >= ctx.time - 1e-9)
        continue;
      const Eigen::Vector3d l = observer.state().segment<3>(3 * slot);
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
void LtvLandmarkHistory::after(const LtvFeatureManager &manager, const LtvObserver &observer, const LtvFeatureContext &ctx) {
  if (std::abs(event_time_ - (ctx.time)) > 1e-9) {
    candidates_.clear();
    event_time_ = ctx.time;
  }
  const auto *pipeline = &manager;
  if (!pipeline || ctx.epoch != epoch_)
    return;
  for (const auto &record : pipeline->pool().landmarks()) {
    auto id = manager.ids().find(record.first);
    if (record.second.entered < 0 || record.second.phase == ltv::LandmarkPhase::Retired || id == manager.ids().end())
      continue;
    const int slot = observer.slotForFeature(id->second);
    if (slot < 0)
      continue;
    auto &history = anchors_[Key(epoch_, record.first, id->second, record.second.entered)];
    history.erase(std::remove_if(history.begin(), history.end(), [&](const Anchor &a) { return !find_pose(ctx, a.time); }), history.end());
    if (history.empty() || history.back().time < ctx.time)
      history.push_back({ctx.time, observer.state().segment<3>(3 * slot)});
    if (history.size() > ctx.poses.size())
      throw std::runtime_error("unbounded approximate landmark anchors");
  }
  for (auto it = anchors_.begin(); it != anchors_.end();) {
    const auto id = manager.ids().find(std::get<1>(it->first));
    const auto record = pipeline->pool().landmarks().find(std::get<1>(it->first));
    if (record == pipeline->pool().landmarks().end() || id == manager.ids().end() || record->second.phase == ltv::LandmarkPhase::Retired ||
        record->second.entered != std::get<3>(it->first) || id->second != std::get<2>(it->first))
      it = anchors_.erase(it);
    else
      ++it;
  }
  candidates_.erase(std::remove_if(candidates_.begin(), candidates_.end(),
                                   [&](const LtvLandmarkCandidate &c) {
                                     const auto id = manager.ids().find(c.feature_id);
                                     const auto record = pipeline->pool().landmarks().find(c.feature_id);
                                     return id == manager.ids().end() || id->second != c.core_id ||
                                            record == pipeline->pool().landmarks().end() || record->second.entered != c.entered ||
                                            record->second.phase == ltv::LandmarkPhase::Retired;
                                   }),
                    candidates_.end());
}

} // namespace ltv
