#pragma once
#include "ltv/diagnostics/LtvLandmarkShadow.h"
#include "ltv/fusion/UpdaterLTV.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include <set>

namespace ov_msckf {
// Experimental independent-noise approximation only. Unknown cross blocks are
// omitted, not proved zero. Native stored P is not a physical-chart Sigma claim.
class UpdaterLandmarkApprox {
public:
  explicit UpdaterLandmarkApprox(const LtvOptions &o) : options_(o) {
    if (!o.landmark_approx_log_path.empty()) {
      if (std::ifstream(o.landmark_approx_log_path).good())
        throw std::runtime_error("refusing landmark approximate log overwrite");
      out_.open(o.landmark_approx_log_path);
      if (!out_)
        throw std::runtime_error("cannot open landmark approximate log");
      out_ << "camera_time,epoch,candidates,mature,ready_mature,age_pass,agreement_pass,eligible,selected,rows,"
              "residual_norm,variance_min_m2,variance_max_m2,information_budget_used,consumed,submit_reason,"
              "landmark_delta_norm,landmark_bg_delta_norm,landmark_ba_delta_norm,ekf_calls,p_min_eigenvalue\n";
      out_ << std::setprecision(17);
      const std::string point_path = o.landmark_approx_log_path + ".points.csv";
      if (std::ifstream(point_path).good())
        throw std::runtime_error("refusing landmark point log overwrite");
      points_out_.open(point_path);
      if (!points_out_)
        throw std::runtime_error("cannot open landmark point log");
      points_out_ << "camera_time,epoch,feature_id,core_id,entered,anchor_time,span_s,relative_disagreement,reason,"
                     "selected,variance_m2,information_before,information_after,rx_m,ry_m,rz_m,applied,landmark_delta_norm\n";
      points_out_ << std::setprecision(17);
    }
  }
  static Eigen::Vector3d prediction(const ov_type::PoseJPL &t, const ov_type::PoseJPL &a, const Eigen::Vector3d &l_a, bool fej = false) {
    const Eigen::Matrix3d Rt = fej ? t.Rot_fej() : t.Rot(), Ra = fej ? a.Rot_fej() : a.Rot();
    const Eigen::Vector3d pt = fej ? t.pos_fej() : t.pos(), pa = fej ? a.pos_fej() : a.pos();
    return Rt * (Ra.transpose() * l_a + pa - pt);
  }
  static Eigen::Matrix<double, 3, 12> jacobian(const ov_type::PoseJPL &t, const ov_type::PoseJPL &a, const Eigen::Vector3d &l_a, bool fej) {
    const Eigen::Matrix3d Rt = fej ? t.Rot_fej() : t.Rot(), Ra = fej ? a.Rot_fej() : a.Rot();
    Eigen::Matrix<double, 3, 12> H;
    H << ov_core::skew_x(prediction(t, a, l_a, fej)), -Rt, -Rt * Ra.transpose() * ov_core::skew_x(l_a), Rt;
    return H;
  }
  static bool b3_gate(bool mature, bool ready, double span, const Eigen::Vector3d &l, const Eigen::Vector3d &main) {
    return mature && ready && std::isfinite(span) && span >= .2 - 1e-6 && span <= .55 + 1e-6 && l.allFinite() && main.allFinite() &&
           (l - main).norm() / std::max(main.norm(), .1) <= .02;
  }
  // Called after IMU prediction, before current bearing/lifecycle/correction.
  void before(const LtvAdapter &adapter, const ltv::FeaturePipelineContext &ctx, const LtvCalibration &cal) {
    if (epoch_ != ctx.epoch) {
      anchors_.clear();
      epoch_ = ctx.epoch;
    }
    candidates_.clear();
    gate_rows_.clear();
    counts_.assign(7, 0);
    camera_time_ = ctx.time - cal.offset;
    offset_ = cal.offset;
    const auto *pipeline = adapter.landmark_adapter();
    if (!pipeline || ctx.execution_pose_index >= ctx.poses.size())
      return;
    const auto &current = ctx.poses.at(ctx.execution_pose_index);
    const auto &records = pipeline->manager().landmarks();
    std::set<size_t> seen;
    for (const auto &observation : ctx.observations) {
      if (observation.camera_id != 0 || !seen.insert(observation.feature_id).second)
        continue;
      ++counts_[0];
      const size_t row_index = gate_rows_.size();
      gate_rows_.push_back(GateRow());
      auto &row = gate_rows_.back();
      row.key = Key(ctx.epoch, observation.feature_id, -1, -1);
      auto record = records.find(observation.feature_id);
      auto id = adapter.feature_ids().find(observation.feature_id);
      if (record == records.end() || id == adapter.feature_ids().end() || record->second.entered < 0 ||
          !std::isfinite(record->second.entered) || record->second.phase == ltv::LandmarkPhase::Retired)
        continue;
      const bool mature =
          adapter.feature_frame().management.time - record->second.entered + 1e-12 >= options_.feature_manager.maturity_seconds;
      counts_[1] += mature;
      const bool ready = adapter.readiness_health().ready_V;
      counts_[2] += mature && ready;
      const Key key(ctx.epoch, observation.feature_id, id->second, record->second.entered);
      row.key = key;
      row.reason = "no_live_anchor";
      const int slot = adapter.core().slotForFeature(id->second);
      auto history = anchors_.find(key);
      if (!adapter.core().started() || slot < 0 || history == anchors_.end())
        continue;
      for (const auto &anchor : history->second) {
        const auto *pose = find_pose(ctx, anchor.time);
        if (!pose || anchor.time >= ctx.time - 1e-9)
          continue;
        const Eigen::Vector3d l = adapter.core().state().segment<3>(3 * slot);
        const Eigen::Vector3d main = shadowMainPoint(current, *pose, anchor.point);
        const double span = ctx.time - anchor.time;
        row.anchor_time = anchor.time;
        row.span = span;
        row.disagreement = (l - main).norm() / std::max(main.norm(), .1);
        const bool age = mature && ready && span >= .2 - 1e-6 && span <= .55 + 1e-6;
        row.reason = !mature ? "immature" : !ready ? "not_ready" : !age ? "age" : "disagreement";
        counts_[3] += age;
        const bool gate = b3_gate(mature, ready, span, l, main);
        counts_[4] += gate;
        const auto &camera = ctx.cameras.at(0);
        const auto ray = camera.R_BC.transpose() * (l - camera.p_BC);
        const auto main_ray = camera.R_BC.transpose() * (main - camera.p_BC);
        if (gate)
          row.reason = "invalid_geometry_or_match";
        // Depth and association validity are available online, never error/GT gates.
        if (gate && observation.match_valid && ray.allFinite() && main_ray.allFinite() && ray.norm() >= .1 && main_ray.norm() >= .1 &&
            ray.z() > 0 && main_ray.z() > 0) {
          row.reason = "eligible";
          candidates_.push_back({key, anchor, l, span, row_index});
          ++counts_[5];
        }
        break; // Same deterministic oldest-live anchor as the frozen shadow.
      }
    }
    std::sort(candidates_.begin(), candidates_.end(), [](const Candidate &a, const Candidate &b) { return a.key < b.key; });
  }
  // Save post-camera snapshots, matching the historical anchor convention.
  void after(const LtvAdapter &adapter, ltv::FeaturePipelineContext ctx, const LtvCalibration &cal, uint64_t frame_epoch) {
    if (std::abs(camera_time_ - (ctx.time - cal.offset)) > options_.time_tolerance_s) {
      candidates_.clear();
      gate_rows_.clear();
      counts_.assign(7, 0);
      camera_time_ = ctx.time - cal.offset;
      offset_ = cal.offset;
    }
    ctx.epoch = frame_epoch;
    const auto *pipeline = adapter.landmark_adapter();
    if (!pipeline || ctx.epoch != epoch_)
      return;
    for (const auto &record : pipeline->manager().landmarks()) {
      auto id = adapter.feature_ids().find(record.first);
      if (record.second.entered < 0 || record.second.phase == ltv::LandmarkPhase::Retired || id == adapter.feature_ids().end())
        continue;
      const int slot = adapter.core().slotForFeature(id->second);
      if (!adapter.core().started() || slot < 0)
        continue;
      auto &history = anchors_[Key(epoch_, record.first, id->second, record.second.entered)];
      history.erase(std::remove_if(history.begin(), history.end(), [&](const Anchor &a) { return !find_pose(ctx, a.time); }),
                    history.end());
      if (history.empty() || history.back().time < ctx.time)
        history.push_back({ctx.time, adapter.core().state().segment<3>(3 * slot)});
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
  }
  MeasurementBlock build(const std::shared_ptr<State> &state, const LtvAdapter &adapter, MeasurementBlock base) {
    information_ = variance_min_ = variance_max_ = residual_ = 0;
    if (!options_.enable_landmark_approx || !base.token.available || !base.token.snapshot.valid ||
        std::abs(state->_timestamp - camera_time_) > options_.time_tolerance_s)
      return base;
    for (const auto &candidate : candidates_) {
      auto &row = gate_rows_.at(candidate.row_index);
      row.reason = "identity_changed";
      if (counts_[6] >= options_.landmark_approx_max_points) {
        row.reason = "frame_supply_limit";
        continue;
      }
      const auto &key = candidate.key;
      if (std::get<0>(key) != base.token.epoch)
        continue;
      const auto *pipeline = adapter.landmark_adapter();
      const auto id = adapter.feature_ids().find(std::get<1>(key));
      const auto record = pipeline->manager().landmarks().find(std::get<1>(key));
      if (id == adapter.feature_ids().end() || record == pipeline->manager().landmarks().end() || id->second != std::get<2>(key) ||
          record->second.entered != std::get<3>(key) || record->second.phase == ltv::LandmarkPhase::Retired)
        continue;
      std::shared_ptr<ov_type::PoseJPL> anchor;
      for (const auto &clone : state->_clones_IMU)
        if (std::abs(clone.first + offset_ - candidate.anchor.time) <= 1e-9)
          anchor = clone.second;
      if (!anchor) {
        row.reason = "anchor_not_live";
        continue;
      }
      MeasurementBlock point;
      point.order = {state->_imu->q(), state->_imu->p(), anchor->q(), anchor->p()};
      point.H = jacobian(*state->_imu->pose(), *anchor, candidate.anchor.point, state->_options.do_fej);
      point.res = candidate.point - prediction(*state->_imu->pose(), *anchor, candidate.anchor.point);
      const double sigma = options_.landmark_approx_sigma_floor_m + .02 * candidate.point.norm();
      double variance = sigma * sigma * (1 + candidate.span / .55); // m^2; heuristic, not P_Riccati.
      const auto P = StateHelper::get_marginal_covariance(state, point.order);
      const double signal = (point.H * P * point.H.transpose()).trace();
      if (!std::isfinite(signal) || signal < 0)
        throw std::runtime_error("invalid native landmark information trace");
      const double cap = options_.landmark_approx_information_cap / options_.landmark_approx_max_points;
      variance = std::max(variance, signal / cap); // Modify R before K; never truncate K*r.
      point.R = variance * Eigen::Matrix3d::Identity();
      std::string reason;
      if (!UpdaterLTV::validate(state, point, reason))
        throw std::runtime_error("invalid landmark approximate block: " + reason);
      row.reason = "selected";
      row.selected = true;
      row.variance = variance;
      row.information_before = signal / (sigma * sigma * (1 + candidate.span / .55));
      row.information_after = signal / variance;
      row.residual = point.res;
      base = UpdaterLTV::merge(base, point);
      ++counts_[6];
      information_ += signal / variance;
      residual_ += point.res.squaredNorm();
      variance_min_ = variance_min_ == 0 ? variance : std::min(variance_min_, variance);
      variance_max_ = std::max(variance_max_, variance);
    }
    base.receipt->diagnostics.landmark_rows = 3 * counts_[6];
    return base;
  }
  void finish_frame(const LtvDiagnostics &d) {
    if (!out_)
      return;
    out_ << camera_time_ << ',' << epoch_;
    for (int n : counts_)
      out_ << ',' << n;
    out_ << ',' << d.landmark_rows << ',' << std::sqrt(residual_) << ',' << variance_min_ << ',' << variance_max_ << ',' << information_
         << ',' << d.consumed << ',' << d.submit_reason << ',' << d.landmark_update_norm << ',' << d.landmark_bg_update_norm << ','
         << d.landmark_ba_update_norm << ',' << d.ekf_calls << ',' << d.p_min_eigenvalue << '\n';
    for (const auto &row : gate_rows_) {
      points_out_ << camera_time_ << ',' << std::get<0>(row.key) << ',' << std::get<1>(row.key) << ',' << std::get<2>(row.key) << ','
                  << std::get<3>(row.key) << ',' << row.anchor_time << ',' << row.span << ',' << row.disagreement << ',' << row.reason
                  << ',' << row.selected << ',' << row.variance << ',' << row.information_before << ',' << row.information_after << ','
                  << row.residual.x() << ',' << row.residual.y() << ',' << row.residual.z() << ','
                  << (row.selected && d.consumed && d.landmark_rows > 0 && d.submit_reason == "joint_applied") << ','
                  << d.landmark_update_norm << '\n';
    }
    if (!out_ || !points_out_)
      throw std::runtime_error("landmark approximate log failed");
  }

private:
  using Key = std::tuple<uint64_t, size_t, int, double>;
  struct Anchor {
    double time;
    Eigen::Vector3d point;
  };
  struct Candidate {
    Key key;
    Anchor anchor;
    Eigen::Vector3d point;
    double span;
    size_t row_index;
  };
  struct GateRow {
    Key key;
    double anchor_time = -1, span = -1, disagreement = -1, variance = 0, information_before = 0, information_after = 0;
    std::string reason = "no_past_identity";
    bool selected = false;
    Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  };
  static const ltv::SeedPose *find_pose(const ltv::FeaturePipelineContext &ctx, double time) {
    for (const auto &pose : ctx.poses)
      if (std::abs(pose.t - time) <= 1e-9)
        return &pose;
    return nullptr;
  }
  LtvOptions options_;
  uint64_t epoch_ = 0;
  double camera_time_ = -1, offset_ = 0, information_ = 0, variance_min_ = 0, variance_max_ = 0, residual_ = 0;
  std::map<Key, std::deque<Anchor>> anchors_;
  std::vector<Candidate> candidates_;
  std::vector<GateRow> gate_rows_;
  std::vector<int> counts_ = std::vector<int>(7, 0);
  std::ofstream out_, points_out_;
};
} // namespace ov_msckf
