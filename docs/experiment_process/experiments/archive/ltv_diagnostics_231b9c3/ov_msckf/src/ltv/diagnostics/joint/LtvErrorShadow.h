#pragma once

#include "ltv/diagnostics/joint/LtvDiscreteSensitivity.h"
#include <Eigen/Dense>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace ltv {

// A joint error map, deliberately separate from the observer's Riccati metric.
// Layout and source covariance must be supplied explicitly by the caller. The
// cross covariance D is Cov(old joint error, source); no implicit independence.
inline Eigen::MatrixXd shadowJointMap(const Eigen::MatrixXd &sigma, const Eigen::MatrixXd &map, const Eigen::MatrixXd &source_map,
                                      const Eigen::MatrixXd &source_covariance, const Eigen::MatrixXd &error_source_cross) {
  if (sigma.rows() != sigma.cols() || map.cols() != sigma.rows() || source_map.rows() != map.rows() ||
      source_map.cols() != source_covariance.rows() || source_covariance.rows() != source_covariance.cols() ||
      error_source_cross.rows() != sigma.rows() || error_source_cross.cols() != source_map.cols())
    throw std::invalid_argument("shadow joint map dimension mismatch");
  return map * sigma * map.transpose() + source_map * source_covariance * source_map.transpose() +
         map * error_source_cross * source_map.transpose() + source_map * error_source_cross.transpose() * map.transpose();
}

// Named fixed-gain blocks retain the historical diagnostic. Named full-gain
// blocks differentiate mean and P_R jointly. Neither is a physical Sigma_L;
// source laws and true-error initial conditions are supplied separately.
class LtvErrorShadow {
public:
  explicit LtvErrorShadow(bool emit = true) {
    if (!emit)
      return;
    const char *path = std::getenv("LTV_JOINT_SHADOW_PATH");
    if (!path || !*path)
      return;
    // Hardened adapter copies/replaces observers transactionally. Copies share
    // the sink; constructing an epoch replacement must not truncate its log.
    static std::map<std::string, std::weak_ptr<std::ofstream>> sinks;
    static std::map<std::string, std::weak_ptr<MatrixSink>> matrix_sinks;
    output_ = sinks[path].lock();
    if (output_) {
      matrix_sink_ = matrix_sinks[path].lock();
      return;
    }
    std::ifstream existing(path);
    if (existing.good())
      throw std::runtime_error("refusing joint shadow overwrite");
    output_ = std::make_shared<std::ofstream>(path);
    sinks[path] = output_;
    matrix_sink_ = std::make_shared<MatrixSink>(path);
    matrix_sinks[path] = matrix_sink_;
    if (!*output_)
      throw std::runtime_error("cannot open joint shadow output");
    *output_ << std::setprecision(17)
             << "event,time,epoch,transaction,dimension,imu_steps,births,retired,substeps,transition_norm,source_map_norm,"
                "conditional_unit_source_gram_trace,persistent_corrected_input_offset_norm,local_feature_ids,valid_unconditional,missing_"
                "blocks,full_direction_dimension,full_source_capacity_events,spectral_floor_boundary_events\n";
  }
  bool enabled() const { return output_ && output_->is_open(); }
  const Eigen::MatrixXd &imuSourceMap() const { return imu_b_; }
  const Eigen::MatrixXd &cameraTransition() const { return camera_f_; }
  const Eigen::MatrixXd &cameraSourceMap() const { return camera_b_; }
  void setEpoch(unsigned long epoch) {
    if (epoch_ != epoch) {
      anchors_.clear();
      anchor_times_.clear();
      retired_ids_.clear();
    }
    epoch_ = epoch;
  }
  void reset(double time) {
    if (!enabled())
      return;
    ++epoch_;
    imu_steps_ = 0;
    persistent_input_offset_ = Eigen::MatrixXd::Zero(6, 6);
    observed_ids_.clear();
    retained_ids_.clear();
    anchors_.clear();
    anchor_times_.clear();
    retired_ids_.clear();
    camera_gains_.clear();
    full_mean_ = Eigen::MatrixXd::Zero(6, 6);
    full_riccati_.assign(6, Eigen::MatrixXd::Zero(6, 6));
    seed_columns_.clear();
    source_capacity_events_ = spectral_boundaries_ = 0;
    write("reset", time, 6, 0, 0, 0, 0, 0);
    record("reset", time, {{"persistent_input_offset", &persistent_input_offset_}});
  }
  void imu(double time, double dt, const Eigen::MatrixXd &f, const Eigen::MatrixXd &b,
           const Eigen::MatrixXd &nominal_state = Eigen::MatrixXd()) {
    if (!enabled())
      return;
    // One persistent corrected-input offset source, NOT a fresh bias draw per
    // IMU step. Birth seed is held fixed here; its true offset dependency is
    // UNKNOWN and prevents statistical validity. Units are (accel, gyro).
    if (persistent_input_offset_.rows() != f.cols())
      throw std::runtime_error("shadow input sensitivity lifecycle mismatch");
    persistent_input_offset_ = (f * persistent_input_offset_ - b).eval();
    imu_b_ = b;
    ++imu_steps_;
    last_imu_f_norm_ = f.norm();
    last_imu_b_norm_ = b.norm();
    record(
        "imu", time,
        {{"F", &f}, {"B_corrected_acc_gyro", &b}, {"state_before", &nominal_state}, {"persistent_input_offset", &persistent_input_offset_}},
        dt);
  }
  void lifecycle(double time, const Eigen::MatrixXd &map, int births, int retired, const std::vector<int> &ids) {
    if (enabled()) {
      persistent_input_offset_ = (map * persistent_input_offset_).eval();
      full_mean_ = (map * full_mean_).eval();
      for (auto &dp : full_riccati_)
        dp = (map * dp * map.transpose()).eval();
      // Seed columns refer to explicit mean inputs, not independent draws.
      // Their geometry/main/bearing law is supplied by the joint source model.
      for (size_t slot = 0; slot < ids.size(); ++slot) {
        if (!seed_columns_.count(ids[slot]) && full_mean_.cols() < 102) {
          const int col = full_mean_.cols();
          seed_columns_[ids[slot]] = col;
          full_mean_.conservativeResize(map.rows(), col + 3);
          full_mean_.rightCols(3).setZero();
          for (int k = 0; k < 3; ++k)
            full_riccati_.push_back(Eigen::MatrixXd::Zero(map.rows(), map.rows()));
        }
      }
      // Retired columns may affect retained states through previous gain
      // coupling. Keep their sufficient derivative history; never delete merely
      // because a feature leaves the main clone/observer layout.
      write("lifecycle", time, map.rows(), births, retired, 0, map.norm(), 0);
      const std::set<int> retained(ids.begin(), ids.end());
      for (auto it = anchors_.begin(); it != anchors_.end();) {
        if (!retained.count(it->first)) {
          retired_ids_.insert(it->first);
          anchor_times_.erase(it->first);
          it = anchors_.erase(it);
        } else
          ++it;
      }
      for (size_t slot = 0; slot < ids.size(); ++slot) {
        if (retired_ids_.count(ids[slot]))
          throw std::runtime_error("shadow rejects retired anchor identity revival");
        if (!anchors_.count(ids[slot])) {
          anchors_[ids[slot]] = persistent_input_offset_.middleRows(3 * slot, 3);
          anchor_times_[ids[slot]] = -1;
        }
      }
      retained_ids_ = ids;
      record("lifecycle", time, {{"M", &map}, {"persistent_input_offset", &persistent_input_offset_}});
    }
  }
  // Persistent corrected-input offset directions, INCLUDING their P_R/gain
  // dependency. These are local derivatives; physical source law is separate.
  void fullImu(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &a, const Eigen::MatrixXd &input,
               const Eigen::Vector3d &acc, double dt) {
    if (!enabled())
      return;
    for (int k = 0; k < full_mean_.cols(); ++k) {
      Eigen::MatrixXd da = Eigen::MatrixXd::Zero(x.size(), x.size());
      Eigen::Vector3d dacc = Eigen::Vector3d::Zero();
      if (k < 3)
        dacc[k] = -1.;
      else if (k < 6) {
        Eigen::Vector3d dw = Eigen::Vector3d::Zero();
        dw[k - 3] = -1.;
        Eigen::Matrix3d skew;
        skew << 0, -dw.z(), dw.y(), dw.z(), 0, -dw.x(), -dw.y(), dw.x(), 0;
        for (int i = 0; i < x.size(); i += 3)
          da.block<3, 3>(i, i) = -skew;
      }
      ObserverTangent d{full_mean_.col(k), full_riccati_.at(k)};
      const auto out = LtvDiscreteSensitivity::imu(x, p, a, input, acc, dt, d, da, dacc);
      full_mean_.col(k) = out.mean;
      full_riccati_[k] = out.riccati;
    }
  }
  void fullCameraSubstep(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &c, const Eigen::VectorXd &y,
                         double hq) {
    if (!enabled())
      return;
    const Eigen::MatrixXd dc = Eigen::MatrixXd::Zero(c.rows(), c.cols());
    const Eigen::VectorXd dy = Eigen::VectorXd::Zero(y.rows());
    for (int k = 0; k < full_mean_.cols(); ++k) {
      ObserverTangent d{full_mean_.col(k), full_riccati_.at(k)};
      const auto out = LtvDiscreteSensitivity::camera(x, p, c, y, hq, d, dc, dy);
      full_mean_.col(k) = out.mean;
      full_riccati_[k] = out.riccati;
    }
  }
  void fullSanitize(const Eigen::MatrixXd &p, double floor) {
    if (!enabled())
      return;
    for (auto &dp : full_riccati_) {
      bool boundary = false;
      dp = LtvDiscreteSensitivity::spectralFloor(p, dp, floor, &boundary);
      if (boundary)
        ++spectral_boundaries_;
    }
    for (auto &dp : bearing_riccati_)
      dp = LtvDiscreteSensitivity::spectralFloor(p, dp, floor);
  }
  void fullSeed(int id, int slot, bool apply) {
    if (enabled() && apply) {
      auto found = seed_columns_.find(id);
      if (found == seed_columns_.end()) {
        ++source_capacity_events_;
        return;
      }
      full_mean_.block<3, 3>(3 * slot, found->second).setIdentity();
    }
  }
  void fullBearingSubstep(const Eigen::VectorXd &x, const Eigen::MatrixXd &p, const Eigen::MatrixXd &c, const Eigen::VectorXd &y, double hq,
                          const std::vector<int> &slots, const std::vector<Eigen::Vector3d> &raw, const Eigen::Matrix3d &rbc,
                          const Eigen::Vector3d &pbc) {
    if (!enabled())
      return;
    if (full_bearing_.cols() != 3 * static_cast<int>(slots.size())) {
      full_bearing_ = Eigen::MatrixXd::Zero(x.size(), 3 * slots.size());
      bearing_riccati_.assign(3 * slots.size(), Eigen::MatrixXd::Zero(x.size(), x.size()));
    }
    for (int k = 0; k < full_bearing_.cols(); ++k) {
      const int obs = k / 3, axis = k % 3;
      const Eigen::Vector3d z = rbc * raw[obs].normalized();
      const Eigen::Vector3d dz = rbc * LtvDiscreteSensitivity::normalizedBearing(raw[obs]).col(axis);
      const Eigen::Matrix3d dpi = LtvDiscreteSensitivity::projection(z, dz);
      Eigen::MatrixXd dc = Eigen::MatrixXd::Zero(c.rows(), c.cols());
      dc.block<3, 3>(3 * obs, 3 * slots[obs]) = dpi;
      Eigen::VectorXd dy = Eigen::VectorXd::Zero(y.size());
      dy.segment<3>(3 * obs) = dpi * pbc;
      ObserverTangent d{full_bearing_.col(k), bearing_riccati_[k]};
      const auto out = LtvDiscreteSensitivity::camera(x, p, c, y, hq, d, dc, dy);
      full_bearing_.col(k) = out.mean;
      bearing_riccati_[k] = out.riccati;
    }
  }
  const Eigen::MatrixXd &fullBearingMap() const { return full_bearing_; }
  const Eigen::MatrixXd &fullInputOffsetMap() const { return full_mean_; }
  void beginCamera() {
    camera_gains_.clear();
    full_bearing_.resize(0, 0);
    bearing_riccati_.clear();
  }
  const std::vector<Eigen::MatrixXd> &cameraGains() const { return camera_gains_; }
  void cameraSubstep(const Eigen::MatrixXd &gain) { camera_gains_.push_back(gain); }
  void camera(double time, const Eigen::MatrixXd &f, const Eigen::MatrixXd &b, int substeps, const std::vector<int> &ids) {
    if (enabled()) {
      // The isolated same-frame source contribution is actually propagated with
      // the joint-map implementation. Unit tangent-source covariance measures
      // sensitivity only: it is explicitly NOT physical bearing uncertainty.
      // Unknown prior/source and main/source blocks are never filled with zeros
      // in an unconditional distribution.
      const Eigen::MatrixXd unit_source = Eigen::MatrixXd::Identity(b.cols(), b.cols());
      const Eigen::MatrixXd fixed_initial = Eigen::MatrixXd::Zero(f.cols(), f.cols());
      const Eigen::MatrixXd fixed_initial_cross = Eigen::MatrixXd::Zero(f.cols(), b.cols());
      persistent_input_offset_ = (f * persistent_input_offset_).eval();
      observed_ids_ = ids;
      source_ids_.clear();
      for (int id : ids)
        source_ids_ += std::to_string(id) + ";";
      camera_f_ = f;
      camera_b_ = b;
      frame_source_gram_ = shadowJointMap(fixed_initial, f, b, unit_source, fixed_initial_cross);
      write("camera", time, f.rows(), 0, 0, substeps, f.norm(), b.norm());
      write("imu_summary", time, f.rows(), 0, 0, 0, last_imu_f_norm_, last_imu_b_norm_);
      const int point_dimension = 3 * static_cast<int>(retained_ids_.size());
      Eigen::MatrixXd anchor_map(point_dimension, 6);
      for (size_t slot = 0; slot < retained_ids_.size(); ++slot)
        anchor_map.middleRows(3 * slot, 3) = anchors_.at(retained_ids_[slot]);
      const Eigen::MatrixXd point_map = persistent_input_offset_.topRows(point_dimension);
      const Eigen::MatrixXd sigma_aa = anchor_map * anchor_map.transpose();
      const Eigen::MatrixXd sigma_tt = point_map * point_map.transpose();
      const Eigen::MatrixXd sigma_at = anchor_map * point_map.transpose();
      record("camera", time,
             {{"full_gain_offset_seed_mean_inputs", &full_mean_},
              {"full_gain_same_frame_raw_bearing", &full_bearing_},
              {"F_fixed_gain", &f},
              {"B_same_bearing", &b},
              {"conditional_unit_source_gram", &frame_source_gram_},
              {"persistent_input_offset", &persistent_input_offset_},
              {"conditional_anchor_source_map", &anchor_map},
              {"conditional_current_point_source_map", &point_map},
              {"conditional_unit_Sigma_aa", &sigma_aa},
              {"conditional_unit_Sigma_tt_crosspoint", &sigma_tt},
              {"conditional_unit_Sigma_at", &sigma_at}});
      // Rolling previous-camera diagnostic anchor. Save only AFTER emitting
      // cross-time blocks, so aa/at reference the prior physical snapshot.
      for (size_t slot = 0; slot < retained_ids_.size(); ++slot) {
        anchors_[retained_ids_[slot]] = point_map.middleRows(3 * slot, 3);
        anchor_times_[retained_ids_[slot]] = time;
      }
      record("anchor_replace", time, {{"conditional_anchor_source_map", &point_map}});
      output_->flush();
      matrix_sink_->index.flush();
    }
  }

private:
  struct MatrixSink {
    std::ofstream data, index;
    explicit MatrixSink(const std::string &path) {
      for (const auto &suffix : {".matrices.bin", ".matrices.jsonl"}) {
        std::ifstream existing(path + suffix);
        if (existing.good())
          throw std::runtime_error("refusing shadow matrix overwrite");
      }
      data.open(path + ".matrices.bin", std::ios::binary);
      index.open(path + ".matrices.jsonl");
      if (!data || !index)
        throw std::runtime_error("cannot open shadow matrix stream");
      index << std::setprecision(17);
    }
  };
  void record(const char *event, double time, std::initializer_list<std::pair<const char *, const Eigen::MatrixXd *>> matrices,
              double dt = 0) {
    auto &sink = *matrix_sink_;
    sink.index << "{\"event\":\"" << event << "\",\"time\":" << time << ",\"dt\":" << dt << ",\"epoch\":" << epoch_
               << ",\"transaction\":\"observer_attempt_uncommitted\",\"valid_unconditional\":false,\"gain_schedule\":\"mixed_named_"
                  "blocks\",\"retained_"
                  "local_ids\":[";
    for (size_t i = 0; i < retained_ids_.size(); ++i) {
      if (i)
        sink.index << ',';
      sink.index << retained_ids_[i];
    }
    sink.index << "],\"anchor_times\":[";
    for (size_t i = 0; i < retained_ids_.size(); ++i) {
      if (i)
        sink.index << ',';
      const int id = retained_ids_[i];
      const auto found = anchor_times_.find(id);
      const double anchor_time = found == anchor_times_.end() ? -1 : found->second;
      sink.index << "{\"local_id\":" << id << ",\"time\":" << anchor_time << ",\"lag_available\":" << (anchor_time >= 0 ? "true" : "false")
                 << '}';
    }
    sink.index << "],\"observed_local_source_ids\":[";
    if (std::string(event) == "camera")
      for (size_t i = 0; i < observed_ids_.size(); ++i) {
        if (i)
          sink.index << ',';
        sink.index << observed_ids_[i];
      }
    sink.index << "],\"full_source_capacity_events\":" << source_capacity_events_
               << ",\"spectral_floor_boundary_events\":" << spectral_boundaries_ << ",\"seed_direction_columns\":[";
    bool first_seed = true;
    for (const auto &entry : seed_columns_) {
      if (!first_seed)
        sink.index << ',';
      first_seed = false;
      sink.index << "{\"local_id\":" << entry.first << ",\"column\":" << entry.second << '}';
    }
    sink.index << "],\"matrices\":{";
    bool first = true;
    for (const auto &entry : matrices) {
      const auto &matrix = *entry.second;
      uint64_t nonzero = 0;
      for (int column = 0; column < matrix.cols(); ++column)
        for (int row = 0; row < matrix.rows(); ++row)
          if (matrix(row, column) != 0)
            ++nonzero;
      const bool sparse = 16 * nonzero < 8 * static_cast<uint64_t>(matrix.size());
      const auto offset = static_cast<uint64_t>(sink.data.tellp());
      if (sparse) {
        for (int column = 0; column < matrix.cols(); ++column)
          for (int row = 0; row < matrix.rows(); ++row)
            if (matrix(row, column) != 0) {
              const uint32_t r = row, c = column;
              const double value = matrix(row, column);
              sink.data.write(reinterpret_cast<const char *>(&r), 4);
              sink.data.write(reinterpret_cast<const char *>(&c), 4);
              sink.data.write(reinterpret_cast<const char *>(&value), 8);
            }
      } else
        sink.data.write(reinterpret_cast<const char *>(matrix.data()), 8 * matrix.size());
      if (!first)
        sink.index << ',';
      first = false;
      sink.index << '\"' << entry.first << "\":{\"offset\":" << offset << ",\"rows\":" << matrix.rows() << ",\"cols\":" << matrix.cols()
                 << ",\"nonzero\":" << nonzero << ",\"encoding\":\""
                 << (sparse ? "native_uint32_row_col_float64_triplets" : "native_float64_column_major") << "\"}";
    }
    sink.index << "}}\n";
    if (!sink.data || !sink.index)
      throw std::runtime_error("shadow matrix stream write failed");
  }
  void write(const char *event, double time, int dimension, int births, int retired, int substeps, double f_norm, double b_norm) {
    *output_ << event << ',' << time << ',' << epoch_ << ",observer_attempt_uncommitted," << dimension << ',' << imu_steps_ << ',' << births
             << ',' << retired << ',' << substeps << ',' << f_norm << ',' << b_norm << ","
             << (std::string(event) == "camera" ? frame_source_gram_.trace() : 0.0) << "," << persistent_input_offset_.norm() << ","
             << (std::string(event) == "camera" ? source_ids_ : "")
             << ",false,seed_history_joint;main_visual_reset_cross;interpolated_imu_source_cross;gain_schedule_sensitivity;independent_"
                "landmark_truth"
             << "," << full_mean_.cols() << "," << source_capacity_events_ << "," << spectral_boundaries_ << '\n';
    if (!*output_)
      throw std::runtime_error("joint shadow write failed");
  }
  std::string source_ids_;
  Eigen::MatrixXd frame_source_gram_, camera_f_, camera_b_, imu_b_;
  Eigen::MatrixXd persistent_input_offset_, full_mean_;
  std::vector<Eigen::MatrixXd> full_riccati_, bearing_riccati_;
  Eigen::MatrixXd full_bearing_;
  std::map<int, int> seed_columns_;
  unsigned long spectral_boundaries_ = 0, source_capacity_events_ = 0;
  std::shared_ptr<std::ofstream> output_;
  std::shared_ptr<MatrixSink> matrix_sink_;
  std::vector<int> retained_ids_, observed_ids_;
  std::map<int, Eigen::MatrixXd> anchors_;
  std::map<int, double> anchor_times_;
  std::set<int> retired_ids_;
  std::vector<Eigen::MatrixXd> camera_gains_;
  unsigned long epoch_ = 0, imu_steps_ = 0;
  double last_imu_f_norm_ = 0, last_imu_b_norm_ = 0;
};

} // namespace ltv
