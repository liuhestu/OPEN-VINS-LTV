#include "ltv/diagnostics/joint/LtvFiniteJointShadow.h"
#include "ltv/diagnostics/joint/LtvMainCrossShadow.h"
#include "ltv/observer/LtvAdapter.h"
#include "utils/quat_ops.h"
#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <set>
namespace ltv {
LtvFiniteJointShadow &LtvFiniteJointShadow::instance() {
  static LtvFiniteJointShadow x;
  return x;
}
LtvFiniteJointShadow::LtvFiniteJointShadow() {
  const char *path = std::getenv("LTV_FINITE_JOINT_PATH");
  enabled_ = path && *path;
  if (!enabled_)
    return;
  for (const auto &suffix : {"", ".matrices.bin", ".matrices.jsonl"})
    if (std::ifstream(std::string(path) + suffix).good())
      throw std::runtime_error("refusing finite joint overwrite");
  output_.open(path);
  matrix_data_.open(std::string(path) + ".matrices.bin", std::ios::binary);
  matrix_index_.open(std::string(path) + ".matrices.jsonl");
  if (!output_ || !matrix_data_ || !matrix_index_)
    throw std::runtime_error("finite joint output failed");
  output_ << std::setprecision(17)
          << "event,time,epoch,feature,main_dimension,observer_dimension,factor_columns,main_trace,observer_trace,main_observer_cross_norm,"
             "anchors,missing_events,local_program_minus_stored_norm,raw_prune_cutoff,pixel_prune_cutoff,visual_remainder_min_eigenvalue,"
             "visual_"
             "roundoff_floors,scope\n";
  matrix_index_ << std::setprecision(17);
  factors_ = std::make_shared<LtvJointFactors>();
}
void LtvFiniteJointShadow::initialize(const void *owner, const Eigen::MatrixXd &p) {
  if (!enabled_)
    return;
  if (owner_ != owner) {
    owner_ = owner;
    factors_ = std::make_shared<LtvJointFactors>();
    factors_->initial("main", p);
    stored_ = p;
    observer_.reset();
    sensitivity_.reset();
    active_ = false;
    anchors_.clear();
    source_times_.clear();
    source_laws_.clear();
    raw_times_.clear();
    raw_prune_cutoff_ = pixel_prune_cutoff_ = -1;
    pre_visual_main_.resize(0, 0);
    nominal_poses_.resize(0, 0);
    time_ = -1;
  }
}
void LtvFiniteJointShadow::reset(const void *owner, const Eigen::MatrixXd &p) {
  if (!enabled_)
    return;
  owner_ = nullptr;
  initialize(owner, p);
}
void LtvFiniteJointShadow::mainMap(const void *owner, const Eigen::MatrixXd &p, const Eigen::MatrixXd &map, const std::string &event) {
  if (!enabled_)
    return;
  initialize(owner, p);
  if (event == "main_imu_propagation")
    return;
  factors_->set("main", (map * factors_->block("main")).eval());
}
void LtvFiniteJointShadow::freezeBias(const Eigen::MatrixXd &selector) {
  if (!enabled_)
    return;
  factors_->set("bias_prior", (selector * factors_->block("main")).eval());
}
void LtvFiniteJointShadow::rawImuReceipt(const std::vector<double> &timestamps, double sigma_acc, double sigma_gyro) {
  if (!enabled_)
    return;
  if (!source_laws_.empty() && (sigma_acc != sigma_acc_ || sigma_gyro != sigma_gyro_))
    throw std::runtime_error("raw source noise density changed within model version");
  raw_times_ = timestamps;
  sigma_acc_ = sigma_acc;
  sigma_gyro_ = sigma_gyro;
}
Eigen::MatrixXd LtvFiniteJointShadow::endpoint(double time) {
  auto hi = std::lower_bound(raw_times_.begin(), raw_times_.end(), time);
  if (hi == raw_times_.end() || raw_times_.size() < 2)
    throw std::runtime_error("finite joint IMU source bracket lost");
  size_t i = std::distance(raw_times_.begin(), hi), left = i, right = i;
  double fraction = 0;
  if (*hi != time) {
    if (i == 0)
      throw std::runtime_error("finite joint IMU left source lost");
    left = i - 1;
    fraction = (time - raw_times_[left]) / (raw_times_[right] - raw_times_[left]);
  }
  auto ensure = [&](size_t at) {
    const double t = raw_times_[at];
    const std::string key = "imu/" + LtvMainCrossShadow::pixelKey(0, 0, t);
    const double dt = at ? raw_times_[at] - raw_times_[at - 1] : raw_times_[1] - raw_times_[0];
    if (!(dt > 0))
      throw std::runtime_error("invalid raw source sampling");
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(6, 6);
    q.topLeftCorner<3, 3>().diagonal().setConstant(sigma_acc_ * sigma_acc_ / dt);
    q.bottomRightCorner<3, 3>().diagonal().setConstant(sigma_gyro_ * sigma_gyro_ / dt);
    if (t < raw_prune_cutoff_ - 1e-9)
      throw std::runtime_error("raw endpoint reused after certified prune");
    if (!source_laws_.count(key))
      source_laws_[key] = q;
    factors_->source(key, source_laws_.at(key));
    source_times_[key] = t;
    return key;
  };
  const std::string l = ensure(left), r = ensure(right);
  return (1 - fraction) * factors_->block(l) + fraction * factors_->block(r);
}
void LtvFiniteJointShadow::mainImu(const Eigen::MatrixXd &f, const Eigen::MatrixXd &b_left, const Eigen::MatrixXd &b_right, double t_left,
                                   double t_right) {
  if (!enabled_)
    return;
  endpoint(t_left);
  endpoint(t_right);
  const auto left = endpoint(t_left), right = endpoint(t_right);
  auto &main = factors_->block("main");
  main.topRows(15) = (f * main.topRows(15) - b_left * left - b_right * right).eval();
}
Eigen::MatrixXd LtvFiniteJointShadow::pixel(const std::string &key, double variance) {
  const std::string name = "pixel/" + key;
  if (std::stod(key.substr(key.rfind('/') + 1)) < pixel_prune_cutoff_ - 1e-9)
    throw std::runtime_error("pixel reused after certified prune");
  if (!LtvMainCrossShadow::instance().registerPixel(key, variance))
    throw std::runtime_error("finite pixel receipt unavailable");
  factors_->source(name, variance * Eigen::Matrix2d::Identity());
  source_times_[name] = std::stod(key.substr(key.rfind('/') + 1));
  return factors_->block(name);
}
void LtvFiniteJointShadow::mainVisual(const Eigen::MatrixXd &h, const Eigen::MatrixXd &k, const Eigen::MatrixXd &r,
                                      const Eigen::MatrixXd &projection) {
  if (!enabled_)
    return;
  auto &ledger = LtvMainCrossShadow::instance();
  Eigen::MatrixXd selected = Eigen::MatrixXd::Zero(r.rows(), r.rows());
  if (projection.rows() != r.rows() || projection.cols() != ledger.sourceDimension()) {
    ++missing_;
  } else
    for (const auto &source : source_times_)
      if (source.first.compare(0, 6, "pixel/") == 0) {
        const int col = ledger.sourceColumn(source.first.substr(6));
        if (col >= 0) {
          const auto u = projection.middleCols(col, 2);
          selected.noalias() += u * factors_->covariance(source.first, source.first) * u.transpose();
        }
      }
  Eigen::MatrixXd independent = (.5 * (r - selected + (r - selected).transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(independent);
  visual_min_eigenvalue_ = es.info() == Eigen::Success ? es.eigenvalues().minCoeff() : -std::numeric_limits<double>::infinity();
  if (es.info() != Eigen::Success || visual_min_eigenvalue_ < -1e-10 * std::max(1e-12, r.norm()))
    throw std::runtime_error("finite visual source remainder not PSD");
  if (visual_min_eigenvalue_ < 0)
    ++visual_roundoff_floors_;
  independent = es.eigenvectors() * es.eigenvalues().cwiseMax(0.).asDiagonal() * es.eigenvectors().transpose();
  const std::string noise = "visual/" + std::to_string(++event_);
  factors_->source(noise, independent);
  Eigen::MatrixXd directions = factors_->block(noise);
  if (projection.rows() == r.rows() && projection.cols() == ledger.sourceDimension())
    for (const auto &source : source_times_)
      if (source.first.compare(0, 6, "pixel/") == 0) {
        const int col = ledger.sourceColumn(source.first.substr(6));
        if (col >= 0)
          directions.noalias() += projection.middleCols(col, 2) * factors_->block(source.first);
      }
  pre_visual_main_ = factors_->block("main");
  const Eigen::MatrixXd f = Eigen::MatrixXd::Identity(h.cols(), h.cols()) - k * h;
  factors_->set("main", (f * factors_->block("main") - k * directions).eval());
  factors_->erase(noise);
}
void LtvFiniteJointShadow::errorReset(const Eigen::VectorXd &dx, const std::vector<int> &orientation_rows) {
  if (!enabled_)
    return;
  auto &post = factors_->block("main");
  if (pre_visual_main_.rows() != post.rows() || pre_visual_main_.cols() != post.cols())
    throw std::runtime_error("finite injection source receipt missing");
  for (int row : orientation_rows) {
    const Eigen::Matrix3d ad = LtvMainCrossShadow::nativeJplProgramAd(dx.segment<3>(row));
    const Eigen::Matrix3d jd = LtvMainCrossShadow::nativeJplReset(dx.segment<3>(row));
    const Eigen::MatrixXd before = pre_visual_main_.middleRows(row, 3), correction = before - post.middleRows(row, 3);
    post.middleRows(row, 3) = ad * before - jd * correction;
  }
  pre_visual_main_.resize(0, 0);
}
void LtvFiniteJointShadow::observerImu(double dt, double t_left, double t_right, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                                       const ov_msckf::LtvCalibration &c) {
  if (!enabled_ || !observer_ || !observer_->started())
    return;
  endpoint(t_left);
  endpoint(t_right);
  const Eigen::MatrixXd raw = .5 * (endpoint(t_left) + endpoint(t_right));
  const Eigen::Matrix3d aa = c.R_ACCtoIMU * c.Da, ww = c.R_GYROtoIMU * c.Dw;
  Eigen::Matrix<double, 6, 6> calibration = Eigen::Matrix<double, 6, 6>::Zero();
  calibration.topLeftCorner<3, 3>() = aa;
  calibration.bottomRightCorner<3, 3>() = ww;
  calibration.bottomLeftCorner<3, 3>() = -ww * c.Tg * aa;
  // Main prior-bias errors are true-est. Estimates use frozen pre-propagation
  // biases. Constant physical-bias scope means corrected perturbation is the
  // SAME map applied to raw noise plus the frozen true-est bias source.
  const Eigen::MatrixXd directions = calibration * (raw + factors_->block("bias_prior"));
  sensitivity_->imuInput(directions);
  observer_->propagateImu(dt, acc, gyro, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  if (!observer_->started()) {
    ++missing_;
    active_ = false;
  }
}
void LtvFiniteJointShadow::camera(const FeaturePipelineContext &ctx, const FeaturePipelineFrame &accepted, const LtvConfig &input,
                                  const ov_msckf::LtvCalibration &cal) {
  if (!enabled_)
    return;
  if (!observer_ || epoch_ != ctx.epoch) {
    for (const auto &a : anchors_)
      factors_->erase(a.second);
    anchors_.clear();
    epoch_ = ctx.epoch;
    active_ = false;
    next_id_ = 0;
    observer_.reset(new LtvObserver(false));
    LtvConfig config = input;
    config.enable = true;
    config.max_features = config.min_features = 1;
    observer_->configure(config);
    observer_->start(ctx.time);
    observer_->enableControlledFeatures(epoch_, true);
    sensitivity_ = std::make_shared<LtvObserverSourceSensitivity>(factors_);
    observer_->attachSourceSensitivity(sensitivity_);
  }
  if (!observer_->started()) {
    ++missing_;
    return;
  }
  nominal_poses_ = Eigen::MatrixXd::Zero(ctx.poses.size() + 1, 10);
  auto nominal = [&](int row, const SeedPose &pose, int orientation, int position) {
    const Eigen::Vector4d q = ov_core::rot_2_quat(pose.R_WB.transpose());
    nominal_poses_(row, 0) = pose.t;
    nominal_poses_(row, 1) = orientation;
    nominal_poses_(row, 2) = position;
    nominal_poses_.block<1, 4>(row, 3) = q.transpose();
    nominal_poses_.block<1, 3>(row, 7) = pose.p_WB.transpose();
  };
  if (ctx.execution_pose_index < ctx.poses.size())
    nominal(0, ctx.poses[ctx.execution_pose_index], 0, 3);
  for (size_t i = 0; i < ctx.poses.size(); ++i) {
    Eigen::Index orientation = 0, position = 0;
    if (ctx.pose_error_selector.rows() == 6 * static_cast<int>(ctx.poses.size())) {
      ctx.pose_error_selector.row(6 * i).maxCoeff(&orientation);
      ctx.pose_error_selector.row(6 * i + 3).maxCoeff(&position);
    } else {
      ++missing_;
    }
    nominal(i + 1, ctx.poses[i], orientation, position);
  }

  if (active_ && std::find(accepted.management.retained_ids.begin(), accepted.management.retained_ids.end(), feature_) ==
                     accepted.management.retained_ids.end()) {
    active_ = false;
    for (const auto &a : anchors_)
      factors_->erase(a.second);
    anchors_.clear();
  }
  LtvControlledFeatures control;
  control.epoch = epoch_;
  control.imu_timestamp = ctx.time;
  if (!active_)
    for (const auto &birth : accepted.management.births) {
      auto d = std::find_if(accepted.seeds.begin(), accepted.seeds.end(),
                            [&](const FeatureSeedDiagnostic &s) { return s.candidate.feature_id == birth.feature_id; });
      if (!birth.apply_seed || d == accepted.seeds.end() || d->main_jacobian.cols() != factors_->block("main").rows()) {
        ++missing_;
        continue;
      }
      bool valid = true;
      for (const auto &o : d->input.observations)
        valid = valid && !o.pixel_source_key.empty() && o.pixel_to_tangent.norm() > 0 &&
                !LtvMainCrossShadow::instance().previouslyUsed(o.pixel_source_key);
      if (!valid) {
        ++missing_;
        continue;
      }
      for (const auto &o : d->input.observations)
        pixel(o.pixel_source_key, ctx.pixel_noise_variance);
      Eigen::MatrixXd seed = -d->main_jacobian * factors_->block("main");
      for (size_t i = 0; i < d->input.observations.size(); ++i) {
        const auto &o = d->input.observations[i];
        seed.noalias() +=
            d->bearing_jacobian.middleCols(2 * i, 2) * o.pixel_to_tangent * pixel(o.pixel_source_key, ctx.pixel_noise_variance);
      }
      sensitivity_->seedInput(seed);
      feature_ = birth.feature_id;
      local_id_ = next_id_++;
      active_ = true;
      LtvLandmarkSeed s;
      s.feature_id = local_id_;
      s.apply_mean = true;
      s.mean_body = birth.seed.landmark_B;
      control.births.push_back(s);
      break;
    }
  std::vector<LtvFeatureObservation> observations;
  if (active_) {
    control.retained_ids = {local_id_};
    for (const auto &o : ctx.observations)
      if (o.feature_id == feature_ && o.camera_id == 0 && o.match_valid) {
        pixel(o.pixel_source_key, ctx.pixel_noise_variance);
        const Eigen::Vector3d unit = o.bearing.normalized();
        const Eigen::MatrixXd d_raw = o.bearing.norm() * LtvSeedEstimator::tangentBasis(unit) * o.pixel_to_tangent *
                                      pixel(o.pixel_source_key, ctx.pixel_noise_variance);
        sensitivity_->cameraInput(o.bearing, d_raw, cal.R_BC, cal.p_BC);
        observations.push_back({local_id_, o.bearing});
        break;
      }
  }
  if (observer_->slotForFeature(local_id_) >= 0)
    record(ctx.time, "before_camera");
  const auto result = observer_->updateFeaturesControlled(ctx.time, ctx.time, observations, cal.R_BC, cal.p_BC, control);
  if (!result.accepted) {
    ++missing_;
    record(ctx.time, "finite_observer_rejected");
    return;
  }
  if (active_) {
    const int slot = observer_->slotForFeature(local_id_);
    const std::string anchor = "anchor/" + LtvMainCrossShadow::pixelKey(feature_, 0, ctx.time);
    factors_->set(anchor, factors_->block("observer_mean").middleRows(3 * slot, 3));
    anchors_[ctx.time] = anchor;
  }
  std::set<double> retained;
  for (const auto &p : ctx.poses)
    retained.insert(p.t);
  for (auto a = anchors_.begin(); a != anchors_.end();)
    if (!retained.count(a->first)) {
      factors_->erase(a->second);
      a = anchors_.erase(a);
    } else
      ++a;
  const double oldest = ctx.poses.empty() ? ctx.time : ctx.poses.front().t;
  for (auto s = source_times_.begin(); s != source_times_.end();)
    if (s->second < (s->first.compare(0, 4, "imu/") == 0 ? ctx.time - .05 : oldest) - 1e-9) {
      factors_->erase(s->first);
      source_laws_.erase(s->first);
      s = source_times_.erase(s);
    } else
      ++s;
  raw_prune_cutoff_ = std::max(raw_prune_cutoff_, ctx.time - .05);
  pixel_prune_cutoff_ = std::max(pixel_prune_cutoff_, oldest);
  LtvMainCrossShadow::instance().prunePixelSources(oldest);
  factors_->compress();
  time_ = ctx.time;
  record(ctx.time, "after_camera");
}
void LtvFiniteJointShadow::record(double time, const char *event) {
  if (!enabled_ || !factors_->contains("observer_mean"))
    return;
  const auto &x = factors_->block("main"), &o = factors_->block("observer_mean");
  output_ << event << ',' << time << ',' << epoch_ << ',' << feature_ << ',' << x.rows() << ',' << o.rows() << ',' << factors_->columns()
          << ',' << x.squaredNorm() << ',' << o.squaredNorm() << ',' << (x * o.transpose()).norm() << ',' << anchors_.size() << ','
          << missing_ << ',' << (stored_.rows() == x.rows() ? (x * x.transpose() - stored_).norm() : -1) << ',' << raw_prune_cutoff_ << ','
          << pixel_prune_cutoff_ << ',' << visual_min_eigenvalue_ << ',' << visual_roundoff_floors_
          << ",FINITE_ONE_POINT_FIXED_CALIB_CONSTANT_TRUE_BIAS_CONDITIONAL_MAIN_GAIN_NOT_REAL_CALIBRATED\n";
  if (std::string(event) != "before_camera")
    return;
  matrix_index_ << "{\"event\":\"" << event << "\",\"time\":" << time << ",\"epoch\":" << epoch_ << ",\"feature\":" << feature_
                << ",\"valid_unconditional_production_model\":false,\"missing_events\":" << missing_ << ",\"matrices\":{";
  bool first = true;
  auto write = [&](const std::string &name, const Eigen::MatrixXd &m) {
    if (!first)
      matrix_index_ << ',';
    first = false;
    const uint64_t offset = matrix_data_.tellp();
    matrix_data_.write(reinterpret_cast<const char *>(m.data()), 8 * m.size());
    matrix_index_ << '"' << name << "\":{\"offset\":" << offset << ",\"rows\":" << m.rows() << ",\"cols\":" << m.cols()
                  << ",\"encoding\":\"native_float64_column_major\"}";
  };
  write("nominal_poses_time_orientation_row_position_row_qGtoI_xyzw_pWorld", nominal_poses_);
  write("P_stored_main", stored_);
  write("Sigma_main_local_program", x * x.transpose());
  write("Sigma_observer_mean", o * o.transpose());
  write("C_main_local_program_observer", x * o.transpose());
  const int slot = observer_->slotForFeature(local_id_);
  if (slot >= 0)
    for (const auto &a : anchors_) {
      const Eigen::MatrixXd &anchor = factors_->block(a.second);
      const auto suffix = std::to_string(a.first);
      write("Sigma_anchor_" + suffix, anchor * anchor.transpose());
      write("Sigma_current_anchor_" + suffix, o.middleRows(3 * slot, 3) * anchor.transpose());
      write("C_main_local_program_anchor_" + suffix, x * anchor.transpose());
    }
  matrix_index_ << "}}\n";
  output_.flush();
  matrix_index_.flush();
  matrix_data_.flush();
}
} // namespace ltv
