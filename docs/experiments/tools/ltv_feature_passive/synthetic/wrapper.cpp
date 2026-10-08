// Synthetic harness ABI. All estimation executes deployed C++ modules.
#include "ltv/landmark_adapter/LtvFeaturePipeline.h"
#include "ltv/observer/ltv_observer.h"
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
using V3 = Eigen::Vector3d;
using RM3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
namespace {
struct Harness {
  int mode;
  ltv::LtvObserver observer;
  std::unique_ptr<ltv::LtvFeaturePipeline> pipeline;
  std::set<int> seen;
  std::string frame_json = "{}", error;
  ltv::LandmarkManagerFrame management;
  bool ready = false;
  Harness(int m, double risk, double sigma) : mode(m) {
    ltv::LtvConfig c;
    c.enable = true;
    observer.configure(c);
    observer.start(0);
    if (mode) {
      ltv::FeaturePipelineConfig p;
      p.manager.apply_seed = (mode != 1);
      p.manager.quality.max_relative_risk = risk;
      p.bearing_sigma_rad = sigma;
      p.source = mode == 4 ? ltv::FeatureSeedSource::StereoThenTemporal
                           : (mode == 3 ? ltv::FeatureSeedSource::Stereo : ltv::FeatureSeedSource::TemporalPose);
      pipeline.reset(new ltv::LtvFeaturePipeline(p));
      pipeline->reset(1);
      if (!observer.enableControlledFeatures(1))
        throw std::runtime_error("controlled startup");
    }
  }
};
std::string json_finite(double x) {
  if (!std::isfinite(x))
    return "null";
  std::ostringstream text;
  text << std::setprecision(17) << x;
  return text.str();
}
void vector_json(std::ostream &s, const V3 &v) {
  s << "[" << json_finite(v.x()) << "," << json_finite(v.y()) << "," << json_finite(v.z()) << "]";
}
} // namespace
extern "C" {
void *fp_create(int mode, double max_risk, double bearing_sigma) {
  try {
    return new Harness(mode, max_risk, bearing_sigma);
  } catch (...) {
    return nullptr;
  }
}
void fp_destroy(void *ptr) { delete static_cast<Harness *>(ptr); }
const char *fp_error(void *ptr) { return static_cast<Harness *>(ptr)->error.c_str(); }
int fp_predict(void *ptr, double dt, const double *a, const double *w) {
  auto &h = *static_cast<Harness *>(ptr);
  h.observer.propagateImu(dt, Eigen::Map<const V3>(a), Eigen::Map<const V3>(w), V3::Zero(), V3::Zero());
  return h.observer.started();
}
int fp_frame(void *ptr, double t, int frame_index, int count, const int *ids, const double *left, const double *right, int poses,
             const double *pose_times, const double *rotations, const double *positions, const double *pose_covariance,
             const double *camera_rotations, const double *camera_positions) {
  auto &h = *static_cast<Harness *>(ptr);
  try {
    ltv::FeaturePipelineContext ctx;
    ctx.epoch = 1;
    ctx.version = frame_index + 1;
    ctx.time = t;
    ctx.execution_pose_index = poses - 1;
    for (int j = 0; j < poses; ++j) {
      ltv::SeedPose p;
      p.t = pose_times[j];
      p.R_WB = Eigen::Map<const RM3>(rotations + 9 * j);
      p.p_WB = Eigen::Map<const V3>(positions + 3 * j);
      ctx.poses.push_back(p);
    }
    for (int j = 0; j < 2; ++j) {
      ltv::SeedCamera c;
      c.R_BC = Eigen::Map<const RM3>(camera_rotations + 9 * j);
      c.p_BC = Eigen::Map<const V3>(camera_positions + 3 * j);
      ctx.cameras.push_back(c);
    }
    ctx.pose_covariance =
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(pose_covariance, 6 * poses, 6 * poses);
    std::vector<ltv::LtvFeatureObservation> obs;
    for (int j = 0; j < count; ++j) {
      h.seen.insert(ids[j]);
      ctx.observations.push_back({static_cast<size_t>(ids[j]), 0, Eigen::Map<const V3>(left + 3 * j), true});
      ctx.observations.push_back({static_cast<size_t>(ids[j]), 1, Eigen::Map<const V3>(right + 3 * j), true});
      obs.push_back({ids[j], Eigen::Map<const V3>(left + 3 * j)});
    }
    std::ostringstream log;
    log << std::setprecision(17) << "{\"t\":" << t << ",\"frame\":" << frame_index << ",\"candidates\":[";
    ltv::LtvSnapshot snapshot;
    if (!h.mode) {
      snapshot = h.observer.updateFeatures(t, t, obs, ctx.cameras[0].R_BC, ctx.cameras[0].p_BC);
      log << "],\"births\":[],\"retired\":[],\"tracks\":[]";
    } else {
      auto frame = h.pipeline->process(ctx);
      h.management = frame.management;
      if (!frame.management.accepted_input)
        throw std::runtime_error("pipeline:" + frame.management.reason);
      for (size_t j = 0; j < frame.seeds.size(); ++j) {
        const auto &d = frame.seeds[j];
        if (j)
          log << ",";
        log << "{\"id\":" << d.candidate.feature_id << ",\"valid\":" << (d.candidate.geometry_valid ? "true" : "false") << ",\"reason\":\""
            << d.estimate.reason << "\",\"seed_B\":";
        vector_json(log, d.estimate.landmark_B);
        log << ",\"risk\":" << json_finite(d.candidate.relative_risk) << ",\"sigma_parallel\":" << json_finite(d.uncertainty.sigma_parallel)
            << ",\"parallax\":" << json_finite(d.candidate.parallax_rad) << ",\"condition\":" << json_finite(d.candidate.condition)
            << ",\"residual\":" << json_finite(d.candidate.residual_rad) << ",\"min_ray\":" << json_finite(d.candidate.min_ray) << "}";
      }
      log << "],\"births\":[";
      ltv::LtvControlledFeatures control;
      control.epoch = 1;
      control.imu_timestamp = t;
      for (auto id : frame.management.retained_ids)
        control.retained_ids.push_back(id);
      for (size_t j = 0; j < frame.management.births.size(); ++j) {
        const auto &b = frame.management.births[j];
        control.births.push_back({static_cast<int>(b.feature_id), b.apply_seed, b.seed.landmark_B});
        if (j)
          log << ",";
        log << "{\"id\":" << b.feature_id << ",\"apply_seed\":" << (b.apply_seed ? "true" : "false") << ",\"seed_B\":";
        vector_json(log, b.seed.landmark_B);
        log << ",\"risk\":" << json_finite(b.seed.relative_risk) << ",\"source\":\"" << ltv::toString(b.seed.source) << "\"}";
      }
      log << "],\"retired\":[";
      for (size_t j = 0; j < frame.management.retired_ids.size(); ++j) {
        if (j)
          log << ",";
        log << frame.management.retired_ids[j];
      }
      log << "],\"tracks\":[";
      for (int j = 0; j < count; ++j) {
        if (j)
          log << ",";
        auto it = h.pipeline->manager().landmarks().find(ids[j]);
        log << "{\"id\":" << ids[j];
        if (it != h.pipeline->manager().landmarks().end()) {
          const auto &track = it->second;
          log << ",\"phase\":\"" << ltv::toString(track.phase) << "\",\"opportunity\":" << (track.ever_opportunity ? "true" : "false")
              << ",\"first_seen\":" << track.first_seen << ",\"entered\":" << track.entered << ",\"seeded\":" << track.seeded
              << ",\"reason\":\"" << track.reason << "\"";
        }
        log << "}";
      }
      log << "]";
      obs.clear();
      for (const auto &o : frame.management.observations)
        obs.push_back({static_cast<int>(o.feature_id), o.bearing});
      const auto result = h.observer.updateFeaturesControlled(t, t, obs, ctx.cameras[0].R_BC, ctx.cameras[0].p_BC, control);
      if (!result.accepted)
        throw std::runtime_error("core:" + result.reason);
      snapshot = result.snapshot;
    }
    h.ready = snapshot.valid && (!h.mode || h.management.enough_mature);
    log << ",\"ready_G\":" << (h.ready && snapshot.gravity_valid ? "true" : "false")
        << ",\"ready_V\":" << (h.ready && snapshot.velocity_valid ? "true" : "false") << ",\"state_features\":" << snapshot.state_features
        << ",\"observed_features\":" << snapshot.observed_features << ",\"mature_visible\":" << (h.mode ? h.management.mature_visible : 0)
        << ",\"camera_substeps\":" << snapshot.camera_substeps << ",\"innovation\":" << json_finite(snapshot.innovation_norm)
        << ",\"reset_reason\":\"" << ltv::toString(snapshot.last_reset_reason) << "\",\"injection_G\":0,\"injection_V\":0}";
    h.frame_json = log.str();
    return h.observer.started();
  } catch (const std::exception &e) {
    h.error = e.what();
    return 0;
  }
}
const char *fp_frame_json(void *ptr) { return static_cast<Harness *>(ptr)->frame_json.c_str(); }
int fp_read(void *ptr, double *state, double *covariance, int *ids, int copy_covariance) {
  const auto &h = *static_cast<Harness *>(ptr);
  const int d = h.observer.state().size();
  std::copy(h.observer.state().data(), h.observer.state().data() + d, state);
  if (copy_covariance)
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(covariance, d, d) = h.observer.covariance();
  for (int j = 0; j < (d - 6) / 3; ++j)
    ids[j] = -1;
  for (int id : h.seen) {
    int slot = h.observer.slotForFeature(id);
    if (slot >= 0)
      ids[slot] = id;
  }
  return d;
}
}

// Independent ABI mapping check; no observer mutation or synthetic labels.
extern "C" int fp_echo(int count, const int *ids, const double *left, const double *right, int poses, const double *pose_times,
                       const double *rotations, const double *positions, const double *pose_covariance, const double *camera_rotations,
                       const double *camera_positions, double *output) {
  int at = 0;
  for (int j = 0; j < count; ++j)
    output[at++] = ids[j];
  for (const double *bearings : {left, right})
    for (int j = 0; j < count; ++j) {
      V3 v = Eigen::Map<const V3>(bearings + 3 * j);
      for (int k = 0; k < 3; ++k)
        output[at++] = v(k);
    }
  for (int j = 0; j < poses; ++j)
    output[at++] = pose_times[j];
  for (int j = 0; j < poses; ++j) {
    RM3 r = Eigen::Map<const RM3>(rotations + 9 * j);
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b)
        output[at++] = r(a, b);
  }
  for (int j = 0; j < poses; ++j) {
    V3 v = Eigen::Map<const V3>(positions + 3 * j);
    for (int k = 0; k < 3; ++k)
      output[at++] = v(k);
  }
  Eigen::MatrixXd covariance =
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(pose_covariance, 6 * poses, 6 * poses);
  for (int a = 0; a < 6 * poses; ++a)
    for (int b = 0; b < 6 * poses; ++b)
      output[at++] = covariance(a, b);
  for (int j = 0; j < 2; ++j) {
    RM3 r = Eigen::Map<const RM3>(camera_rotations + 9 * j);
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b)
        output[at++] = r(a, b);
  }
  for (int j = 0; j < 2; ++j) {
    V3 v = Eigen::Map<const V3>(camera_positions + 3 * j);
    for (int k = 0; k < 3; ++k)
      output[at++] = v(k);
  }
  return at;
}
