// Input-only ABI for the actual deployed LtvAdapter. No truth/phase arguments.
#include "ltv/LtvAdapter.h"
#include "ltv/LtvBoundedDiagnostics.h"
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
using V3 = Eigen::Vector3d;
using RM3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
namespace {
ov_msckf::LtvOptions options(int hardened, int source, double sigma) {
  ov_msckf::LtvOptions o;
  o.enabled = o.observer.enable = o.feature_readiness_enabled = true;
  o.passive_hardening_enabled = hardened;
  o.hardening_initial_warmup = hardened >= 2;
  o.hardening_preserve_constrained_state = hardened >= 3;
  o.hardening_ready_soft_grace = hardened >= 4;
  o.active_consistency.enabled = hardened == 5;
  o.active_consistency.max_history_residual_rad = .02;
  o.active_consistency.max_holdout_residual_rad = .02;
  o.feature_seed_source = source == 0 ? "TEMPORAL_POSE" : (source == 1 ? "STEREO" : "STEREO_THEN_TEMPORAL");
  o.feature_bearing_sigma_rad = sigma;
  return o;
}
struct Harness {
  ov_msckf::LtvAdapter adapter;
  std::string error, json = "{}";
  ov_msckf::LtvFrame frame;
  uint64_t receipt = 0;
  bool hardened;
  bool diagnostics = true;
  Harness(int h, int s, double sigma) : adapter(options(h, s, sigma)), hardened(h) {}
};
void number(std::ostream &out, double value) {
  if (std::isfinite(value))
    out << value;
  else
    out << "null";
}
void vector(std::ostream &out, const V3 &v) {
  out << '[';
  for (int j = 0; j < 3; ++j) {
    if (j)
      out << ',';
    number(out, v(j));
  }
  out << ']';
}
template <class T> void ids(std::ostream &out, const T &list) {
  out << '[';
  bool first = true;
  for (auto id : list) {
    if (!first)
      out << ',';
    first = false;
    out << id;
  }
  out << ']';
}
} // namespace
extern "C" {
void *ph_create(int hardened, int source, double sigma) {
  try {
    if (hardened < 0 || hardened > 5 || source < 0 || source > 2)
      return nullptr;
    return new Harness(hardened, source, sigma);
  } catch (...) {
    return nullptr;
  }
}
void ph_destroy(void *ptr) { delete static_cast<Harness *>(ptr); }
void ph_set_diagnostics(void *ptr, int enabled) { static_cast<Harness *>(ptr)->diagnostics = enabled != 0; }
const char *ph_error(void *ptr) { return static_cast<Harness *>(ptr)->error.c_str(); }
const char *ph_json(void *ptr) { return static_cast<Harness *>(ptr)->json.c_str(); }
int ph_imu(void *ptr, double time, const double *measurement) {
  auto &h = *static_cast<Harness *>(ptr);
  try {
    ov_core::ImuData s;
    s.timestamp = time;
    s.am = Eigen::Map<const V3>(measurement);
    s.wm = Eigen::Map<const V3>(measurement + 3);
    h.adapter.feed_imu(s);
    return 1;
  } catch (const std::exception &e) {
    h.error = e.what();
    return 0;
  }
}
int ph_frame(void *ptr, int64_t camera_ns, double time, int count, const int64_t *feature_ids, const int *cameras, const int64_t *actual_ns,
             const double *bearings, int poses, const double *pose_times, const double *rotations, const double *positions,
             const double *covariance, const double *Rbc, const double *pc) {
  auto &h = *static_cast<Harness *>(ptr);
  try {
    if (poses < 1 || count < 0 || !std::isfinite(time) || std::llround(time * 1e9) != camera_ns)
      throw std::invalid_argument("invalid camera input");
    ++h.receipt;
    ltv::FeaturePipelineContext ctx;
    ctx.time = time;
    ctx.version = h.receipt;
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
      c.R_BC = Eigen::Map<const RM3>(Rbc + 9 * j);
      c.p_BC = Eigen::Map<const V3>(pc + 3 * j);
      ctx.cameras.push_back(c);
    }
    ctx.pose_covariance =
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(covariance, 6 * poses, 6 * poses);
    std::vector<ov_msckf::LtvBearing> left;
    std::set<size_t> left_ids, right_ids;
    std::vector<int> rejected;
    for (int j = 0; j < count; ++j) {
      if (feature_ids[j] < 0 || cameras[j] < 0 || cameras[j] > 1)
        throw std::invalid_argument("invalid feature/camera ID");
      // Synchrony is an input fact, not inferred from the scene or GT.
      if (actual_ns[j] != camera_ns) {
        rejected.push_back(j);
        continue;
      }
      const auto id = static_cast<size_t>(feature_ids[j]);
      const V3 z = Eigen::Map<const V3>(bearings + 3 * j);
      ctx.observations.push_back({id, cameras[j], z, true});
      if (cameras[j] == 0) {
        left.push_back({id, z});
        left_ids.insert(id);
      } else
        right_ids.insert(id);
    }
    ov_msckf::LtvCalibration calibration;
    calibration.R_BC = ctx.cameras[0].R_BC;
    calibration.p_BC = ctx.cameras[0].p_BC;
    h.frame = h.adapter.process(time, left, calibration, V3::Zero(), V3::Zero(), h.receipt, &ctx);
    if (h.frame.available && !h.adapter.claim(h.frame))
      throw std::runtime_error("frame claim failed");
    const auto &f = h.frame;
    const auto &pf = h.adapter.feature_frame();
    const auto &m = pf.management;
    const bool raw = h.hardened ? f.raw_current : (f.available && f.imu_time == time);
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\"receipt_id\":" << h.receipt << ",\"camera_ns\":" << camera_ns << ",\"t\":" << time << ",\"epoch\":" << f.epoch
        << ",\"sequence\":" << f.sequence << ",\"imu_time\":" << f.imu_time << ",\"cursor\":" << f.cursor
        << ",\"available\":" << f.available << ",\"raw_current\":" << raw << ",\"ready_G\":" << f.ready_G << ",\"ready_V\":" << f.ready_V
        << ",\"health_state\":\"" << ltv::toString(f.health.state) << "\",\"health_reason\":\"" << f.health.reason << "\",\"reason\":\""
        << f.reason << "\",\"pool_ready\":" << f.health.pool_ready << ",\"observer_valid\":" << f.health.observer_valid
        << ",\"last_bootstrap_time\":" << f.health.last_bootstrap_time << ",\"bootstrap_count\":" << f.health.bootstrap_count
        << ",\"bootstrap_source\":\"" << f.health.last_bootstrap_source << "\",\"physical_fault_count\":" << f.health.physical_fault_count
        << ",\"actual_corrections\":" << f.actual_corrections << ",\"compute_time_ms\":" << f.compute_time_ms
        << ",\"heavy_diagnostics\":" << h.diagnostics << ",\"correction_diagnostics_valid\":" << f.correction_diagnostics_valid
        << ",\"prediction_angle_p95_rad\":";
    number(out, f.prediction_angle_p95_rad);
    out << ",\"velocity_correction_rate\":";
    number(out, f.velocity_correction_rate);
    out << ",\"gravity_correction_rate\":";
    number(out, f.gravity_correction_rate);
    out << ",\"state_features\":" << f.snapshot.state_features << ",\"observed_features\":" << f.snapshot.observed_features
        << ",\"mature_features\":" << f.mature_features << ",\"camera_substeps\":" << f.snapshot.camera_substeps << ",\"corrected_ids\":";
    std::set<size_t> corrected_ids;
    if (f.available && raw && f.snapshot.camera_substeps > 0)
      for (const auto &observation : m.observations)
        if (observation.camera_id == 0)
          corrected_ids.insert(observation.feature_id);
    ids(out, corrected_ids);
    out << ",\"ready_soft_grace_enabled\":" << f.health.ready_soft_grace_enabled << ",\"grace_G\":" << f.health.grace_G
        << ",\"grace_V\":" << f.health.grace_V << ",\"soft_failure_frames_G\":" << f.health.soft_failure_frames_G
        << ",\"soft_failure_frames_V\":" << f.health.soft_failure_frames_V << ",\"last_strict_good_G\":" << f.health.last_strict_good_G
        << ",\"last_strict_good_V\":" << f.health.last_strict_good_V;
    out << ",\"raw_v\":";
    if (raw)
      vector(out, f.snapshot.velocity_body);
    else
      out << "null";
    out << ",\"raw_eta\":";
    if (raw)
      vector(out, f.snapshot.gravity_body);
    else
      out << "null";
    out << ",\"snapshot_v\":";
    vector(out, f.snapshot.velocity_body);
    out << ",\"snapshot_eta\":";
    vector(out, f.snapshot.gravity_body);
    out << ",\"current_cam0_ids\":";
    ids(out, left_ids);
    out << ",\"current_cam1_ids\":";
    ids(out, right_ids);
    out << ",\"rejected_async_observation_indices\":";
    ids(out, rejected);
    out << ",\"management_accepted_input\":" << m.accepted_input << ",\"management_reason\":\"" << m.reason
        << "\",\"opportunity_tracks\":" << m.opportunity_tracks << ",\"admitted_tracks\":" << m.admitted_tracks << ",\"retained_ids\":";
    ids(out, m.retained_ids);
    out << ",\"retired_ids\":";
    ids(out, m.retired_ids);
    out << ",\"seeds\":[";
    bool first = true;
    if (h.diagnostics)
      for (const auto &d : pf.seeds) {
        if (!first)
          out << ',';
        first = false;
        const auto &s = d.candidate;
        bool admitted = false, written = false;
        for (const auto &b : m.births)
          if (b.feature_id == s.feature_id) {
            admitted = true;
            written = b.apply_seed;
          }
        out << "{\"id\":" << s.feature_id << ",\"time\":" << s.time << ",\"source\":\"" << ltv::toString(s.source)
            << "\",\"geometry_valid\":" << s.geometry_valid << ",\"admitted\":" << admitted << ",\"mean_written\":" << written
            << ",\"reason\":\"" << d.estimate.reason << "\",\"landmark_B\":";
        vector(out, s.landmark_B);
        out << ",\"relative_risk\":";
        number(out, s.relative_risk);
        out << ",\"sigma_parallel\":";
        number(out, d.uncertainty.sigma_parallel);
        out << ",\"condition\":";
        number(out, s.condition);
        out << ",\"min_ray\":";
        number(out, s.min_ray);
        out << ",\"parallax_rad\":";
        number(out, s.parallax_rad);
        out << ",\"residual_rad\":";
        number(out, s.residual_rad);
        out << ",\"heldout_check_available\":" << d.heldout_check_available << ",\"heldout_max_residual_rad\":";
        number(out, d.heldout_max_residual_rad);
        out << '}';
      }
    out << "],\"tracks\":[";
    first = true;
    if (h.diagnostics && h.adapter.feature_pipeline())
      for (const auto &entry : h.adapter.feature_pipeline()->manager().landmarks()) {
        if (!first)
          out << ',';
        first = false;
        const auto &tr = entry.second;
        out << "{\"id\":" << entry.first << ",\"phase\":\"" << ltv::toString(tr.phase) << "\",\"reason\":\"" << tr.reason
            << "\",\"first_seen\":" << tr.first_seen << ",\"entered\":" << tr.entered << ",\"seeded\":" << tr.seeded
            << ",\"last_seen\":" << tr.last_seen << ",\"ever_opportunity\":" << tr.ever_opportunity
            << ",\"seed_written\":" << tr.seed_written << '}';
      }
    if (h.hardened && h.diagnostics)
      for (const auto &event : m.retirement_events) {
        if (!first)
          out << ',';
        first = false;
        ltv::writeBoundedLandmarkRecord(out, event.feature_id, event.record);
      }
    out << ']';
    out << ",\"births\":[";
    first = true;
    for (const auto &birth : m.births) {
      if (!first)
        out << ',';
      first = false;
      out << "{\"id\":" << birth.feature_id << ",\"apply_seed\":" << birth.apply_seed << ",\"source\":\""
          << ltv::toString(birth.seed.source) << "\"}";
    }
    out << ']';
    if (h.hardened)
      ltv::writeBoundedManagement(out, m);
    {
      size_t manager_records = 0, history_records = 0, max_samples = 0, total_samples = 0;
      if (h.adapter.feature_pipeline()) {
        const auto &manager = h.adapter.feature_pipeline()->manager();
        manager_records = manager.landmarks().size();
        history_records = manager.history().tracks().size();
        for (const auto &entry : manager.history().tracks()) {
          max_samples = std::max(max_samples, entry.second.samples.size());
          total_samples += entry.second.samples.size();
        }
      }
      out << ",\"resource_diagnostics\":{\"manager_records\":" << manager_records << ",\"history_records\":" << history_records
          << ",\"max_samples_per_track\":" << max_samples << ",\"total_history_samples\":" << total_samples
          << ",\"adapter_map_size\":" << h.adapter.feature_ids().size()
          << ",\"core_admission_history_size\":" << h.adapter.core().admissionHistorySize()
          << ",\"core_admission_high_water\":" << h.adapter.core().admissionHighWaterMark()
          << ",\"imu_buffer_size\":" << h.adapter.bufferedImuSamples() << ",\"retained_ids\":" << m.retained_ids.size()
          << ",\"guard_capacity_bytes\":" << (h.hardened ? ltv::LtvIdentityGuard::capacityBytes() : 0) << '}';
    }
    out << '}';
    h.json = out.str();
    return 1;
  } catch (const std::exception &e) {
    h.error = e.what();
    return 0;
  }
}
int ph_read(void *ptr, double *state, double *covariance, int64_t *ids_out, int copy_P) {
  auto &h = *static_cast<Harness *>(ptr);
  const auto &core = h.adapter.core();
  const int d = core.state().size();
  if (d > 96 || d < 0)
    return -1;
  if (d)
    std::copy(core.state().data(), core.state().data() + d, state);
  if (copy_P)
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(covariance, d, d) = core.covariance();
  for (int j = 0; j < 30; ++j)
    ids_out[j] = -1;
  for (const auto &entry : h.adapter.feature_ids()) {
    int slot = core.slotForFeature(entry.second);
    if (slot >= 0 && slot < 30)
      ids_out[slot] = entry.first;
  }
  return d;
}
}
