#include "ltv/diagnostics/LtvPassiveCache.h"
#include <cstring>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
namespace ov_msckf {
namespace {
const char MAGIC[] = "LTVPCACHE001";
constexpr uint64_t MAX_BYTES = 256ULL * 1024 * 1024;
struct Binary {
  std::istream *in = nullptr;
  std::ostream *out = nullptr;
  explicit Binary(std::istream &s) : in(&s) {}
  explicit Binary(std::ostream &s) : out(&s) {}
  template <typename T> void scalar(T &v) {
    static_assert(std::is_arithmetic<T>::value, "scalar arithmetic only");
    if (in)
      in->read(reinterpret_cast<char *>(&v), sizeof(v));
    else
      out->write(reinterpret_cast<const char *>(&v), sizeof(v));
    if ((in && !*in) || (out && !*out))
      throw std::runtime_error("cache I/O error");
  }
  void count(size_t &v, uint64_t limit = 1000000) {
    uint64_t n = v;
    scalar(n);
    if (n > limit)
      throw std::runtime_error("cache dimension limit");
    v = static_cast<size_t>(n);
  }
  void text(std::string &s) {
    size_t n = s.size();
    count(n, MAX_BYTES);
    if (in) {
      s.resize(n);
      if (n)
        in->read(&s[0], n);
    } else if (n)
      out->write(s.data(), n);
    if ((in && !*in) || (out && !*out))
      throw std::runtime_error("cache string I/O error");
  }
  template <typename Derived> void matrix(Eigen::MatrixBase<Derived> &base) {
    auto &m = base.derived();
    size_t rows = m.rows(), cols = m.cols();
    count(rows, 10000);
    count(cols, 10000);
    if (rows * cols > MAX_BYTES / sizeof(double))
      throw std::runtime_error("cache matrix too large");
    if ((Derived::RowsAtCompileTime != Eigen::Dynamic && rows != static_cast<size_t>(Derived::RowsAtCompileTime)) ||
        (Derived::ColsAtCompileTime != Eigen::Dynamic && cols != static_cast<size_t>(Derived::ColsAtCompileTime)))
      throw std::runtime_error("cache matrix shape mismatch");
    if (in)
      m.resize(rows, cols);
    for (size_t j = 0; j < cols; ++j)
      for (size_t i = 0; i < rows; ++i)
        scalar(m(i, j));
  }
};
void platform() {
  uint32_t one = 1;
  if (sizeof(double) != 8 || *reinterpret_cast<char *>(&one) != 1)
    throw std::runtime_error("cache requires little endian IEEE754 host");
}
void sample(Binary &b, ov_core::ImuData &s) {
  b.scalar(s.timestamp);
  b.matrix(s.wm);
  b.matrix(s.am);
}
void observations(Binary &b, std::vector<ltv::HistoryObservation> &os) {
  size_t n = os.size();
  b.count(n);
  if (b.in)
    os.resize(n);
  for (auto &o : os) {
    b.count(o.feature_id, UINT64_MAX);
    b.scalar(o.camera_id);
    b.matrix(o.bearing);
    b.scalar(o.match_valid);
  }
}
void context(Binary &b, ltv::FeaturePipelineContext &c) {
  b.scalar(c.epoch);
  b.scalar(c.version);
  b.scalar(c.time);
  b.count(c.execution_pose_index, 10000);
  size_t n = c.poses.size();
  b.count(n, 1000);
  if (b.in)
    c.poses.resize(n);
  for (auto &p : c.poses) {
    b.matrix(p.R_WB);
    b.matrix(p.p_WB);
    b.scalar(p.t);
  }
  b.matrix(c.pose_covariance);
  n = c.cameras.size();
  b.count(n, 16);
  if (b.in)
    c.cameras.resize(n);
  for (auto &cam : c.cameras) {
    b.matrix(cam.R_BC);
    b.matrix(cam.p_BC);
  }
  observations(b, c.observations);
}
void calibration(Binary &b, LtvCalibration &c) {
  b.matrix(c.Da);
  b.matrix(c.Dw);
  b.matrix(c.Tg);
  b.matrix(c.R_ACCtoIMU);
  b.matrix(c.R_GYROtoIMU);
  b.matrix(c.R_BC);
  b.matrix(c.p_BC);
  b.scalar(c.offset);
}
void seed(Binary &b, ltv::FeatureSeedCandidate &s) {
  b.count(s.feature_id, UINT64_MAX);
  b.scalar(s.epoch);
  b.scalar(s.time);
  b.matrix(s.landmark_B);
  int source = static_cast<int>(s.source);
  b.scalar(source);
  b.scalar(s.geometry_valid);
  b.scalar(s.independent_check_available);
  b.scalar(s.min_ray);
  b.scalar(s.condition);
  b.scalar(s.parallax_rad);
  b.scalar(s.residual_rad);
  b.scalar(s.relative_risk);
  b.scalar(s.check_residual_rad);
}
std::string evidence(const LtvFrame &original, const LtvAdapter &adapter) {
  std::ostringstream out(std::ios::binary);
  Binary b(static_cast<std::ostream &>(out));
  auto f = original;
#define FIELD(x) b.scalar(f.x)
  FIELD(epoch);
  FIELD(version);
  FIELD(sequence);
  FIELD(camera_ns);
  FIELD(camera_time);
  FIELD(imu_time);
  FIELD(cursor);
  FIELD(available);
  FIELD(ready_G);
  FIELD(ready_V);
  b.count(f.mature_features);
  b.text(f.reason);
  FIELD(integrated_steps);
  FIELD(integrated_seconds);
#undef FIELD
  auto &s = f.snapshot;
#define FIELD(x) b.scalar(s.x)
  FIELD(frame_timestamp);
  FIELD(imu_timestamp);
  b.matrix(s.velocity_body);
  b.matrix(s.gravity_body);
  FIELD(valid);
  FIELD(velocity_valid);
  FIELD(gravity_valid);
  FIELD(state_features);
  FIELD(observed_features);
  FIELD(healthy_camera_updates);
  FIELD(innovation_norm);
  FIELD(covariance_trace);
  FIELD(covariance_min_diagonal);
  FIELD(covariance_max_diagonal);
  FIELD(camera_substeps);
  int reset = static_cast<int>(s.last_reset_reason);
  b.scalar(reset);
  FIELD(snapshot_time_error);
  FIELD(gravity_angle_ltv_vs_vins);
  FIELD(gravity_factor_residual_norm);
  FIELD(gravity_factor_weighted_residual_norm);
  FIELD(gravity_factor_added);
  FIELD(gravity_gate_base_eligible);
  FIELD(gravity_gate_feature_ok);
  FIELD(gravity_gate_eta_norm_ok);
  FIELD(gravity_gate_innovation_ok);
  FIELD(gravity_gate_reset_ok);
  FIELD(gravity_gate_pass);
  FIELD(gravity_gate_reason_mask);
  b.matrix(s.vins_velocity_body);
  b.matrix(s.velocity_factor_residual);
  FIELD(velocity_factor_residual_norm);
  FIELD(velocity_factor_weighted_residual_norm);
  FIELD(velocity_factor_added);
  FIELD(velocity_factor_base_eligible);
  FIELD(velocity_gate_feature_ok);
  FIELD(velocity_gate_innovation_ok);
  FIELD(velocity_gate_disagreement_ok);
  FIELD(velocity_gate_reset_ok);
  FIELD(velocity_gate_pass);
  FIELD(velocity_gate_reason_mask);
  FIELD(velocity_oracle_mask_loaded);
  FIELD(velocity_oracle_mask_hit);
  FIELD(velocity_oracle_pass);
#undef FIELD
  Eigen::VectorXd x = adapter.core().state();
  Eigen::MatrixXd P = adapter.core().covariance();
  b.matrix(x);
  b.matrix(P);
  auto ids = adapter.feature_ids();
  size_t n = ids.size();
  b.count(n);
  for (const auto &entry : ids) {
    size_t id = entry.first;
    b.count(id, UINT64_MAX);
    int core_id = entry.second;
    b.scalar(core_id);
    int slot = adapter.core().slotForFeature(core_id);
    b.scalar(slot);
  }
  auto management = adapter.feature_frame().management;
  b.scalar(management.accepted_input);
  b.text(management.reason);
  b.scalar(management.epoch);
  b.scalar(management.time);
  for (auto *v : {&management.retained_ids, &management.retired_ids}) {
    n = v->size();
    b.count(n);
    for (auto &id : *v)
      b.count(id, UINT64_MAX);
  }
  observations(b, management.observations);
  n = management.births.size();
  b.count(n);
  for (auto &birth : management.births) {
    b.count(birth.feature_id, UINT64_MAX);
    b.scalar(birth.apply_seed);
    seed(b, birth.seed);
  }
  b.count(management.mature_visible);
  b.count(management.opportunity_tracks);
  b.count(management.admitted_tracks);
  b.scalar(management.enough_mature);
  // Include rejected seed diagnostics, so matching output alone cannot hide a cache
  // path that selected different inputs or discarded different candidates.
  auto diagnostics = adapter.feature_frame().seeds;
  n = diagnostics.size();
  b.count(n);
  for (auto &d : diagnostics) {
    seed(b, d.candidate);
    b.scalar(d.heldout_check_available);
    b.count(d.heldout_check_observations);
    b.scalar(d.heldout_max_residual_rad);
    b.text(d.estimate.reason);
    b.scalar(d.estimate.valid);
    b.matrix(d.estimate.point_W);
    b.matrix(d.estimate.landmark_B);
    b.scalar(d.uncertainty.valid);
    b.text(d.uncertainty.reason);
    b.matrix(d.uncertainty.covariance_B);
    b.matrix(d.uncertainty.jacobian);
    b.matrix(d.input_covariance);
    b.scalar(d.uncertainty.sigma_parallel);
    b.scalar(d.uncertainty.relative_parallel_risk);
  }
  return out.str();
}
void same(const std::string &expected, const LtvFrame &f, const LtvAdapter &adapter, uint64_t event) {
  const auto actual = evidence(f, adapter);
  if (expected != actual) {
    size_t at = 0;
    while (at < std::min(expected.size(), actual.size()) && expected[at] == actual[at])
      ++at;
    throw std::runtime_error("cache exact mismatch event=" + std::to_string(event) + " byte=" + std::to_string(at));
  }
}
} // namespace
static void number(std::ostream &out, double x) {
  if (std::isfinite(x))
    out << x;
  else
    out << "null";
}
static void vector(std::ostream &out, const Eigen::Vector3d &x) {
  out << "[";
  for (int k = 0; k < 3; ++k) {
    if (k)
      out << ",";
    number(out, x(k));
  }
  out << "]";
}
static void id_array(std::ostream &out, const std::vector<size_t> &ids) {
  out << "[";
  for (size_t k = 0; k < ids.size(); ++k) {
    if (k)
      out << ",";
    out << ids[k];
  }
  out << "]";
}

void diagnostic_row(std::ostream &out, uint64_t event, const char *type, double camera_time, const LtvFrame &f, const LtvAdapter &adapter,
                    const ltv::FeaturePipelineContext *ctx) {
  std::set<size_t> left, right;
  if (ctx)
    for (const auto &o : ctx->observations) {
      if (o.camera_id == 0)
        left.insert(o.feature_id);
      if (o.camera_id == 1)
        right.insert(o.feature_id);
    }
  std::vector<size_t> cam0(left.begin(), left.end()), stereo;
  for (auto id : left)
    if (right.count(id))
      stereo.push_back(id);
  out << std::setprecision(17) << "{\"cache_event_index\":" << event << ",\"event_type\":\"" << type << "\",\"camera_time\":" << camera_time
      << ",\"camera_ns_roundtrip\":" << f.camera_ns << ",\"epoch\":" << f.epoch << ",\"sequence\":" << f.sequence
      << ",\"version\":" << f.version << ",\"imu_time\":" << f.imu_time << ",\"cursor\":" << f.cursor << ",\"available\":" << f.available
      << ",\"ready_G\":" << f.ready_G << ",\"ready_V\":" << f.ready_V << ",\"reason\":\"" << f.reason
      << "\",\"state_features\":" << f.snapshot.state_features << ",\"observed_features\":" << f.snapshot.observed_features
      << ",\"mature_features\":" << f.mature_features << ",\"healthy_updates\":" << f.snapshot.healthy_camera_updates
      << ",\"camera_substeps\":" << f.snapshot.camera_substeps << ",\"v_body\":";
  vector(out, f.snapshot.velocity_body);
  out << ",\"eta_body\":";
  vector(out, f.snapshot.gravity_body);
  out << ",\"current_cam0_ids\":";
  id_array(out, cam0);
  out << ",\"current_stereo_ids\":";
  id_array(out, stereo);
  out << ",\"retained_ids\":[";
  const auto &management = adapter.feature_frame().management;
  for (size_t k = 0; k < management.retained_ids.size(); ++k) {
    if (k)
      out << ",";
    out << management.retained_ids[k];
  }
  out << "],\"management_accepted_input\":" << management.accepted_input << ",\"opportunity_tracks\":" << management.opportunity_tracks
      << ",\"admitted_tracks\":" << management.admitted_tracks << ",\"retired_ids\":";
  id_array(out, management.retired_ids);
  out << ",\"seeds\":[";
  const auto &seeds = adapter.feature_frame().seeds;
  for (size_t k = 0; k < seeds.size(); ++k) {
    if (k)
      out << ",";
    const auto &d = seeds[k];
    const auto &c = d.candidate;
    bool admitted = false, mean_written = false;
    for (const auto &b : management.births)
      if (b.feature_id == c.feature_id) {
        admitted = true;
        mean_written = b.apply_seed;
      }
    out << "{\"id\":" << c.feature_id << ",\"time\":" << c.time << ",\"source\":\"" << ltv::toString(c.source)
        << "\",\"geometry_valid\":" << c.geometry_valid << ",\"reason\":\"" << d.estimate.reason << "\",\"admitted\":" << admitted
        << ",\"mean_written\":" << mean_written << ",\"landmark_B\":";
    vector(out, c.landmark_B);
    out << ",\"relative_risk\":";
    number(out, c.relative_risk);
    out << ",\"sigma_parallel\":";
    number(out, d.uncertainty.sigma_parallel);
    out << ",\"range\":";
    number(out, d.estimate.range);
    out << ",\"depth_z\":";
    number(out, d.estimate.depth_z);
    out << ",\"min_ray\":";
    number(out, c.min_ray);
    out << ",\"condition\":";
    number(out, c.condition);
    out << ",\"parallax_rad\":";
    number(out, c.parallax_rad);
    out << ",\"residual_rad\":";
    number(out, c.residual_rad);
    out << ",\"heldout_check_available\":" << d.heldout_check_available
        << ",\"heldout_check_observations\":" << d.heldout_check_observations << ",\"heldout_max_residual_rad\":";
    number(out, d.heldout_max_residual_rad);
    out << ",\"covariance_assumption\":\"" << d.covariance_assumption << "\",\"covariance_B\":[";
    for (int row = 0; row < 3; ++row) {
      if (row)
        out << ",";
      for (int col = 0; col < 3; ++col) {
        if (col)
          out << ",";
        number(out, d.uncertainty.covariance_B(row, col));
      }
    }
    out << "]}";
  }
  out << "],\"tracks\":[";
  const auto *manager = adapter.feature_pipeline() ? &adapter.feature_pipeline()->manager() : nullptr;
  if (manager) {
    bool first = true;
    for (const auto &entry : manager->landmarks()) {
      if (entry.second.phase == ltv::LandmarkPhase::Retired &&
          std::find(management.retired_ids.begin(), management.retired_ids.end(), entry.first) == management.retired_ids.end())
        continue;
      if (!first)
        out << ",";
      first = false;
      const auto &track = entry.second;
      out << "{\"id\":" << entry.first << ",\"phase\":\"" << ltv::toString(track.phase) << "\",\"reason\":\"" << track.reason
          << "\",\"first_seen\":" << track.first_seen << ",\"entered\":" << track.entered << ",\"seeded\":" << track.seeded
          << ",\"last_seen\":" << track.last_seen << ",\"missed_frames\":" << track.missed_frames
          << ",\"ever_opportunity\":" << track.ever_opportunity << ",\"seed_written\":" << track.seed_written << "}";
    }
  }
  out << "]";
  std::vector<size_t> never_admitted_ttl, admitted_retired;
  if (manager)
    for (const auto &entry : manager->landmarks()) {
      const auto &track = entry.second;
      if (track.phase != ltv::LandmarkPhase::Retired)
        continue;
      if (track.entered >= 0)
        admitted_retired.push_back(entry.first);
      else if (track.reason == "candidate_ttl")
        never_admitted_ttl.push_back(entry.first);
    }
  out << ",\"never_admitted_candidate_ttl_ids\":";
  id_array(out, never_admitted_ttl);
  out << ",\"admitted_retired_ids\":";
  id_array(out, admitted_retired);
  out << "}\n";
  if (!out)
    throw std::runtime_error("diagnostic output write failed");
}
LtvPassiveCacheResult recoverCache(const std::string &path, const LtvOptions &options, std::ostream &diagnostics) {
  platform();
  std::ifstream in(path, std::ios::binary);
  char magic[sizeof(MAGIC)];
  in.read(magic, sizeof(magic));
  if (!in || std::memcmp(magic, MAGIC, sizeof(magic)))
    throw std::runtime_error("invalid cache header");
  LtvAdapter adapter(options);
  LtvPassiveCacheResult result;
  Binary outer(static_cast<std::istream &>(in));
  uint64_t index = 0;
  for (;;) {
    uint8_t type = 0;
    outer.scalar(type);
    std::string payload;
    outer.text(payload);
    ++index;
    if (type == 0) {
      if (!payload.empty() || in.peek() != std::char_traits<char>::eof())
        throw std::runtime_error("cache trailing bytes");
      break;
    }
    std::istringstream data(payload, std::ios::binary);
    Binary b(static_cast<std::istream &>(data));
    if (type == 1) {
      ov_core::ImuData s;
      sample(b, s);
      adapter.feed_imu(s);
      if (!result.imu_events)
        result.first_imu_time = s.timestamp;
      result.last_imu_time = s.timestamp;
      ++result.imu_events;
    } else if (type == 2) {
      double time;
      uint64_t version;
      b.scalar(time);
      b.scalar(version);
      size_t n = 0;
      b.count(n);
      std::vector<LtvBearing> obs(n);
      for (auto &v : obs) {
        b.count(v.id, UINT64_MAX);
        b.matrix(v.coordinate);
      }
      LtvCalibration c;
      calibration(b, c);
      Eigen::Vector3d ba, bg;
      b.matrix(ba);
      b.matrix(bg);
      bool present;
      b.scalar(present);
      ltv::FeaturePipelineContext ctx;
      if (present)
        context(b, ctx);
      std::string expected;
      b.text(expected);
      auto f = adapter.process(time, obs, c, ba, bg, version, present ? &ctx : nullptr);
      if (f.available && !adapter.claim(f))
        throw std::runtime_error("cache frame claim rejected");
      same(expected, f, adapter, index);
      diagnostic_row(diagnostics, index, "process", time, f, adapter, present ? &ctx : nullptr);
      ++result.process_events;
      ++result.compared_events;
    } else if (type == 3) {
      double time;
      uint64_t version;
      std::string reason, expected;
      b.scalar(time);
      b.scalar(version);
      b.text(reason);
      b.text(expected);
      auto f = adapter.pause(time, reason, version);
      same(expected, f, adapter, index);
      diagnostic_row(diagnostics, index, "pause", time, f, adapter, nullptr);
      ++result.pause_events;
      ++result.compared_events;
    } else
      throw std::runtime_error("invalid cache event type");
    if (data.peek() != std::char_traits<char>::eof())
      throw std::runtime_error("cache event trailing bytes");
  }
  if (!result.compared_events)
    throw std::runtime_error("cache has no comparable events");
  return result;
}
} // namespace ov_msckf

int main(int argc, char **argv) {
  try {
    if (argc != 5)
      throw std::runtime_error("usage: recover frozen_config.yaml cache.bin new_diagnostics.jsonl new_report.json");
    for (int i : {3, 4}) {
      std::ifstream existing(argv[i]);
      if (existing.good())
        throw std::runtime_error("refusing existing output");
    }
    auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
    ov_msckf::LtvOptions options;
    options.load(parser);
    if (!options.feature_readiness_enabled || options.enable_gravity || options.enable_velocity)
      throw std::runtime_error("requires frozen new Passive method");
    options.passive_cache_path.clear();
    options.log_enabled = false;
    options.value_diagnostics_enabled = false;
    std::ofstream diagnostics(argv[3]);
    if (!diagnostics)
      throw std::runtime_error("cannot create diagnostics");
    auto r = ov_msckf::recoverCache(argv[2], options, diagnostics);
    diagnostics.close();
    std::ofstream report(argv[4]);
    if (!report)
      throw std::runtime_error("cannot create report");
    report << "{\"status\":\"PASS\",\"comparison\":\"exact_frozen_cache_non_wallclock\",\"imu_events\":" << r.imu_events
           << ",\"process_events\":" << r.process_events << ",\"pause_events\":" << r.pause_events
           << ",\"compared_events\":" << r.compared_events << "}\n";
    std::cout << "recovery PASS " << r.compared_events << " exact frame comparisons\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
