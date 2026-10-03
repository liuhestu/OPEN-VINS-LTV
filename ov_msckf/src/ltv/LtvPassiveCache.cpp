#include "LtvPassiveCache.h"
#include <cstring>
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
LtvPassiveCacheWriter::LtvPassiveCacheWriter(const std::string &path) {
  if (path.empty())
    return;
  platform();
  std::ifstream previous(path, std::ios::binary);
  if (previous.good())
    throw std::runtime_error("refusing cache overwrite: " + path);
  out_.open(path, std::ios::binary);
  if (!out_)
    throw std::runtime_error("cannot create cache: " + path);
  out_.write(MAGIC, sizeof(MAGIC));
  enabled_ = true;
}
void LtvPassiveCacheWriter::record(uint8_t type, const std::string &payload) {
  if (!enabled_)
    return;
  if (finished_)
    throw std::runtime_error("cache already finalized");
  Binary b(static_cast<std::ostream &>(out_));
  b.scalar(type);
  auto copy = payload;
  b.text(copy);
  out_.flush();
  if (!out_)
    throw std::runtime_error("cache flush failed");
}
void LtvPassiveCacheWriter::imu(const ov_core::ImuData &s) {
  if (!enabled_)
    return;
  std::ostringstream o(std::ios::binary);
  Binary b(static_cast<std::ostream &>(o));
  auto copy = s;
  sample(b, copy);
  record(1, o.str());
}
void LtvPassiveCacheWriter::process(double time, const std::vector<LtvBearing> &bearings, const LtvCalibration &c,
                                    const Eigen::Vector3d &ba, const Eigen::Vector3d &bg, uint64_t version,
                                    const ltv::FeaturePipelineContext *ctx, const LtvFrame &expected, const LtvAdapter &adapter) {
  if (!enabled_)
    return;
  std::ostringstream o(std::ios::binary);
  Binary b(static_cast<std::ostream &>(o));
  b.scalar(time);
  b.scalar(version);
  auto obs = bearings;
  size_t n = obs.size();
  b.count(n);
  for (auto &v : obs) {
    b.count(v.id, UINT64_MAX);
    b.matrix(v.coordinate);
  }
  auto cal = c;
  calibration(b, cal);
  Eigen::Vector3d a = ba, g = bg;
  b.matrix(a);
  b.matrix(g);
  bool present = ctx != nullptr;
  b.scalar(present);
  if (present) {
    auto copy = *ctx;
    context(b, copy);
  }
  auto e = evidence(expected, adapter);
  b.text(e);
  record(2, o.str());
}
void LtvPassiveCacheWriter::pause(double time, const std::string &reason, uint64_t version, const LtvFrame &expected,
                                  const LtvAdapter &adapter) {
  if (!enabled_)
    return;
  std::ostringstream o(std::ios::binary);
  Binary b(static_cast<std::ostream &>(o));
  b.scalar(time);
  b.scalar(version);
  auto r = reason;
  b.text(r);
  auto e = evidence(expected, adapter);
  b.text(e);
  record(3, o.str());
}
void LtvPassiveCacheWriter::finish() {
  if (!enabled_ || finished_)
    return;
  record(0, "");
  finished_ = true;
  out_.close();
}
LtvPassiveCacheResult replayLtvPassiveCache(const std::string &path, const LtvOptions &options) {
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
