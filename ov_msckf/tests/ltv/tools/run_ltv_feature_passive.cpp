// Deterministic ASL input adapter: no GT enters the estimator, no ROS transport drops.
#include "LtvEventReceipt.h"
#include "core/VioManager.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include <algorithm>
#include <boost/uuid/detail/sha1.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <sstream>

using namespace ov_msckf;
struct Image {
  int64_t ns;
  std::string filename;
};
std::vector<Image> images(const std::string &path) {
  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("cannot read " + path);
  std::vector<Image> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    auto comma = line.find(',');
    Image x{std::stoll(line.substr(0, comma)), line.substr(comma + 1)};
    if (!x.filename.empty() && x.filename.back() == '\r')
      x.filename.pop_back();
    if (!rows.empty() && x.ns <= rows.back().ns)
      throw std::runtime_error("non-monotonic images");
    rows.push_back(x);
  }
  return rows;
}
// Digest every public state family and its FEJ, plus full covariance layout.
// No GT interfaces are reachable from this runner.
class Digest {
  boost::uuids::detail::sha1 hash_;

public:
  template <class T> void scalar(const T &v) { hash_.process_bytes(&v, sizeof(v)); }
  void matrix(const Eigen::MatrixXd &m) {
    const Eigen::Index rows = m.rows(), cols = m.cols();
    scalar(rows);
    scalar(cols);
    hash_.process_bytes(m.data(), sizeof(double) * m.size());
  }
  void type(const std::shared_ptr<ov_type::Type> &v) {
    const bool present = bool(v);
    scalar(present);
    if (!v)
      return;
    scalar(v->id());
    scalar(v->size());
    matrix(v->value());
    matrix(v->fej());
  }
  template <class Map> void types(const Map &values) {
    std::vector<typename Map::key_type> keys;
    for (const auto &e : values)
      keys.push_back(e.first);
    std::sort(keys.begin(), keys.end());
    scalar(keys.size());
    for (const auto &key : keys) {
      scalar(key);
      type(values.at(key));
    }
  }
  void ids(const std::vector<size_t> &values) {
    scalar(values.size());
    for (auto v : values)
      scalar(v);
  }
  std::string finish() {
    unsigned int value[5];
    hash_.get_digest(value);
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (auto v : value)
      out << std::setw(8) << v;
    return out.str();
  }
};
class ReplayManager : public VioManager {
  LtvEventReceipt camera_receipt_;

public:
  using VioManager::VioManager;
  void begin_camera_receipt(int64_t left_ns, int64_t right_ns, double time) { camera_receipt_.begin(left_ns, right_ns, time); }
  std::string state_digest() {
    Digest h;
    h.scalar(state->_timestamp);
    h.type(state->_imu);
    h.matrix(StateHelper::get_full_covariance(state));
    h.types(state->_clones_IMU);
    h.types(state->_features_SLAM);
    h.type(state->_calib_dt_CAMtoIMU);
    h.types(state->_calib_IMUtoCAM);
    h.types(state->_cam_intrinsics);
    h.type(state->_calib_imu_dw);
    h.type(state->_calib_imu_da);
    h.type(state->_calib_imu_tg);
    h.type(state->_calib_imu_GYROtoIMU);
    h.type(state->_calib_imu_ACCtoIMU);
    std::vector<size_t> cameras;
    for (const auto &e : state->_cam_intrinsics_cameras)
      cameras.push_back(e.first);
    std::sort(cameras.begin(), cameras.end());
    h.ids(cameras);
    for (auto id : cameras)
      h.matrix(state->_cam_intrinsics_cameras.at(id)->get_value());
    return h.finish();
  }
  std::string tracker_digest() {
    Digest h;
    auto data = trackFEATS->get_feature_database()->get_internal_data();
    std::vector<size_t> ids;
    for (const auto &e : data)
      ids.push_back(e.first);
    std::sort(ids.begin(), ids.end());
    h.ids(ids);
    for (auto id : ids) {
      const auto f = data.at(id);
      h.scalar(id);
      h.scalar(f->to_delete);
      std::vector<size_t> cameras;
      for (const auto &e : f->timestamps)
        cameras.push_back(e.first);
      std::sort(cameras.begin(), cameras.end());
      h.ids(cameras);
      for (auto camera : cameras) {
        const auto &times = f->timestamps.at(camera);
        h.scalar(times.size());
        for (size_t k = 0; k < times.size(); ++k) {
          h.scalar(times[k]);
          h.matrix(f->uvs.at(camera).at(k).cast<double>());
          h.matrix(f->uvs_norm.at(camera).at(k).cast<double>());
        }
      }
    }
    return h.finish();
  }
  std::string visual_digest() {
    Digest h;
    auto observations = passive_current_observations;
    std::sort(observations.begin(), observations.end(), [](const ltv::HistoryObservation &a, const ltv::HistoryObservation &b) {
      return a.feature_id != b.feature_id ? a.feature_id < b.feature_id : a.camera_id < b.camera_id;
    });
    h.scalar(observations.size());
    for (const auto &o : observations) {
      h.scalar(o.feature_id);
      h.scalar(o.camera_id);
      h.matrix(o.bearing);
      h.scalar(o.match_valid);
    }

    h.ids(passive_msckf_input_ids);
    h.ids(passive_msckf_used_ids);
    h.ids(passive_slam_update_ids);
    h.ids(passive_slam_init_ids);
    h.scalar(good_features_MSCKF.size());
    for (const auto &v : good_features_MSCKF)
      h.matrix(v);
    return h.finish();
  }
  void finish_cache() {
    if (passive_cache)
      passive_cache->finish();
  }
  void assert_passive() {
    if (params.ltv_options.enable_gravity || params.ltv_options.enable_velocity || passive_auxiliary_receipts ||
        passive_gravity_submissions || passive_velocity_submissions)
      throw std::runtime_error("PASSIVE VIOLATION: actual auxiliary receipt/submission");
  }
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
  void feature_row(std::ostream &out, int64_t camera_ns) {
    const auto &f = passive_ltv_frame;
    camera_receipt_.validate(camera_ns, f.camera_time, f.available, f.epoch, f.sequence);
    const bool frame_matches = f.camera_time == camera_receipt_.physical_camera_time;
    const bool managed_event = ltv_adapter && ltv_adapter->feature_frame().management.accepted_input && frame_matches &&
                               ltv_adapter->feature_frame().management.epoch == f.epoch &&
                               ltv_adapter->feature_frame().management.time == f.imu_time;
    const bool new_path =
        params.ltv_options.feature_readiness_enabled && bool(ltv_adapter) && (f.available || (f.hardened && managed_event));
    if (new_path && (!frame_matches || !ltv_adapter->feature_pipeline()))
      throw std::runtime_error("Current feature event missing its management diagnostics");
    if (new_path &&
        (ltv_adapter->feature_frame().management.epoch != f.epoch || ltv_adapter->feature_frame().management.time != f.imu_time))
      throw std::runtime_error("Management diagnostics belong to another observer event");
    std::vector<size_t> passive_current_cam0_ids, passive_current_stereo_ids;
    std::set<size_t> left_ids, right_ids;
    for (const auto &o : passive_current_observations) {
      if (o.camera_id == 0)
        left_ids.insert(o.feature_id);
      if (o.camera_id == 1)
        right_ids.insert(o.feature_id);
    }
    passive_current_cam0_ids.assign(left_ids.begin(), left_ids.end());
    for (auto id : left_ids)
      if (right_ids.count(id))
        passive_current_stereo_ids.push_back(id);
    out << std::setprecision(17) << "{\"camera_ns\":" << camera_ns << ",\"receipt_id\":" << camera_receipt_.id
        << ",\"right_camera_ns\":" << camera_receipt_.right_camera_ns << ",\"frame_matches_receipt\":" << frame_matches
        << ",\"management_present\":" << new_path << ",\"management_status\":\""
        << (new_path ? "CURRENT_EVENT"
                     : (!ltv_adapter ? "LTV_DISABLED"
                                     : (!params.ltv_options.feature_readiness_enabled
                                            ? "LEGACY_FEATURE_PATH"
                                            : (frame_matches ? "OBSERVER_UNAVAILABLE" : "NO_OBSERVER_EVENT"))))
        << "\",\"initialized\":" << initialized() << ",\"state_time\":" << state->_timestamp << ",\"epoch\":" << f.epoch
        << ",\"sequence\":" << f.sequence << ",\"target_imu_time\":" << camera_ns * 1e-9 + state->_calib_dt_CAMtoIMU->value()(0)
        << ",\"observer_started\":" << (ltv_adapter && ltv_adapter->core().started()) << ",\"imu_time\":" << f.imu_time
        << ",\"cursor\":" << f.cursor << ",\"available\":" << f.available << ",\"ready_G\":" << f.ready_G << ",\"ready_V\":" << f.ready_V
        << ",\"reason\":\"" << f.reason << "\",\"state_features\":" << f.snapshot.state_features
        << ",\"observed_features\":" << f.snapshot.observed_features << ",\"mature_features\":" << f.mature_features
        << ",\"healthy_updates\":" << f.snapshot.healthy_camera_updates << ",\"camera_substeps\":" << f.snapshot.camera_substeps
        << ",\"v_body\":";
    if (!f.hardened || f.raw_current)
      vector(out, f.snapshot.velocity_body);
    else
      out << "null";
    out << ",\"eta_body\":";
    if (!f.hardened || f.raw_current)
      vector(out, f.snapshot.gravity_body);
    else
      out << "null";
    if (f.hardened) {
      out << ",\"hardening\":true,\"raw_current\":" << f.raw_current << ",\"availability_state\":\"" << ltv::toString(f.health.state)
          << "\",\"pool_ready\":" << f.health.pool_ready << ",\"observer_valid\":" << f.health.observer_valid
          << ",\"joint_ready\":" << (f.ready_G && f.ready_V) << ",\"bootstrap_count\":" << f.health.bootstrap_count
          << ",\"physical_fault_count\":" << f.health.physical_fault_count << ",\"last_bootstrap_time\":" << f.health.last_bootstrap_time
          << ",\"last_bootstrap_source\":\"" << f.health.last_bootstrap_source
          << "\",\"eligible_seeds\":" << ltv_adapter->feature_frame().management.eligible_seeds
          << ",\"actual_corrections\":" << f.actual_corrections << ",\"correction_diagnostics_valid\":" << f.correction_diagnostics_valid
          << ",\"prediction_angle_p95_rad\":";
      number(out, f.prediction_angle_p95_rad);
      out << ",\"velocity_correction_rate\":";
      number(out, f.velocity_correction_rate);
      out << ",\"gravity_correction_rate\":";
      number(out, f.gravity_correction_rate);
    }
    out << ",\"current_cam0_ids\":";
    id_array(out, passive_current_cam0_ids);
    out << ",\"current_stereo_ids\":";
    id_array(out, passive_current_stereo_ids);
    out << ",\"retained_ids\":[";
    if (new_path) {
      const auto &management = ltv_adapter->feature_frame().management;
      for (size_t k = 0; k < management.retained_ids.size(); ++k) {
        if (k)
          out << ",";
        out << management.retained_ids[k];
      }
      out << "],\"opportunity_tracks\":" << management.opportunity_tracks << ",\"admitted_tracks\":" << management.admitted_tracks
          << ",\"retired_ids\":";
      id_array(out, management.retired_ids);
      out << ",\"seeds\":[";
      const auto &seeds = ltv_adapter->feature_frame().seeds;
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
      const auto *manager = ltv_adapter->feature_pipeline() ? &ltv_adapter->feature_pipeline()->manager() : nullptr;
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
    } else
      out << "]";
    // Cumulative per-epoch sets allow exact offline event reconstruction without
    // a second unbounded logger history. Candidate TTL never occupied a core slot.
    std::vector<size_t> candidate_ttl, admitted_retired;
    if (ltv_adapter && ltv_adapter->feature_pipeline())
      for (const auto &entry : ltv_adapter->feature_pipeline()->manager().landmarks()) {
        const auto &track = entry.second;
        if (track.phase != ltv::LandmarkPhase::Retired)
          continue;
        if (track.entered >= 0)
          admitted_retired.push_back(entry.first);
        else if (track.reason == "candidate_ttl")
          candidate_ttl.push_back(entry.first);
      }
    out << ",\"never_admitted_candidate_ttl_ids\":";
    id_array(out, candidate_ttl);
    out << ",\"admitted_retired_ids\":";
    id_array(out, admitted_retired);
    out << "}\n";
    LtvEventReceipt::require_written(out);
    camera_receipt_.complete(camera_ns, f.camera_time, f.available, f.epoch, f.sequence);
  }
  void injection_row(std::ostream &out) {
    out << "," << passive_auxiliary_receipts << "," << passive_gravity_submissions << "," << passive_velocity_submissions;
  }
};
int main(int argc, char **argv) {
  try {
    if (argc < 5 || argc > 6)
      throw std::runtime_error(
          "usage: run_ltv_feature_passive config.yaml sensor_ASL_root output_dir B|P_OLD|P_NEW [short_input_seconds<=10]");
    const std::string root = argv[2], out = argv[3];
    const std::string mode = argv[4];
    const double limit = argc == 6 ? std::stod(argv[5]) : 0;
    if ((mode != "B" && mode != "P_OLD" && mode != "P_NEW") || !std::isfinite(limit) || limit < 0 || limit > 10 ||
        (argc == 6 && limit == 0))
      throw std::runtime_error("invalid passive mode/short input duration");
    auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
    VioManagerOptions options;
    options.print_and_load(parser);
    if (options.ltv_options.enable_gravity || options.ltv_options.enable_velocity)
      throw std::runtime_error("PASSIVE only: injection settings must be OFF in input configuration");
    if (options.ltv_options.enabled != (mode != "B") || options.ltv_options.feature_readiness_enabled != (mode == "P_NEW"))
      throw std::runtime_error("mode differs from parsed configuration");
    options.ltv_options.passive_audit_enabled = true;
    options.ltv_options.passive_cache_path = mode == "B" ? "" : out + "/cache.bin";
    // Counterfactual injection diagnostics are not part of this passive experiment.
    if (options.ltv_options.value_diagnostics_enabled)
      throw std::runtime_error("disable legacy counterfactual value diagnostics for passive runner");
    const auto &c0 = options.ltv_options.observer;
    if (c0.q_landmark != 1e-4 || c0.v_landmark != 1e6 || c0.v_velocity != 1e6 || c0.v_gravity != 1e6 || c0.initial_p_landmark != 1 ||
        c0.initial_p_velocity != 1 || c0.initial_p_gravity != 1 || c0.max_features != 30 || c0.min_features != 15)
      throw std::runtime_error("observer must retain C0 Q/V/P0 and 30/15 feature limits");
    cv::setNumThreads(1);
    options.use_multi_threading_subs = false;
    options.use_multi_threading_pubs = false;
    // Serialize values after native parsing and the two explicit deterministic runner overrides.
    std::ofstream effective(out + "/effective_options.json");
    if (!effective)
      throw std::runtime_error("cannot write effective configuration");
    effective << std::setprecision(17) << "{\n";
#define EFFECTIVE(key, value) effective << "\"" << key << "\": " << value << ",\n"
    EFFECTIVE("ltv_value_diagnostics_enabled", options.ltv_options.value_diagnostics_enabled);
    EFFECTIVE("ltv_value_diagnostics_matrices", options.ltv_options.value_diagnostics_matrices);
    EFFECTIVE("ltv_enabled", options.ltv_options.enabled);
    EFFECTIVE("ltv_feature_readiness_enabled", options.ltv_options.feature_readiness_enabled);
    EFFECTIVE("ltv_feature_apply_seed", options.ltv_options.feature_apply_seed);
    EFFECTIVE("ltv_passive_audit_enabled", options.ltv_options.passive_audit_enabled);
    EFFECTIVE("ltv_enable_gravity", options.ltv_options.enable_gravity);
    EFFECTIVE("ltv_enable_velocity", options.ltv_options.enable_velocity);
    EFFECTIVE("ltv_allow_correlated_pseudomeasurements", options.ltv_options.allow_correlated_pseudomeasurements);
    EFFECTIVE("ltv_enable_huber", options.ltv_options.enable_huber);
    EFFECTIVE("ltv_huber_delta", options.ltv_options.huber_delta);
    EFFECTIVE("ltv_enable_quality_gate", options.ltv_options.enable_quality_gate);
    EFFECTIVE("ltv_enable_nis_gate", options.ltv_options.enable_nis_gate);
    EFFECTIVE("ltv_log_enabled", options.ltv_options.log_enabled);
    EFFECTIVE("ltv_sigma_gravity_deg", options.ltv_options.sigma_gravity_deg);
    EFFECTIVE("ltv_sigma_velocity_mps", options.ltv_options.sigma_velocity_mps);
    EFFECTIVE("ltv_max_gravity_angle_deg", options.ltv_options.max_gravity_angle_deg);
    EFFECTIVE("ltv_time_tolerance_s", options.ltv_options.time_tolerance_s);
    EFFECTIVE("ltv_nis_gravity", options.ltv_options.nis_gravity);
    EFFECTIVE("ltv_nis_velocity", options.ltv_options.nis_velocity);
    EFFECTIVE("ltv_quality_eta_error", options.ltv_options.quality_eta_error);
    EFFECTIVE("ltv_quality_innovation", options.ltv_options.quality_innovation);
    EFFECTIVE("ltv_quality_velocity_disagreement", options.ltv_options.quality_velocity_disagreement);
    EFFECTIVE("ltv_quality_min_features", options.ltv_options.quality_min_features);
    EFFECTIVE("ltv_observer_max_features", options.ltv_options.observer.max_features);
    EFFECTIVE("ltv_observer_min_features", options.ltv_options.observer.min_features);
    EFFECTIVE("ltv_observer_max_missed_frames", options.ltv_options.observer.max_missed_frames);
    EFFECTIVE("ltv_observer_warmup_camera_updates", options.ltv_options.observer.warmup_camera_updates);
    EFFECTIVE("ltv_observer_max_imu_dt", options.ltv_options.observer.max_imu_dt);
    EFFECTIVE("ltv_observer_reset_gap", options.ltv_options.observer.reset_gap);
    EFFECTIVE("ltv_observer_timestamp_tolerance", options.ltv_options.observer.timestamp_tolerance);
    EFFECTIVE("ltv_observer_q_landmark", options.ltv_options.observer.q_landmark);
    EFFECTIVE("ltv_observer_v_landmark", options.ltv_options.observer.v_landmark);
    EFFECTIVE("ltv_observer_v_velocity", options.ltv_options.observer.v_velocity);
    EFFECTIVE("ltv_observer_v_gravity", options.ltv_options.observer.v_gravity);
    EFFECTIVE("ltv_observer_initial_p_landmark", options.ltv_options.observer.initial_p_landmark);
    EFFECTIVE("ltv_observer_initial_p_velocity", options.ltv_options.observer.initial_p_velocity);
    EFFECTIVE("ltv_observer_initial_p_gravity", options.ltv_options.observer.initial_p_gravity);
    EFFECTIVE("ltv_observer_covariance_floor", options.ltv_options.observer.covariance_floor);
    EFFECTIVE("ltv_observer_covariance_failure_threshold", options.ltv_options.observer.covariance_failure_threshold);
    EFFECTIVE("ltv_observer_camera_euler_safety", options.ltv_options.observer.camera_euler_safety);
    EFFECTIVE("ltv_observer_max_camera_substeps", options.ltv_options.observer.max_camera_substeps);
    EFFECTIVE("ltv_observer_gravity_norm_min", options.ltv_options.observer.gravity_norm_min);
    EFFECTIVE("ltv_observer_gravity_norm_max", options.ltv_options.observer.gravity_norm_max);
    EFFECTIVE("do_fej", options.state_options.do_fej);
    EFFECTIVE("do_calib_camera_pose", options.state_options.do_calib_camera_pose);
    EFFECTIVE("do_calib_camera_intrinsics", options.state_options.do_calib_camera_intrinsics);
    EFFECTIVE("do_calib_camera_timeoffset", options.state_options.do_calib_camera_timeoffset);
    EFFECTIVE("do_calib_imu_intrinsics", options.state_options.do_calib_imu_intrinsics);
    EFFECTIVE("do_calib_imu_g_sensitivity", options.state_options.do_calib_imu_g_sensitivity);
    EFFECTIVE("num_cameras", options.state_options.num_cameras);
    EFFECTIVE("max_slam_features", options.state_options.max_slam_features);
    EFFECTIVE("max_clone_size", options.state_options.max_clone_size);
    EFFECTIVE("calib_camimu_dt", options.calib_camimu_dt);
    EFFECTIVE("use_multi_threading_subs", options.use_multi_threading_subs);
    EFFECTIVE("use_multi_threading_pubs", options.use_multi_threading_pubs);
#undef EFFECTIVE
    effective << "\"correlation_model\": \"" << options.ltv_options.correlation_model << "\"\n}\n";
    effective.close();
    ov_core::Printer::setPrintLevel("WARNING");
    ReplayManager app(options);
    app.assert_passive();
    const auto raw_left = images(root + "/cam0/data.csv"), raw_right = images(root + "/cam1/data.csv");
    // Exact-time synchronization tolerates missing messages, never pairs different headers.
    // Every unmatched input is recorded, including leading/trailing messages.
    std::vector<Image> left, right;
    std::ofstream unmatched(out + "/unmatched_camera.csv");
    if (!unmatched)
      throw std::runtime_error("cannot write synchronization audit");
    unmatched << "camera_id,timestamp_ns,filename\n";
    size_t il = 0, ir = 0;
    while (il < raw_left.size() || ir < raw_right.size()) {
      if (il < raw_left.size() && ir < raw_right.size() && raw_left[il].ns == raw_right[ir].ns) {
        left.push_back(raw_left[il++]);
        right.push_back(raw_right[ir++]);
      } else if (ir == raw_right.size() || (il < raw_left.size() && raw_left[il].ns < raw_right[ir].ns)) {
        unmatched << "0," << raw_left[il].ns << "," << raw_left[il].filename << "\n";
        ++il;
      } else {
        unmatched << "1," << raw_right[ir].ns << "," << raw_right[ir].filename << "\n";
        ++ir;
      }
    }
    if (left.empty())
      throw std::runtime_error("UNSUPPORTED no exact-time stereo pairs");
    std::ifstream imu_file(root + "/imu0/data.csv");
    if (!imu_file)
      throw std::runtime_error("no IMU");
    std::vector<ov_core::ImuData> imus;
    std::string line;
    int64_t last_ns = -1;
    while (std::getline(imu_file, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      int64_t ns;
      ov_core::ImuData x;
      row >> ns >> x.wm(0) >> x.wm(1) >> x.wm(2) >> x.am(0) >> x.am(1) >> x.am(2);
      if (!row || ns <= last_ns || !x.am.allFinite() || !x.wm.allFinite())
        throw std::runtime_error("invalid IMU row");
      last_ns = ns;
      x.timestamp = ns * 1e-9;
      imus.push_back(x);
    }
    std::ofstream trajectory(out + "/trajectory.csv"), audit(out + "/audit.csv"), features(out + "/features.jsonl");
    if (!trajectory || !audit || !features)
      throw std::runtime_error("output must exist");
    trajectory << "timestamp,qx,qy,qz,qw,px,py,pz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n" << std::setprecision(17);
    audit << "camera_ns,initialized,state_time,clones,state_digest,tracker_digest,visual_digest,auxiliary_receipts,gravity_submissions,"
             "velocity_submissions\n"
          << std::setprecision(17);
    if (imus.empty())
      throw std::runtime_error("empty IMU input");
    const double sensor_start = std::min(imus.front().timestamp, left.front().ns * 1e-9 + options.calib_camimu_dt);
    const double sensor_end = limit ? sensor_start + limit : std::numeric_limits<double>::infinity();
    double last_input_time = sensor_start;
    size_t cursor = 0, packets = 0, outputs = 0;
    for (size_t i = 0; i < left.size(); ++i) {
      double t = left[i].ns * 1e-9;
      double target = t + options.calib_camimu_dt;
      if (limit) {
        auto right_endpoint =
            std::upper_bound(imus.begin(), imus.end(), target, [](double value, const ov_core::ImuData &x) { return value < x.timestamp; });
        if (target > sensor_end || (right_endpoint != imus.end() && right_endpoint->timestamp > sensor_end))
          break;
      }
      while (cursor < imus.size() && imus[cursor].timestamp <= target) {
        app.feed_measurement_imu(imus[cursor++]);
      }
      if (cursor < imus.size())
        app.feed_measurement_imu(imus[cursor++]); // one right endpoint, cached only
      ov_core::CameraData camera;
      camera.timestamp = t;
      camera.sensor_ids = {0, 1};
      for (auto name : {std::string("cam0/data/") + left[i].filename, std::string("cam1/data/") + right[i].filename}) {
        auto image = cv::imread(root + "/" + name, cv::IMREAD_GRAYSCALE);
        if (image.empty())
          throw std::runtime_error("missing image " + name);
        camera.images.push_back(image);
        camera.masks.push_back(options.use_mask ? options.masks.at(camera.masks.size()).clone() : cv::Mat::zeros(image.size(), CV_8UC1));
      }
      app.begin_camera_receipt(left[i].ns, right[i].ns, t);
      app.feed_measurement_camera(camera);
      app.assert_passive();
      last_input_time = std::max(last_input_time, target);
      if (cursor)
        last_input_time = std::max(last_input_time, imus[cursor - 1].timestamp);
      ++packets;
      auto state = app.get_state();
      audit << left[i].ns << "," << app.initialized() << "," << state->_timestamp << "," << state->_clones_IMU.size() << ","
            << app.state_digest() << "," << app.tracker_digest() << "," << app.visual_digest();
      app.feature_row(features, left[i].ns);
      app.injection_row(audit);
      audit << "\n";
      if (app.initialized() && state->_timestamp == t) {
        const Eigen::VectorXd x = state->_imu->value();
        if (!x.allFinite() || !StateHelper::get_full_covariance(state).allFinite())
          throw std::runtime_error("non-finite native state/P");
        trajectory << target;
        for (int j = 0; j < x.size(); ++j)
          trajectory << "," << x(j);
        trajectory << "\n";
        ++outputs;
      }
    }
    if (!limit) {
      while (cursor < imus.size())
        app.feed_measurement_imu(imus[cursor++]);
      last_input_time = std::max(last_input_time, imus.back().timestamp);
    }
    if (limit && last_input_time - sensor_start > limit + 1e-9)
      throw std::runtime_error("short input exceeded registered duration");
    trajectory.flush();
    audit.flush();
    unmatched.flush();
    features.flush();
    if (!trajectory || !audit || !unmatched || !features)
      throw std::runtime_error("diagnostic output write failed");
    app.finish_cache();
    std::ofstream meta(out + "/replay.json");
    meta << "{\"input_left_images\":" << raw_left.size() << ",\"input_right_images\":" << raw_right.size()
         << ",\"unmatched_left_images\":" << raw_left.size() - left.size()
         << ",\"unmatched_right_images\":" << raw_right.size() - right.size() << ",\"camera_packets\":" << packets
         << ",\"input_camera_packets\":" << left.size() << ",\"imu_consumed\":" << cursor << ",\"input_imu_samples\":" << imus.size()
         << ",\"output_rows\":" << outputs << ",\"input_span_seconds\":" << std::setprecision(17) << last_input_time - sensor_start
         << ",\"short_limit_seconds\":" << limit << ",\"mode\":\"" << mode
         << "\",\"actual_G_submissions\":0,\"actual_V_submissions\":0,\"complete\":" << (!limit ? "true" : "false") << "}\n";
    meta.flush();
    if (!meta)
      throw std::runtime_error("replay metadata write failed");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "REPLAY FAILURE: " << e.what() << std::endl;
    return 1;
  }
}
