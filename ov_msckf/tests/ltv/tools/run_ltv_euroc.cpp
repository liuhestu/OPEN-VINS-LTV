// Deterministic ASL input adapter: no GT enters the estimator, no ROS transport drops.
#include "core/VioManager.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include <boost/uuid/detail/sha1.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
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
class ReplayManager : public VioManager {
public:
  using VioManager::VioManager;
  std::string digest() {
    boost::uuids::detail::sha1 hash;
    auto bytes = [&](const void *p, size_t n) { hash.process_bytes(p, n); };
    auto matrix = [&](const Eigen::MatrixXd &m) {
      Eigen::Index rows = m.rows(), cols = m.cols();
      bytes(&rows, sizeof(rows));
      bytes(&cols, sizeof(cols));
      bytes(m.data(), sizeof(double) * m.size());
    };
    bytes(&state->_timestamp, sizeof(double));
    matrix(state->_imu->value());
    matrix(state->_imu->fej());
    matrix(StateHelper::get_full_covariance(state));
    for (const auto &entry : state->_clones_IMU) {
      bytes(&entry.first, sizeof(double));
      matrix(entry.second->value());
      matrix(entry.second->fej());
    }
    auto data = trackFEATS->get_feature_database()->get_internal_data();
    std::vector<size_t> ids;
    for (const auto &entry : data)
      ids.push_back(entry.first);
    std::sort(ids.begin(), ids.end());
    for (size_t id : ids) {
      bytes(&id, sizeof(id));
      const auto f = data.at(id);
      bytes(&f->to_delete, sizeof(f->to_delete));
    }
    for (const auto &v : good_features_MSCKF)
      matrix(v);
    unsigned int value[5];
    hash.get_digest(value);
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (auto v : value)
      out << std::setw(8) << v;
    return out.str();
  }
};
int main(int argc, char **argv) {
  try {
    if (argc < 4 || argc > 5)
      throw std::runtime_error("usage: run_ltv_euroc config.yaml ASL/mav0 output_dir [max_camera_packets]");
    const std::string root = argv[2], out = argv[3];
    const int limit = argc == 5 ? std::stoi(argv[4]) : 0;
    auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
    VioManagerOptions options;
    options.print_and_load(parser);
    options.use_multi_threading_subs = false;
    options.use_multi_threading_pubs = false;
    // Serialize values after native parsing and the two explicit deterministic runner overrides.
    std::ofstream effective(out + "/effective_options.json");
    if (!effective)
      throw std::runtime_error("cannot write effective configuration");
    effective << std::setprecision(17) << "{\n";
#define EFFECTIVE(key, value) effective << "\"" << key << "\": " << value << ",\n"
    EFFECTIVE("ltv_enabled", options.ltv_options.enabled);
    EFFECTIVE("ltv_enable_gravity", options.ltv_options.enable_gravity);
    EFFECTIVE("ltv_enable_velocity", options.ltv_options.enable_velocity);
    EFFECTIVE("ltv_allow_correlated_pseudomeasurements", options.ltv_options.allow_correlated_pseudomeasurements);
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
      if (!row || ns <= last_ns)
        throw std::runtime_error("invalid IMU row");
      last_ns = ns;
      x.timestamp = ns * 1e-9;
      imus.push_back(x);
    }
    std::ofstream trajectory(out + "/trajectory.csv"), audit(out + "/audit.csv");
    if (!trajectory || !audit)
      throw std::runtime_error("output must exist");
    trajectory << "timestamp,qx,qy,qz,qw,px,py,pz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n" << std::setprecision(17);
    audit << "camera_ns,initialized,state_time,clones,digest\n" << std::setprecision(17);
    size_t cursor = 0, packets = 0, outputs = 0;
    for (size_t i = 0; i < left.size() && (!limit || packets < size_t(limit)); ++i) {
      double t = left[i].ns * 1e-9;
      double target = t + options.calib_camimu_dt;
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
        camera.masks.push_back(cv::Mat::zeros(image.size(), CV_8UC1));
      }
      app.feed_measurement_camera(camera);
      ++packets;
      auto state = app.get_state();
      audit << left[i].ns << "," << app.initialized() << "," << state->_timestamp << "," << state->_clones_IMU.size() << "," << app.digest()
            << "\n";
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
    if (!limit)
      while (cursor < imus.size())
        app.feed_measurement_imu(imus[cursor++]);
    std::ofstream meta(out + "/replay.json");
    meta << "{\"input_left_images\":" << raw_left.size() << ",\"input_right_images\":" << raw_right.size()
         << ",\"unmatched_left_images\":" << raw_left.size() - left.size()
         << ",\"unmatched_right_images\":" << raw_right.size() - right.size() << ",\"camera_packets\":" << packets
         << ",\"input_camera_packets\":" << left.size() << ",\"imu_consumed\":" << cursor << ",\"input_imu_samples\":" << imus.size()
         << ",\"output_rows\":" << outputs << ",\"complete\":" << (!limit ? "true" : "false") << "}\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "REPLAY FAILURE: " << e.what() << std::endl;
    return 1;
  }
}
