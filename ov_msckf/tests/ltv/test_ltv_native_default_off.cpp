// Deterministic native main-estimator regression: default-off or Passive C02.
// Passive mode also writes the lossless input/observer cache for exact comparison.
// Simulation initializes the main filter with GT, as upstream run_simulation does.
#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "state/StateHelper.h"
#include <boost/uuid/detail/sha1.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
using namespace ov_msckf;
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
class RegressionManager : public VioManager {
public:
  using VioManager::VioManager;
  void finish_cache() {
    if (passive_cache)
      passive_cache->finish();
  }
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
};
int main(int argc, char **argv) {
  if (argc != 3 && argc != 4)
    return 2;
  const bool passive = argc == 4 && std::string(argv[3]) == "--passive-c02";
  if (argc == 4 && !passive)
    return 2;
  const int frame_limit = passive ? 80 : 40;
  auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
  ov_core::Printer::setPrintLevel("SILENT");
  VioManagerOptions params;
  params.print_and_load(parser);
  params.print_and_load_simulation(parser);
  const auto &o = params.ltv_options;
  if (o.enable_gravity || o.enable_velocity)
    throw std::runtime_error("regression forbids G/V injection");
  if (passive) {
    if (!o.enabled || !o.feature_readiness_enabled || !o.passive_hardening_enabled || !o.hardening_initial_warmup ||
        !o.hardening_preserve_constrained_state || !o.hardening_ready_soft_grace || !o.active_consistency.enabled ||
        o.active_consistency.max_history_residual_rad != .02 || o.active_consistency.max_holdout_residual_rad != .02 ||
        o.feature_seed_source != "STEREO_THEN_TEMPORAL")
      throw std::runtime_error("passive regression requires frozen C02");
    params.ltv_options.passive_cache_path = std::string(argv[2]) + ".ltvcache";
  } else if (o.enabled)
    throw std::runtime_error("default-off regression requires LTV disabled");
  params.num_opencv_threads = 0;
  params.use_multi_threading_pubs = false;
  params.use_multi_threading_subs = false;
  auto sim = std::make_shared<Simulator>(params);
  RegressionManager sys(params);
  Eigen::Matrix<double, 17, 1> initial;
  if (!sim->get_state(sim->current_timestamp() + 1.0 / params.sim_freq_imu, initial))
    throw std::runtime_error("simulator initialization failed");
  initial(0) -= sim->get_true_parameters().calib_camimu_dt;
  sys.initialize_with_gt(initial);
  std::ofstream output(argv[2]);
  if (!output)
    throw std::runtime_error("cannot open regression output");
  double camera = -1;
  std::vector<int> cameras;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> features;
  int frames = 0, imu_count = 0;
  while (sim->ok() && frames < frame_limit) {
    ov_core::ImuData imu;
    if (sim->get_next_imu(imu.timestamp, imu.wm, imu.am)) {
      sys.feed_measurement_imu(imu);
      ++imu_count;
    }
    double next_camera;
    std::vector<int> next_cameras;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> next_features;
    if (sim->get_next_cam(next_camera, next_cameras, next_features)) {
      if (camera >= 0) {
        sys.feed_measurement_simulation(camera, cameras, features);
        output << frames++ << " " << sys.state_digest() << '\n';
      }
      camera = next_camera;
      cameras = std::move(next_cameras);
      features = std::move(next_features);
    }
  }
  if (frames != frame_limit || imu_count < 40 || !StateHelper::get_full_covariance(sys.get_state()).allFinite())
    throw std::runtime_error("incomplete/nonfinite native regression");
  sys.finish_cache();
  output << "frames=" << frames << " imu=" << imu_count << '\n';
}
