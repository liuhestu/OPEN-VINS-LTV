#include "native_test_support.h"
#include <cstdio>
#include <fstream>
#include <unistd.h>
int main() {
  const std::string file = "/tmp/ltv-options-" + std::to_string(getpid()) + ".yaml";
  auto load = [&](const std::string &contents) {
    std::ofstream out(file);
    out << "%YAML:1.0\n" << contents;
    out.close();
    auto parser = std::make_shared<ov_core::YamlParser>(file);
    LtvOptions options;
    options.load(parser);
    return options;
  };
  auto off = load("unused: 1\n");
  require(!off.enabled && !off.enable_gravity && !off.enable_velocity && !off.allow_correlated_pseudomeasurements,
          "missing fields default OFF");
  const std::string active = "ltv_enabled: true\nltv_enable_gravity: true\nltv_allow_correlated_pseudomeasurements: true\n";
  auto first = load(active + "ltv_sigma_gravity_deg: 10.0\nltv_sigma_velocity_mps: 1.0\nltv_observer_max_features: 35\n");
  auto second = load(active + "ltv_sigma_gravity_deg: 20.0\nltv_sigma_velocity_mps: 2.0\nltv_observer_max_features: 35\n");
  first.validate(StateOptions());
  second.validate(StateOptions());
  require(first.observer.max_features == 35, "flat observer parameter actually read");
  auto s = make_state();
  ltv::LtvSnapshot snapshot;
  snapshot.gravity_body = s->_imu->Rot() * Eigen::Vector3d(0, 0, -9.81);
  snapshot.velocity_body.setZero();
  UpdaterLTV a(first), b(second);
  auto c = a.context(*s);
  metric("yaml_gravity_sigma_squared", (b.gravity(*s, c, snapshot).R - 4 * a.gravity(*s, c, snapshot).R).norm(), 1e-14);
  metric("yaml_velocity_sigma_squared", (b.velocity(*s, c, snapshot).R - 4 * a.velocity(*s, c, snapshot).R).norm(), 1e-14);
  metric("degrees_to_radians_once", std::abs(a.gravity(*s, c, snapshot).R(0, 0) - std::pow(10 * std::acos(-1.0) / 180, 2)), 1e-15);
  int rejected = 0;
  for (int i = 0; i < 5; ++i) {
    auto invalid = first;
    auto state = StateOptions();
    if (i == 0)
      invalid.allow_correlated_pseudomeasurements = false;
    if (i == 1)
      invalid.enabled = false;
    if (i == 2)
      invalid.sigma_gravity_deg = 0;
    if (i == 3)
      state.do_calib_camera_pose = true;
    if (i == 4)
      state.num_cameras = 3;
    try {
      invalid.validate(state);
    } catch (const std::invalid_argument &) {
      ++rejected;
    }
  }
  require(rejected == 5, "invalid/unsupported configuration fails explicitly");
  std::remove(file.c_str());
}
