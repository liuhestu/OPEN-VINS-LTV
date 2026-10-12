#include "ltv/LtvOptions.h"
#include <yaml-cpp/yaml.h>

namespace ov_msckf {
void LtvOptions::load_profile(const std::string &path, const std::string &override_mode, const std::string &override_scene) {
  profile_path = path;
  mode = "OFF";
  scene = "MH";
  weight_G = weight_V = weight_L = 1;
  YAML::Node yaml;
  if (!path.empty()) {
    try {
      yaml = YAML::LoadFile(path);
    } catch (const YAML::Exception &error) {
      throw std::invalid_argument("Cannot read LTV config " + path + ": " + error.what());
    }
    if (!yaml.IsMap() || !yaml["mode"].IsScalar())
      throw std::invalid_argument("LTV config requires mode");
    mode = yaml["mode"].as<std::string>();
    if (yaml["scene"])
      scene = yaml["scene"].as<std::string>();
  }
  if (!override_mode.empty())
    mode = override_mode;
  if (!override_scene.empty())
    scene = override_scene;
  if (mode != "OFF" && mode != "G" && mode != "V" && mode != "L" && mode != "GVL")
    throw std::invalid_argument("LTV mode must be OFF, G, V, L or GVL");
  enabled = mode != "OFF";
  if (enabled && path.empty())
    throw std::invalid_argument("An enabled LTV mode requires ltv_config");
  enable_gravity = mode == "G" || mode == "GVL";
  enable_velocity = mode == "V" || mode == "GVL";
  enable_landmark_approx = mode == "L" || mode == "GVL";
  allow_correlated_pseudomeasurements = enabled;
  YAML::Node weights;
  if (!path.empty()) {
    if (yaml["profiles"]) {
      const auto profiles = yaml["profiles"];
      if (!profiles.IsMap() || !profiles[scene] || !profiles[scene].IsMap() || !profiles[scene][mode] || !profiles[scene][mode].IsMap())
        throw std::invalid_argument("Missing LTV preset: " + scene + "/" + mode);
      weights = profiles[scene][mode]["weights"];
    } else {
      weights = yaml["weights"]; // Generated single-mode profiles remain supported.
    }
  }
  auto weight = [&](const char *name, bool active, double &value) {
    bool numeric = false;
    if (weights.IsMap() && weights[name].IsScalar()) {
      try {
        value = weights[name].as<double>();
        numeric = true;
      } catch (const YAML::Exception &) {
      }
    }
    if (active && (!numeric || !std::isfinite(value) || value <= 0))
      throw std::invalid_argument(std::string("LTV enabled weight must be finite and positive: ") + name);
  };
  weight("G", enable_gravity, weight_G);
  weight("V", enable_velocity, weight_V);
  weight("L", enable_landmark_approx, weight_L);
  sigma_gravity_deg = 10 / std::sqrt(enable_gravity ? weight_G : 1);
  sigma_velocity_mps = 1 / std::sqrt(enable_velocity ? weight_V : 1);
  landmark_approx_sigma_floor_m = .5 / std::sqrt(enable_landmark_approx ? weight_L : 1);
  landmark_approx_information_cap = .01 * (enable_landmark_approx ? weight_L : 1);
}
} // namespace ov_msckf
