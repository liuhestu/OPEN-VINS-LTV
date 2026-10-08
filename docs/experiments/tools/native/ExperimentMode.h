#pragma once
#include <stdexcept>
#include <string>

struct ExperimentMode {
  std::string name;
  bool observer, gravity, velocity, landmark, landmark_shadow;
};

inline ExperimentMode experiment_mode(const std::string &name) {
  if (name == "OFF")
    return {"OFF", false, false, false, false, false};
  if (name == "G")
    return {"G", true, true, false, false, false};
  if (name == "V")
    return {"V", true, false, true, false, false};
  if (name == "GV")
    return {"GV", true, true, true, false, false};
  if (name == "L" || name == "L_ON")
    return {"L", true, false, false, true, true};
  if (name == "L_GV")
    return {"L_GV", true, true, true, true, true};
  // Historical aliases are excluded from the canonical six-mode batch.
  if (name == "L_OFF")
    return {"L_OFF", true, false, false, false, true};
  if (name == "V10")
    return {"V10", true, false, true, false, false};
  throw std::invalid_argument("expected OFF|G|V|GV|L|L_GV (legacy L_OFF|L_ON|V10)");
}
