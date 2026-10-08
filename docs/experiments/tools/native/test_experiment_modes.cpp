#include "ExperimentMode.h"
#include <cassert>
#include <iostream>
int main() {
  const char *names[] = {"OFF", "G", "V", "GV", "L", "L_GV"};
  const bool expected[][4] = {{false, false, false, false}, {true, true, false, false}, {true, false, true, false},
                              {true, true, true, false},    {true, false, false, true}, {true, true, true, true}};
  for (int i = 0; i < 6; ++i) {
    const auto m = experiment_mode(names[i]);
    assert(m.name == names[i]);
    assert(m.observer == expected[i][0]);
    assert(m.gravity == expected[i][1]);
    assert(m.velocity == expected[i][2]);
    assert(m.landmark == expected[i][3]);
    assert(m.landmark_shadow == m.landmark);
  }
  assert(experiment_mode("L_ON").name == "L");
  bool rejected = false;
  try {
    experiment_mode("invalid");
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  std::cout << "PASS six mode flags and invalid mode rejection\n";
}
