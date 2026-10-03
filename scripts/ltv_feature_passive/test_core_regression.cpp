#include "ltv/ltv_observer.h"
#define ltv ltv_baseline
#include "ltv_observer.h"
#undef ltv
#include <cassert>
#include <iostream>
int main() {
  ltv::LtvObserver a;
  ltv_baseline::LtvObserver b;
  ltv::LtvConfig ca;
  ltv_baseline::LtvConfig cb;
  ca.enable = cb.enable = true;
  a.configure(ca);
  b.configure(cb);
  a.start(0);
  b.start(0);
  auto same = [&]() {
    assert(a.state().size() == b.state().size());
    assert((a.state().array() == b.state().array()).all());
    assert((a.covariance().array() == b.covariance().array()).all());
    assert(a.started() == b.started());
    for (int id = 0; id < 64; ++id) assert(a.slotForFeature(id) == b.slotForFeature(id));
    auto x = a.snapshot(), y = ltv::LtvSnapshot();
    auto z = b.snapshot();
    (void)y;
    assert(x.valid == z.valid);
  };
  for (int tick = 0; tick <= 500; ++tick) {
    double t = tick * .005;
    if (tick) {
      Eigen::Vector3d acc(.2 * std::sin(t), .3, -9.81), omega(.2, -.1 * std::cos(t), .3);
      a.propagateImu(.005, acc, omega, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      b.propagateImu(.005, acc, omega, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      same();
    }
    if (tick % 10 == 0) {
      std::vector<ltv::LtvFeatureObservation> za;
      std::vector<ltv_baseline::LtvFeatureObservation> zb;
      // IDs switch, reorder, disappear and compete for the original capacity.
      for (int j = 0; j < 34; ++j) {
        if ((tick / 10 + j) % 7 == 0) continue;
        int id = (j + tick / 50) % 64;
        Eigen::Vector3d bearing(.03 * j + .01 * t, std::sin(.1 * j), 1);
        ltv::LtvFeatureObservation x; x.feature_id=id;x.normalized_coordinate=bearing;za.push_back(x);
        ltv_baseline::LtvFeatureObservation y; y.feature_id=id;y.normalized_coordinate=bearing;zb.push_back(y);
      }
      a.updateFeatures(t,t,za,Eigen::Matrix3d::Identity(),Eigen::Vector3d(.1,-.03,.02));
      b.updateFeatures(t,t,zb,Eigen::Matrix3d::Identity(),Eigen::Vector3d(.1,-.03,.02));
      same();
    }
  }
  a.reset();b.reset();same();
  std::cout << "PASS 501 physical ticks, x/P/slots/start/valid exact against frozen original core\n";
}
