#include "ltv/LtvAdapter.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace ov_msckf;
void check(bool x, const char *message) { if (!x) throw std::runtime_error(message); }
int main() {
  LtvOptions options;
  options.enabled = true;
  options.observer.min_features = 1;
  options.observer.warmup_camera_updates = 1;
  LtvAdapter adapter(options);
  LtvCalibration calibration;
  std::vector<LtvBearing> bearings = {{42, Eigen::Vector3d(0.1, 0.2, 1)}};
  Eigen::Vector3d am(0.4, -0.2, 9.9), wm(0.06, 0.02, 0.03);
  Eigen::Vector3d ba1(0.1, -0.2, 0.09), bg1(0.01, 0.02, 0.03);
  Eigen::Vector3d ba2(0.2, -0.1, 0.15), bg2(0.015, 0.01, 0.025);
  for (int k = 0; k <= 40; ++k) { ov_core::ImuData x; x.timestamp=k*0.005; x.am=am; x.wm=wm; adapter.feed_imu(x); }
  adapter.process(0, bearings, calibration, ba1, bg1, 1);
  auto frozen = adapter.process(0.02, bearings, calibration, ba1, bg1, 2);
  const auto frozen_velocity = frozen.snapshot.velocity_body;
  auto tampered = frozen; ++tampered.version;
  check(!adapter.claim(tampered) && adapter.claim(frozen) && !adapter.claim(frozen), "version and once claim");
  auto next = adapter.process(0.04, bearings, calibration, ba2, bg2, 3);
  ltv::LtvObserver reference;
  auto config=options.observer; config.enable=true; reference.configure(config); reference.start(0);
  ltv::LtvFeatureObservation feature; feature.feature_id=0; feature.normalized_coordinate=bearings[0].coordinate;
  reference.updateFeatures(0,0,{feature},Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());
  for (int interval=0; interval<2; ++interval) {
    for (int k=0;k<4;++k) reference.propagateImu(0.005,am,wm,interval?ba2:ba1,interval?bg2:bg1);
    double t=(interval+1)*0.02; reference.updateFeatures(t,t,{feature},Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());
  }
  double state_error=(reference.state()-adapter.core().state()).norm();
  double p_error=(reference.covariance()-adapter.core().covariance()).norm();
  check(state_error<1e-10 && p_error<1e-8,"bias applied only to its next interval");
  check(next.integrated_steps==8 && std::abs(next.integrated_seconds-0.04)<1e-12,"future cache not integrated");
  check((frozen.snapshot.velocity_body-frozen_velocity).norm()==0 && !adapter.claim(frozen),"frozen copy and stale claim");
  LtvAdapter unix_adapter(options); calibration.offset=0.003; const double base=1400000000.;
  for (int k=0;k<=40;++k) { ov_core::ImuData x; x.timestamp=base+k*0.005; x.am=am; x.wm=wm; unix_adapter.feed_imu(x); }
  auto first=unix_adapter.process(base+0.001,bearings,calibration,ba1,bg1,1);
  auto second=unix_adapter.process(base+0.021,bearings,calibration,ba2,bg2,2);
  check(first.available && second.available,"Unix-time fractional boundary");
  auto bad_key=second; ++bad_key.camera_ns;
  check(!unix_adapter.claim(bad_key) && unix_adapter.claim(second),"canonical packet key near Unix epoch");
  std::cout << "bias_interval_state_error=" << state_error << " P_error=" << p_error << " Unix_epoch_claim PASS\n";
}
