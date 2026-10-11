#include "ltv/diagnostics/LtvLandmarkShadow.h"
#include <iostream>
using namespace ov_msckf;
static void require(bool value) {
  if (!value)
    throw std::runtime_error("landmark shadow geometry contract failed");
}
static void transaction_checks() {
  LtvOptions options;
  options.enabled = options.observer.enable = options.feature_readiness_enabled = options.passive_hardening_enabled = true;
  options.feature_seed_source = "STEREO";
  options.hardening_initial_warmup = true;
  LtvAdapter plain(options), instrumented(options);
  unsigned int calls = 0;
  int previous_updates = 0;
  instrumented.set_prediction_diagnostic([&](const LtvAdapter &a, const ltv::FeaturePipelineContext &ctx, const LtvCalibration &) {
    ++calls;
    require(ctx.time == a.core().snapshot().imu_timestamp);
    require(a.core().snapshot().healthy_camera_updates == previous_updates);
    if (ctx.time > 0)
      require(a.feature_frame().management.time < ctx.time);
  });
  int imu_tick = -2;
  for (int frame = 0; frame < 48; ++frame) {
    const double time = frame * .05;
    while (imu_tick < int(std::llround(time * 200)) + 1) {
      ov_core::ImuData s;
      s.timestamp = (++imu_tick) * .005;
      s.wm.setZero();
      s.am = Eigen::Vector3d(0, 0, 9.81);
      plain.feed_imu(s);
      instrumented.feed_imu(s);
    }
    ltv::FeaturePipelineContext ctx;
    ctx.time = time;
    ctx.version = frame + 1;
    ltv::SeedPose pose;
    pose.t = time;
    ctx.poses = {pose};
    ctx.pose_covariance = .0001 * Eigen::Matrix<double, 6, 6>::Identity();
    ctx.cameras.resize(2);
    ctx.cameras[1].p_BC = Eigen::Vector3d(.2, 0, 0);
    if (frame < 30 || frame >= 36)
      for (size_t id = 0; id < 16; ++id)
        for (int camera = 0; camera < 2; ++camera) {
          ltv::HistoryObservation o;
          o.feature_id = id;
          o.camera_id = camera;
          o.bearing = (Eigen::Vector3d(.03 * id, .1, 3) - ctx.cameras[camera].p_BC).normalized();
          ctx.observations.push_back(o);
        }
    std::vector<LtvBearing> bearings;
    for (const auto &o : ctx.observations)
      if (!o.camera_id)
        bearings.push_back({o.feature_id, o.bearing});
    LtvCalibration calibration;
    const auto a = plain.process(time, bearings, calibration, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ctx.version, &ctx);
    const auto b = instrumented.process(time, bearings, calibration, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ctx.version, &ctx);
    require((plain.core().state().array() == instrumented.core().state().array()).all());
    require((plain.core().covariance().array() == instrumented.core().covariance().array()).all());
    require(plain.feature_ids() == instrumented.feature_ids());
    require(plain.feature_frame().management.retained_ids == instrumented.feature_frame().management.retained_ids);
    require(plain.feature_frame().management.retired_ids == instrumented.feature_frame().management.retired_ids);
    require(a.ready_V == b.ready_V && a.ready_G == b.ready_G && a.epoch == b.epoch);
    previous_updates = plain.core().snapshot().healthy_camera_updates;
  }
  require(calls == 48);
}
int main() {
  ltv::SeedPose a, t;
  a.R_WB = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  t.R_WB = Eigen::AngleAxisd(-.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  a.p_WB = Eigen::Vector3d(1, .5, .2);
  t.p_WB = Eigen::Vector3d(1.3, .4, .1);
  const Eigen::Vector3d world(2, 1, 5);
  const Eigen::Vector3d anchor = a.R_WB.transpose() * (world - a.p_WB);
  const auto body = shadowMainPoint(t, a, anchor);
  require((body - t.R_WB.transpose() * (world - t.p_WB)).norm() < 1e-12);
  ltv::SeedCamera camera;
  camera.R_BC = Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitY()).toRotationMatrix();
  camera.p_BC = Eigen::Vector3d(.2, .05, 0);
  const Eigen::Vector3d ray = camera.R_BC.transpose() * (body - camera.p_BC);
  require(shadowBearingAngle(body, camera, ray.normalized()) < 3e-8);
  require(shadowBearingAngle(body, camera, Eigen::Vector3d(1, 0, 1)) > .1);
  require(!std::isfinite(shadowBearingAngle(camera.p_BC, camera, ray)));
  require(!std::isfinite(shadowBearingAngle(camera.p_BC - camera.R_BC * Eigen::Vector3d::UnitZ(), camera, ray)));
  require(!std::isfinite(shadowBearingAngle(body, camera, Eigen::Vector3d::Zero())));
  // Excluding current and future observations is part of the prediction contract.
  ltv::FeaturePipelineContext context;
  context.time = 2;
  context.execution_pose_index = 2;
  context.cameras = {camera};
  for (int i = 0; i < 3; ++i) {
    ltv::SeedPose pose;
    pose.t = i;
    pose.p_WB = Eigen::Vector3d(.1 * i, 0, 0);
    context.poses.push_back(pose);
  }
  ltv::FeatureTrackHistory history;
  for (int i = 0; i < 4; ++i) {
    ltv::HistorySample sample;
    sample.time = i;
    ltv::HistoryObservation observation;
    observation.feature_id = 1;
    observation.bearing = Eigen::Vector3d(.1, 0, 1).normalized();
    sample.observations.push_back(observation);
    history.samples.push_back(sample);
  }
  const auto past = shadowPastGeometryInput(context, history, 0);
  require(past.observations.size() == 2);
  for (const auto &observation : past.observations)
    require(past.poses[observation.pose_index].t < context.time);
  const auto late_anchor = shadowPastGeometryInput(context, history, 1);
  require(late_anchor.observations.size() == 1);
  transaction_checks();
  std::cout << "landmark shadow geometry/extrinsics/invalid prediction PASS\n";
}
