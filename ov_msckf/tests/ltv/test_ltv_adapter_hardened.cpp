#include "ltv/observer/LtvAdapter.h"
#include <iostream>
#include <stdexcept>
using namespace ov_msckf;
static void require(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}
struct Harness {
  LtvAdapter adapter;
  LtvCalibration cal;
  int imu_tick = -2;
  double last_imu = -.01;
  explicit Harness(const LtvOptions &options) : adapter(options) { cal.p_BC = Eigen::Vector3d(.02, 0, 0); }
  void feed_to(double t) {
    const int end = static_cast<int>(std::llround(t * 200)) + 1;
    while (imu_tick < end) {
      ++imu_tick;
      ov_core::ImuData s;
      s.timestamp = imu_tick * .005;
      s.am = Eigen::Vector3d(0, 0, 9.81);
      s.wm.setZero();
      adapter.feed_imu(s);
      last_imu = s.timestamp;
    }
  }
  ltv::FeaturePipelineContext context(int frame, bool visible) {
    ltv::FeaturePipelineContext ctx;
    ctx.time = frame * .05;
    ctx.version = frame + 1;
    ltv::SeedPose pose;
    pose.t = ctx.time;
    ctx.poses = {pose};
    ctx.pose_covariance = .0001 * Eigen::Matrix<double, 6, 6>::Identity();
    ltv::SeedCamera left, right;
    left.p_BC = cal.p_BC;
    right.p_BC = cal.p_BC + Eigen::Vector3d(.2, 0, 0);
    ctx.cameras = {left, right};
    if (visible)
      for (size_t id = 0; id < 16; ++id)
        for (int camera = 0; camera < 2; ++camera) {
          ltv::HistoryObservation o;
          o.feature_id = id;
          o.camera_id = camera;
          o.bearing = (Eigen::Vector3d(.03 * id, .1, 3) - ctx.cameras[camera].p_BC).normalized();
          ctx.observations.push_back(o);
        }
    return ctx;
  }
  LtvFrame frame(int tick, bool visible) {
    const double t = tick * .05;
    feed_to(t);
    auto ctx = context(tick, visible);
    std::vector<LtvBearing> bearings;
    for (const auto &o : ctx.observations)
      if (o.camera_id == 0)
        bearings.push_back({o.feature_id, o.bearing});
    auto f = adapter.process(t, bearings, cal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ctx.version, &ctx);
    if (f.available)
      require(adapter.claim(f), "unique available frame claim");
    return f;
  }
  void duplicate_imu() {
    ov_core::ImuData s;
    s.timestamp = last_imu;
    s.am = Eigen::Vector3d(0, 0, 9.81);
    s.wm.setZero();
    adapter.feed_imu(s);
  }
};
int main() {
  try {
    LtvOptions options;
    options.enabled = true;
    options.feature_readiness_enabled = true;
    options.feature_seed_source = "STEREO";
    options.passive_hardening_enabled = true;
    Harness h(options);
    LtvFrame f;
    auto startup = h.adapter.pause(0, "uninitialized", 0);
    require(startup.health.physical_fault_count == 0, "startup unavailable is not a physical sensor fault");
    for (int k = 0; k < 20; ++k) {
      f = h.frame(k, false);
      require(!h.adapter.core().started() && f.integrated_steps == 0 && !f.raw_current && !f.ready_G && !f.ready_V,
              "empty collecting must not integrate or publish raw");
      require(h.adapter.core().state().size() == 6 && h.adapter.core().state().norm() == 0, "empty collecting no hidden state drift");
    }
    for (int k = 20; k < 24; ++k) {
      f = h.frame(k, true);
      require(!h.adapter.core().started() && !f.raw_current, "history plus three supply confirmations required");
      require(h.adapter.feature_frame().management.births.empty(), "prebootstrap preview cannot consume seed");
    }
    f = h.frame(24, true);
    require(h.adapter.core().started() && f.raw_current, "third qualified supply commits observer");
    require(f.health.bootstrap_count == 1 && f.health.last_bootstrap_source == "ZERO_DELAYED_START" && f.health.physical_fault_count == 0,
            "explicit cold bootstrap source");
    require(f.integrated_steps == 0 && f.snapshot.velocity_body.norm() == 0 && f.snapshot.gravity_body.norm() == 0,
            "no discarded prebootstrap propagation");
    require(h.adapter.feature_frame().management.births.size() == 16 && f.snapshot.state_features == 16,
            "all seed/slots commit atomically");
    for (const auto &birth : h.adapter.feature_frame().management.births) {
      const auto id = h.adapter.feature_ids().at(birth.feature_id);
      const int slot = h.adapter.core().slotForFeature(id);
      require(slot >= 0 && (h.adapter.core().state().segment<3>(3 * slot) - birth.seed.landmark_B).norm() < 1e-10,
              "seed written before first zero-length correction");
    }
    auto x = h.adapter.core().state();
    auto P = h.adapter.core().covariance();
    auto duplicate = h.frame(24, true);
    require(!duplicate.raw_current && !duplicate.ready_G && !duplicate.ready_V && !duplicate.available,
            "duplicate packet cannot publish current result");
    require(!duplicate.health.accepted_time && !duplicate.health.observer_valid && !duplicate.health.ready_G && !duplicate.health.ready_V &&
                !duplicate.health.joint_ready && !duplicate.health.grace_G && !duplicate.health.grace_V &&
                duplicate.health.reason == "duplicate_or_backward_camera",
            "duplicate return diagnostics cannot inherit accepted readiness");
    require((x - h.adapter.core().state()).norm() == 0 && (P - h.adapter.core().covariance()).norm() == 0,
            "duplicate packet preserves full x/P");
    for (int k = 25; k < 29; ++k) {
      f = h.frame(k, true);
      require(h.adapter.feature_frame().management.births.empty() && f.health.bootstrap_count == 1, "no repeated seed");
    }
    require(f.integrated_steps > 0, "started observer consumes causal IMU");
    bool dormant = false;
    for (int k = 29; k < 54; ++k) {
      f = h.frame(k, false);
      require(!f.ready_G && !f.ready_V, "first missing camera revokes ready");
      if (f.health.state == ltv::LtvAvailability::Dormant)
        dormant = true;
    }
    require(dormant && !f.raw_current && !h.adapter.core().started(), "prolonged loss stops observer with explicit dormant");
    const auto stopped_steps = f.integrated_steps;
    for (int k = 54; k < 70; ++k) {
      f = h.frame(k, true);
      require(!f.raw_current && !h.adapter.core().started() && f.health.bootstrap_count == 1, "quality restart is cooling down");
      require(f.integrated_steps == stopped_steps, "dormant does not continue blind propagation");
    }
    h.duplicate_imu();
    f = h.frame(70, true);
    require(!f.raw_current && !f.ready_G && !f.ready_V && f.health.physical_fault_count == 1,
            "actual duplicate IMU marks physical fault separately");
    for (int k = 71; k < 76; ++k)
      f = h.frame(k, true);
    require(f.health.bootstrap_count == 2 && h.adapter.core().started() && f.raw_current,
            "physical input recovery can restart within quality cooldown");
    require(f.health.last_bootstrap_time < options.hardening_readiness.quality_bootstrap_interval,
            "test physically distinguishes cooldown bypass");
    require(!f.ready_G && !f.ready_V, "physical recovery cannot inherit previous readiness");
    LtvOptions warm_options = options;
    warm_options.hardening_initial_warmup = true;
    LtvOptions previous_options = options;
    previous_options.passive_hardening_enabled = false;
    Harness previous(previous_options), warm(warm_options);
    for (int k = 0; k <= 80; ++k) {
      previous.frame(k, true);
      auto current = warm.frame(k, true);
      const auto &a = previous.adapter.core();
      const auto &b = warm.adapter.core();
      require(a.state().size() == b.state().size() && a.covariance().rows() == b.covariance().rows(),
              "initial warmup preserves baseline complete state dimensions");
      require((a.state().array() == b.state().array()).all() && (a.covariance().array() == b.covariance().array()).all(),
              "initial warmup preserves exact baseline full state and cross covariance");
      require(previous.adapter.feature_ids() == warm.adapter.feature_ids(), "initial warmup preserves original feature mapping");
      for (const auto &entry : previous.adapter.feature_ids())
        require(a.slotForFeature(entry.second) == b.slotForFeature(entry.second), "initial warmup preserves every feature slot");
      require(current.health.bootstrap_count == 1 && current.health.physical_fault_count == 0 &&
                  current.health.last_bootstrap_source == "BOUNDED_INITIAL_ZERO_WARMUP",
              "initial finite warmup acknowledged once without changing core");
    }
    Harness empty_warm(warm_options);
    uint64_t dormant_boundary_steps = 0;
    for (int k = 0; k <= 20; ++k) {
      auto current = empty_warm.frame(k, false);
      require(!current.ready_G && !current.ready_V && current.health.bootstrap_count == 1, "empty warmup never ready and occurs once");
      if (k == 20) {
        require(current.health.state == ltv::LtvAvailability::Dormant && current.health.request_dormant && !current.ready_G &&
                    !current.ready_V,
                "empty warmup requests dormancy at exactly one second");
        dormant_boundary_steps = current.integrated_steps;
      }
    }
    for (int k = 21; k < 40; ++k) {
      auto current = empty_warm.frame(k, true);
      require(!empty_warm.adapter.core().started() && !current.raw_current && current.health.bootstrap_count == 1,
              "next event resets dormant core before publishing another current state");
      require(current.integrated_steps == dormant_boundary_steps,
              "no IMU integration after one-second dormant request, including the next event");
    }
    Harness fault_before_warm(warm_options);
    fault_before_warm.feed_to(0);
    fault_before_warm.duplicate_imu();
    auto physical = fault_before_warm.frame(0, false);
    require(physical.health.physical_fault_count == 1 && physical.health.bootstrap_count == 0,
            "physical fault before first start remains a real fault");
    for (int k = 1; k <= 5; ++k) {
      physical = fault_before_warm.frame(k, false);
      require(!fault_before_warm.adapter.core().started() && physical.health.bootstrap_count == 0 && !physical.raw_current,
              "physical recovery cannot sneak into initial empty warmup");
    }
    std::cout << "hardened adapter PASS real pipeline collection/atomic seed/duplicate/dormant/cooldown/physical recovery\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
