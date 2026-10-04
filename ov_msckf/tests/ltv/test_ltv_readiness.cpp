#include "ltv/LtvReadiness.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace ltv;
static void require(bool v, const char *m) {
  if (!v)
    throw std::runtime_error(m);
}
static LtvReadinessInput healthy(double t) {
  LtvReadinessInput i;
  i.time = i.observer_time = t;
  i.eligible_seeds = i.observed_features = i.mature_observed_features = 15;
  i.geometry_valid = i.observer_started = i.observer_finite = i.correction_diagnostics_valid = true;
  i.actual_corrections = 20;
  i.gravity = Eigen::Vector3d(0, 0, -9.81);
  i.velocity = Eigen::Vector3d(1, 0, 0);
  i.prediction_angle_p95_rad = .001;
  i.velocity_correction_rate = .01;
  i.gravity_correction_rate = .01;
  return i;
}
int main() {
  try {
    LtvReadiness off;
    auto o = off.update(healthy(0));
    require(o.state == LtvAvailability::Disabled && !o.request_bootstrap && !o.ready_V, "default off");
    LtvReadinessConfig c;
    c.enabled = true;
    LtvReadiness r(c);
    for (int k = 0; k < 3; ++k) {
      auto i = healthy(.05 * k);
      i.observer_started = false;
      i.observer_finite = false;
      o = r.update(i);
      require(!o.ready_V && !o.observer_valid, "collecting cannot invent current output");
      require(o.request_bootstrap == (k == 2), "three current supply confirmations");
    }
    auto pending = healthy(.11);
    o = r.update(pending);
    require(!o.request_bootstrap && !o.ready_V && o.state == LtvAvailability::Recovering, "request alone cannot start old state");
    auto commit = healthy(.15);
    commit.bootstrap_source = "ZERO_START";
    r.update(commit);
    o = r.acknowledgeBootstrap(commit);
    require(o.bootstrap_count == 1 && !o.ready_V, "atomic commit starts health confirmation");
    for (int k = 1; k <= 20; ++k)
      o = r.update(healthy(.15 + .05 * k));
    require(o.ready_G && o.ready_V && o.joint_ready, "healthy confirmations become ready");
    auto unready = healthy(1.20);
    unready.observed_features = unready.mature_observed_features = 0;
    unready.geometry_valid = false;
    o = r.update(unready);
    require(!o.ready_G && !o.ready_V && o.state == LtvAvailability::Degraded, "first bad packet revokes ready");
    unready.time = unready.observer_time = 2.21;
    o = r.update(unready);
    require(o.state == LtvAvailability::Dormant && o.request_dormant && !o.observer_valid, "finite insufficiency enters dormant");
    for (int k = 0; k < 4; ++k) {
      auto i = healthy(2.25 + .05 * k);
      i.observer_started = false;
      i.observer_finite = false;
      o = r.update(i);
      require(!o.request_bootstrap, "quality cooldown");
    }
    auto badcommit = healthy(3);
    badcommit.bootstrap_committed = true;
    badcommit.bootstrap_source = "ZERO_START";
    bool rejected = false;
    try {
      r.update(badcommit);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "early quality commit refused");
    auto fault = healthy(3);
    fault.physical_input_fault = true;
    o = r.update(fault);
    require(o.accepted_time && o.physical_fault_count == 1 && !o.ready_G, "rejected transaction did not consume event time");
    fault.time = fault.observer_time = 3.05;
    o = r.update(fault);
    require(o.physical_fault_count == 1, "continuous fault does not reset storm");
    for (int k = 0; k < 3; ++k) {
      auto i = healthy(3.1 + .05 * k);
      i.observer_started = false;
      i.observer_finite = false;
      o = r.update(i);
    }
    require(o.request_bootstrap && o.bootstrap_is_physical_recovery, "physical repair explicitly distinct from quality cooldown");
    commit = healthy(3.25);
    commit.bootstrap_was_physical_recovery = true;
    commit.bootstrap_source = "PHYSICAL_ZERO_START";
    r.update(commit);
    o = r.acknowledgeBootstrap(commit);
    require(o.bootstrap_count == 2 && !o.ready_V, "physical recovery requires reconfirmation");
    auto zeroeta = healthy(3.3);
    zeroeta.gravity.setZero();
    o = r.update(zeroeta);
    require(!o.ready_G && !o.ready_V, "zero eta never ready even with mature pool");
    auto hugecorrection = healthy(3.35);
    hugecorrection.velocity_correction_rate = 100;
    o = r.update(hugecorrection);
    require(!o.ready_V, "correction rate is not physical acceleration");
    auto nonfinite = healthy(3.4);
    nonfinite.velocity.x() = std::numeric_limits<double>::quiet_NaN();
    o = r.update(nonfinite);
    require(!o.observer_valid && !o.ready_V && o.state == LtvAvailability::Dormant, "nonfinite observer invalidates current state");
    require(!r.update(nonfinite).accepted_time, "duplicate event is rejected");
    auto stale = healthy(3.45);
    stale.observer_time = 3.2;
    o = r.update(stale);
    require(!o.observer_valid && !o.ready_G, "stale raw timestamp never healthy");
    unsigned int recovery_requests = 0;
    for (int k = 0; k < 3; ++k) {
      auto i = healthy(30.2 + .05 * k);
      i.observer_started = false;
      i.observer_finite = false;
      o = r.update(i);
      recovery_requests += o.request_bootstrap;
    }
    require(recovery_requests == 1 && !o.bootstrap_is_physical_recovery, "quality cooldown expires causally without request storm");
    LtvReadiness cold(c);
    for (int k = 0; k < 200; ++k) {
      auto i = healthy(.05 * k);
      i.eligible_seeds = 0;
      i.observer_started = false;
      o = cold.update(i);
      require(!o.request_bootstrap && !o.ready_V, "no supply stays collecting");
    }
    LtvReadiness atomic(c);
    auto unsolicited = healthy(0);
    unsolicited.bootstrap_committed = true;
    unsolicited.bootstrap_source = "UNSOLICITED";
    rejected = false;
    try {
      atomic.update(unsolicited);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "unsolicited first bootstrap rejected");
    for (int k = 0; k < 3; ++k) {
      auto i = healthy(.05 * k);
      i.observer_started = false;
      o = atomic.update(i);
    }
    require(o.request_bootstrap, "rejected ack did not consume time");
    auto ack = healthy(.1);
    ack.bootstrap_source = "ZERO_START";
    auto badvector = ack;
    badvector.velocity.x() = std::numeric_limits<double>::quiet_NaN();
    rejected = false;
    try {
      atomic.acknowledgeBootstrap(badvector);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "ack finite flag cannot hide NaN vector");
    auto wrongkind = ack;
    wrongkind.bootstrap_was_physical_recovery = true;
    rejected = false;
    try {
      atomic.acknowledgeBootstrap(wrongkind);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "ack must match pending physical/quality nature");
    o = atomic.acknowledgeBootstrap(ack);
    require(o.bootstrap_count == 1 && !o.ready_V && o.accepted_time, "same event ack consumes no extra observation");
    rejected = false;
    try {
      atomic.acknowledgeBootstrap(ack);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "same event duplicate ack rejected");
    for (int k = 1; k < 20; ++k)
      o = atomic.update(healthy(.1 + .05 * k));
    require(!o.ready_V, "ack not counted as healthy observation");
    o = atomic.update(healthy(1.1));
    require(o.ready_V, "twenty actual postack confirmations");
    auto unauthorized = healthy(32);
    unauthorized.bootstrap_committed = true;
    unauthorized.bootstrap_source = "UNSOLICITED";
    rejected = false;
    try {
      atomic.update(unauthorized);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "tracking cannot rewrite after cooldown without pending request");
    o = atomic.update(healthy(1.5));
    require(!o.ready_V, "missing event interval breaks ready confirmation");
    LtvReadiness cancelled(c);
    for (int k = 0; k < 3; ++k) {
      auto i = healthy(.05 * k);
      i.observer_started = false;
      o = cancelled.update(i);
    }
    auto lost_delayed = healthy(.15);
    lost_delayed.eligible_seeds = 0;
    lost_delayed.geometry_valid = false;
    lost_delayed.bootstrap_committed = true;
    lost_delayed.bootstrap_source = "ZERO_START";
    rejected = false;
    try {
      cancelled.update(lost_delayed);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "pending cannot accept delayed ack on lost current supply");
    auto empty = healthy(.15);
    empty.eligible_seeds = 0;
    empty.observer_started = false;
    cancelled.update(empty);
    auto late = healthy(.15);
    late.bootstrap_source = "ZERO_START";
    rejected = false;
    try {
      cancelled.acknowledgeBootstrap(late);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "cancelled supply request cannot commit");
    LtvReadiness gap_policy(c);
    for (int k = 0; k < 5; ++k) {
      auto i = healthy(k * 1.0);
      i.observer_started = false;
      o = gap_policy.update(i);
      require(!o.request_bootstrap, "sparse frames cannot accumulate supply confirmations");
    }
    LtvReadinessConfig warm_config = c;
    warm_config.initial_unseeded_warmup = true;
    LtvReadiness warm(warm_config);
    auto unseeded = healthy(0);
    unseeded.eligible_seeds = unseeded.observed_features = unseeded.mature_observed_features = 0;
    unseeded.geometry_valid = false;
    unseeded.gravity.setZero();
    unseeded.velocity.setZero();
    unseeded.actual_corrections = 0;
    auto not_started = unseeded;
    not_started.observer_started = false;
    require(!warm.update(not_started).request_bootstrap, "warmup cannot invent a core");
    unseeded.time = unseeded.observer_time = .05;
    o = warm.update(unseeded);
    require(o.request_bootstrap && o.reason == "initial_bounded_warmup" && !o.ready_G && !o.ready_V,
            "current empty core may request initial bounded warmup");
    unseeded.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
    o = warm.acknowledgeBootstrap(unseeded);
    require(o.bootstrap_count == 1 && o.last_bootstrap_time == .05 && o.physical_fault_count == 0 && !o.ready_V,
            "warmup uses normal atomic ack and quality clock");
    for (int k = 1; k <= 20; ++k) {
      unseeded.time = unseeded.observer_time = .05 + k * .05;
      o = warm.update(unseeded);
      require(!o.request_bootstrap && !o.ready_G && !o.ready_V, "empty warmup never ready or repeated");
    }
    require(o.state == LtvAvailability::Dormant && o.request_dormant, "empty warmup bounded to one second from ack");
    for (int k = 0; k < 3; ++k) {
      auto supplied = healthy(1.10 + .05 * k);
      o = warm.update(supplied);
      require(!o.request_bootstrap && o.bootstrap_count == 1, "finite restarted core cannot bypass quality cooldown");
    }
    for (int k = 0; k < 3; ++k)
      o = warm.update(healthy(30.05 + .05 * k));
    require(o.request_bootstrap && o.reason == "atomic_bootstrap_required", "later recovery requires ordinary confirmed supply");
    LtvReadiness default_warm(c);
    unseeded.time = unseeded.observer_time = 0;
    require(!default_warm.update(unseeded).request_bootstrap, "default R1 finite empty core still waits for seed supply");
    std::cout << "readiness PASS collecting/atomic bootstrap/confirmation/revocation/dormancy/cooldown/physical faults\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
