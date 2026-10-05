#include "ltv/observer/LtvReadiness.h"
#include <cmath>
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
    for (bool grace : {false, true}) {
      LtvReadinessConfig base_config;
      base_config.enabled = base_config.initial_unseeded_warmup = base_config.preserve_constrained_state = true;
      base_config.ready_soft_grace = grace;
      auto short_config = base_config;
      short_config.experimental_velocity_confirm_frames = 10;
      LtvReadiness baseline(base_config), shorter(short_config);
      for (auto *policy : {&baseline, &shorter}) {
        auto initial = healthy(0);
        initial.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
        policy->update(initial);
        policy->acknowledgeBootstrap(initial);
      }
      for (int k = 1; k <= 20; ++k) {
        auto a = baseline.update(healthy(.05 * k));
        auto b = shorter.update(healthy(.05 * k));
        require(a.ready_V == (k == 20) && b.ready_V == (k >= 10), "only V confirmation changes from 20 to 10");
        require(a.ready_G == b.ready_G && b.ready_G == (k == 20), "G confirmation remains 20");
      }
      auto thin = healthy(1.05);
      thin.observed_features = 14;
      auto bad = shorter.update(thin);
      require(!bad.ready_V && !bad.grace_V, "thin supply cannot be released by shorter confirmation");
      for (int k = 1; k <= 10; ++k) {
        auto b = shorter.update(healthy(1.05 + .05 * k));
        require(b.ready_V == (k == 10), "recovery still requires ten genuinely good frames");
      }
      auto gap = shorter.update(healthy(2.0));
      require(!gap.ready_V && !gap.grace_V, "time gap clears short confirmation and grace");
      for (int k = 1; k < 10; ++k) {
        auto invalid = healthy(2.0 + .05 * k);
        invalid.correction_diagnostics_valid = false;
        require(!shorter.update(invalid).ready_V, "invalid diagnostics never accumulate short confirmation");
      }
      for (int which = 0; which < 7; ++which) {
        LtvReadiness policy(short_config);
        auto initial = healthy(0);
        initial.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
        policy.update(initial);
        policy.acknowledgeBootstrap(initial);
        for (int k = 1; k <= 20; ++k)
          policy.update(healthy(.05 * k));
        auto invalid = healthy(1.05);
        if (which == 0)
          invalid.mature_observed_features = 14;
        if (which == 1)
          invalid.actual_corrections = 19;
        if (which == 2)
          invalid.prediction_angle_p95_rad = .041;
        if (which == 3)
          invalid.velocity_correction_rate = 1.01;
        if (which == 4)
          invalid.gravity_correction_rate = 2.01;
        if (which == 5)
          invalid.gravity.setZero();
        if (which == 6)
          invalid.observer_time = 1.0;
        auto b = policy.update(invalid);
        require(!b.ready_V && !b.grace_V, "short confirmation cannot bypass existing hard health or time boundaries");
      }
    }
    auto invalid_experiment = LtvReadinessConfig();
    invalid_experiment.experimental_velocity_confirm_frames = 11;
    bool invalid_length_rejected = false;
    try {
      LtvReadiness invalid(invalid_experiment);
    } catch (const std::invalid_argument &) {
      invalid_length_rejected = true;
    }
    require(invalid_length_rejected, "only the predeclared ten-frame alternative is allowed");
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
    LtvReadinessConfig preserve_config = warm_config;
    preserve_config.preserve_constrained_state = true;
    LtvReadiness preserve(preserve_config);
    auto preserved = healthy(0);
    preserved.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
    require(preserve.update(preserved).request_bootstrap, "R3 initial warmup keeps atomic contract");
    preserve.acknowledgeBootstrap(preserved);
    for (int k = 1; k <= 200; ++k) {
      preserved = healthy(.05 * k);
      preserved.observed_features = preserved.mature_observed_features = 1 + k % 14;
      o = preserve.update(preserved);
      require(o.state == LtvAvailability::Degraded && o.observer_valid && !o.ready_G && !o.ready_V && !o.request_dormant,
              "ten seconds of thin current observations preserve state without ready");
    }
    for (int k = 1; k <= 20; ++k) {
      o = preserve.update(healthy(10 + .05 * k));
      require(o.ready_V == (k == 20), "thin-state recovery requires full health reconfirmation");
    }
    for (int k = 0; k <= 20; ++k) {
      preserved = healthy(11.05 + .05 * k);
      preserved.observed_features = preserved.mature_observed_features = 0;
      o = preserve.update(preserved);
      require(!o.ready_G && !o.ready_V, "first empty packet revokes readiness");
      require(o.request_dormant == (k == 20), "empty observation clock expires at one second, not earlier");
    }
    require(o.state == LtvAvailability::Dormant, "zero observations eventually dormant");
    auto physical_preserve = healthy(12.1);
    physical_preserve.physical_input_fault = true;
    o = preserve.update(physical_preserve);
    require(o.physical_fault_count == 1 && !o.ready_G && !o.observer_valid, "thin-state option never hides physical input fault");
    for (int k = 0; k < 3; ++k) {
      preserved = healthy(12.15 + .05 * k);
      preserved.observer_started = false;
      o = preserve.update(preserved);
    }
    require(o.request_bootstrap && o.bootstrap_is_physical_recovery, "physical repair still requires confirmed seed supply");
    for (int axis = 0; axis < 2; ++axis) {
      LtvReadiness bounded_norm(preserve_config);
      auto normal = healthy(0);
      normal.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
      bounded_norm.update(normal);
      bounded_norm.acknowledgeBootstrap(normal);
      auto excessive = healthy(.05);
      excessive.observed_features = 1;
      if (axis == 0)
        excessive.velocity = Eigen::Vector3d(100.01, 0, 0);
      else
        excessive.gravity = Eigen::Vector3d(0, 0, -50.01);
      o = bounded_norm.update(excessive);
      require(o.request_dormant && !o.observer_valid && o.reason == "observer_norm_guard", "finite extreme state stops immediately");
    }
    LtvReadiness invalid_geometry(preserve_config);
    auto initial = healthy(0);
    initial.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
    invalid_geometry.update(initial);
    invalid_geometry.acknowledgeBootstrap(initial);
    for (int k = 1; k <= 21; ++k) {
      auto bad_geometry = healthy(.05 * k);
      bad_geometry.observed_features = 14;
      bad_geometry.geometry_valid = false;
      o = invalid_geometry.update(bad_geometry);
    }
    require(o.request_dormant && o.state == LtvAvailability::Dormant, "positive count without valid geometry does not preserve state");
    LtvReadiness stale_preserved(preserve_config);
    stale_preserved.update(initial);
    stale_preserved.acknowledgeBootstrap(initial);
    auto stale_thin = healthy(.05);
    stale_thin.observed_features = 1;
    stale_thin.observer_time = 0;
    o = stale_preserved.update(stale_thin);
    require(o.request_dormant && !o.observer_valid, "thin count cannot preserve stale observer");
    LtvReadinessConfig grace_config = warm_config;
    grace_config.ready_soft_grace = true;
    auto primed = [&](double origin) {
      LtvReadiness policy(grace_config);
      auto i = healthy(origin);
      i.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
      policy.update(i);
      policy.acknowledgeBootstrap(i);
      for (int k = 1; k <= 20; ++k) {
        auto result = policy.update(healthy(origin + k * .05));
        require(result.ready_G == (k == 20), "grace preserves original twenty-frame startup confirmation");
      }
      return policy;
    };
    for (double origin : {0.0, 1.4e9}) {
      auto base = primed(origin);
      const double last = origin + 1.;
      const double deadline = std::nextafter(last + .10, std::numeric_limits<double>::infinity());
      auto soft = healthy(last + .05);
      soft.prediction_angle_p95_rad = .04;
      auto bounded_time = base;
      o = bounded_time.update(soft);
      require(o.ready_G && o.ready_V && o.grace_G && o.grace_V && o.soft_failure_frames_G == 1 && o.last_strict_good_G == last,
              "first soft event retains but never refreshes last strict good");
      soft.time = soft.observer_time = deadline;
      o = bounded_time.update(soft);
      require(o.grace_G && o.soft_failure_frames_G == 2, "predeclared upward one-ULP deadline included");
      auto beyond = base;
      soft.time = soft.observer_time = std::nextafter(deadline, std::numeric_limits<double>::infinity());
      o = beyond.update(soft);
      require(!o.ready_G && !o.ready_V, "next representable time beyond frozen deadline excluded");
      auto packets = base;
      for (int k = 1; k <= 3; ++k) {
        soft.time = soft.observer_time = last + .02 * k;
        o = packets.update(soft);
        require(o.ready_G == (k < 3), "third soft event revokes even within sixty milliseconds");
      }
      for (int k = 1; k <= 20; ++k) {
        o = packets.update(healthy(last + .1 + k * .05));
        require(o.ready_G == (k == 20), "revoked grace needs twenty new strict-good events");
      }
    }
    auto independent_grace = primed(0);
    auto v_only = healthy(1.05);
    v_only.velocity_correction_rate = .75;
    o = independent_grace.update(v_only);
    require(o.ready_G && !o.grace_G && o.ready_V && o.grace_V && o.last_strict_good_G == 1.05 && o.last_strict_good_V == 1.,
            "G strict good and V soft grace maintain independent clocks");
    v_only.time = v_only.observer_time = 1.1;
    v_only.velocity_correction_rate = 1.01;
    o = independent_grace.update(v_only);
    require(o.ready_G && !o.ready_V && o.soft_failure_frames_V == 0, "V excess cannot revoke independent healthy G");
    v_only = healthy(1.15);
    v_only.gravity_correction_rate = 1.5;
    o = independent_grace.update(v_only);
    require(o.ready_G && o.grace_G && !o.ready_V, "G hold cannot resurrect already revoked V");
    for (int which = 0; which < 7; ++which) {
      auto hard_exit = primed(0);
      auto bad = healthy(1.05);
      bad.prediction_angle_p95_rad = .03;
      hard_exit.update(bad);
      bad.time = bad.observer_time = 1.1;
      if (which == 0)
        bad.observed_features = 14;
      if (which == 1)
        bad.mature_observed_features = 14;
      if (which == 2)
        bad.correction_diagnostics_valid = false;
      if (which == 3)
        bad.gravity.setZero();
      if (which == 4)
        bad.velocity_correction_rate = -1;
      if (which == 5)
        bad.physical_input_fault = true;
      if (which == 6)
        bad.observer_time = 1.0;
      o = hard_exit.update(bad);
      require(!o.ready_G && !o.ready_V && !o.grace_G && !o.grace_V && o.soft_failure_frames_G == 0 && o.soft_failure_frames_V == 0,
              "hard invalidity cannot be hidden by active soft grace");
    }
    auto refreshed_grace = primed(0);
    auto slight = healthy(1.05);
    slight.prediction_angle_p95_rad = .03;
    refreshed_grace.update(slight);
    o = refreshed_grace.update(healthy(1.1));
    require(o.ready_G && !o.grace_G && o.soft_failure_frames_G == 0 && o.last_strict_good_G == 1.1,
            "actual good clears grace and updates last-good time");
    refreshed_grace.reset();
    o = refreshed_grace.update(healthy(0));
    require(!o.ready_G && o.last_strict_good_G == -1 && o.soft_failure_frames_G == 0, "explicit reset clears all grace state");
    std::cout << "readiness PASS collecting/atomic bootstrap/confirmation/revocation/dormancy/cooldown/physical faults\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
