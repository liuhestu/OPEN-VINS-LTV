#define main previous_independent_main
#include "test_readiness_independent.cpp"
#undef main
int main() {
  try {
    LtvReadinessConfig c;
    c.enabled = true;
    c.initial_unseeded_warmup = true;
    LtvReadiness r(c);
    auto empty = input(0);
    empty.eligible_seeds = empty.observed_features = empty.mature_observed_features = 0;
    empty.geometry_valid = false;
    empty.actual_corrections = 0;
    empty.gravity.setZero();
    auto o = r.update(empty);
    check(o.request_bootstrap && !o.ready_G && !o.ready_V, "initial warmup request");
    auto ack = empty;
    ack.bootstrap_source = "ZERO_DELAYED_START";
    check(rejects([&] { r.acknowledgeBootstrap(ack); }), "warmup source not enforced");
    ack.bootstrap_source = "BOUNDED_INITIAL_ZERO_WARMUP";
    ack.time = ack.observer_time = .05;
    check(rejects([&] { r.acknowledgeBootstrap(ack); }), "warmup ack accepted next event");
    ack.time = ack.observer_time = 0;
    o = r.acknowledgeBootstrap(ack);
    check(o.bootstrap_count == 1 && o.last_bootstrap_time == 0 && !o.ready_V, "initial warmup commit");
    check(rejects([&] { r.acknowledgeBootstrap(ack); }), "warmup repeat ack");
    for (int k = 1; k <= 20; ++k) {
      empty.time = empty.observer_time = .05 * k;
      o = r.update(empty);
      check(!o.ready_G && !o.ready_V && !o.request_bootstrap, "empty warmup reported ready/repeated");
    }
    check(o.state == LtvAvailability::Dormant && o.request_dormant, "warmup not bounded at one second");
    for (int k = 21; k < 600; ++k) {
      o = r.update(input(.05 * k));
      check(!o.request_bootstrap && o.bootstrap_count == 1 && !o.ready_V, "quality cooldown bypass via current core");
    }
    o = r.update(input(30));
    check(o.request_bootstrap && !o.bootstrap_is_physical_recovery, "30 second quality boundary");
    auto second = input(30);
    second.bootstrap_source = "ZERO_DELAYED_START";
    o = r.acknowledgeBootstrap(second);
    check(o.bootstrap_count == 2 && !o.ready_V, "proper quality recovery");
    LtvReadiness faulted(c);
    auto bad = input(0);
    bad.physical_input_fault = true;
    o = faulted.update(bad);
    check(o.physical_fault_count == 1, "physical fault not counted");
    empty.time = empty.observer_time = .05;
    o = faulted.update(empty);
    check(!o.request_bootstrap, "physical fault bypassed supply using initial warmup");
    for (int k = 2; k <= 4; ++k)
      o = faulted.update(input(.05 * k));
    check(o.request_bootstrap && o.bootstrap_is_physical_recovery && o.reason != "initial_bounded_warmup",
          "actual recovery needs reliable supply");
    std::cout << "PASS independent warmup source/pending/same-event/once/1s bound/30s cooldown/physical fault checks\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
