#include "ltv/LtvReadiness.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace ltv;
void check(bool x, const char *m) {
  if (!x)
    throw std::runtime_error(m);
}
LtvReadinessInput input(double t) {
  LtvReadinessInput i;
  i.time = i.observer_time = t;
  i.geometry_valid = true;
  i.eligible_seeds = i.observed_features = i.mature_observed_features = 15;
  i.observer_started = i.observer_finite = i.correction_diagnostics_valid = true;
  i.gravity = Eigen::Vector3d(0, 0, -9.81);
  i.actual_corrections = 20;
  return i;
}
template <class F> bool rejects(F f) {
  try {
    f();
    return false;
  } catch (const std::invalid_argument &) {
    return true;
  }
}
void request(LtvReadiness &r) {
  for (int k = 0; k < 3; k++) {
    auto i = input(k * .05);
    i.observer_started = false;
    auto o = r.update(i);
    check(o.request_bootstrap == (k == 2), "pending confirmation");
  }
}
int main() {
  try {
    LtvReadinessConfig c;
    c.enabled = true;
    LtvReadiness r(c);
    request(r);
    auto ack = input(.1);
    ack.bootstrap_source = "ZERO_START";
    auto wrong = ack;
    wrong.gravity.x() = std::numeric_limits<double>::infinity();
    check(rejects([&] { r.acknowledgeBootstrap(wrong); }), "infinite gravity ack accepted");
    auto o = r.acknowledgeBootstrap(ack);
    check(o.bootstrap_count == 1 && !o.ready_G, "same-event ack semantics");
    check(rejects([&] { r.acknowledgeBootstrap(ack); }), "duplicate ack accepted");
    for (int k = 1; k <= 20; k++) {
      o = r.update(input(.1 + k * .05));
      check(o.ready_V == (k == 20), "ack counted as observation");
    }
    auto bad = input(1.15);
    bad.observed_features = 0;
    o = r.update(bad);
    check(!o.ready_G && !o.ready_V, "empty event did not revoke");
    o = r.update(input(1.20));
    check(!o.ready_V, "recovery skipped health confirmation");
    LtvReadiness cancelled(c);
    request(cancelled);
    auto delayed = input(.15);
    delayed.bootstrap_committed = true;
    delayed.bootstrap_source = "ZERO_START";
    delayed.eligible_seeds = 0;
    delayed.geometry_valid = false;
    check(rejects([&] { cancelled.update(delayed); }), "delayed commit accepted after current seed supply disappeared");
    // Failed ack must not consume the input; explicit empty event cancels pending.
    delayed.bootstrap_committed = false;
    o = cancelled.update(delayed);
    check(o.accepted_time && o.bootstrap_count == 0 && !o.request_bootstrap, "rejected commit mutated clock/count");
    std::cout << "PASS independent pending/ack/finite/reconfirmation/withdrawn-supply checks\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
