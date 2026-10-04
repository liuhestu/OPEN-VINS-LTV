#include "LtvEventReceipt.h"
#include <cassert>
#include <iostream>
#include <sstream>

template <class F> void rejects(F f) {
  bool rejected = false;
  try {
    f();
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  assert(rejected);
}

int main() {
  const int64_t ns = 1450000000000000123LL;
  const double t = ns * 1e-9;
  assert(std::llround(t * 1e9) != ns);
  LtvEventReceipt receipt;
  receipt.begin(ns, ns, t);
  receipt.complete(ns, 0, false, 0, 0); // Startup still consumes exactly one receipt.
  assert(receipt.id == 1 && receipt.camera_ns == ns);
  rejects([&] { receipt.complete(ns, 0, false, 0, 0); });
  receipt.begin(ns + 50000000, ns + 50000000, (ns + 50000000) * 1e-9);
  receipt.complete(ns + 50000000, (ns + 50000000) * 1e-9, true, 1, 1);
  assert(receipt.id == 2);
  receipt.begin(ns + 100000000, ns + 100000000, (ns + 100000000) * 1e-9);
  receipt.complete(ns + 100000000, (ns + 100000000) * 1e-9, false, 1, 1); // Pause does not increment observer sequence.
  receipt.begin(ns + 150000000, ns + 150000000, (ns + 150000000) * 1e-9);
  rejects([&] { receipt.complete(ns + 150000000, (ns + 150000000) * 1e-9, true, 1, 1); });
  receipt.complete(ns + 150000000, (ns + 150000000) * 1e-9, true, 2, 1);
  assert(receipt.id == 4);
  LtvEventReceipt bad;
  rejects([&] { bad.begin(ns, ns + 1, t); });
  bad.begin(ns, ns, t);
  rejects([&] { bad.begin(ns, ns, t); });
  rejects([&] { bad.complete(ns, t + 1, true, 1, 1); });
  rejects([&] { bad.complete(ns + 1, t, true, 1, 1); });
  bad.complete(ns, t, true, 1, 1);
  LtvEventReceipt adjacent;
  assert(ns * 1e-9 == (ns + 1) * 1e-9);
  adjacent.begin(ns, ns, t);
  adjacent.complete(ns, 0, false, 0, 0);
  adjacent.begin(ns + 1, ns + 1, (ns + 1) * 1e-9);
  adjacent.complete(ns + 1, 0, false, 0, 0);
  assert(adjacent.id == 2 && adjacent.camera_ns == ns + 1);
  rejects([&] { adjacent.begin(ns + 1, ns + 1, (ns + 1) * 1e-9); });
  rejects([&] { adjacent.begin(ns, ns, t); });
  assert(adjacent.id == 2);
  std::ostringstream failed;
  failed.setstate(std::ios::badbit);
  rejects([&] { LtvEventReceipt::require_written(failed); });
  std::ostringstream healthy;
  healthy << "receipt";
  LtvEventReceipt::require_written(healthy);
  std::cout << "PASS integer receipt, startup, pause, epoch, duplicate and stale frame checks\n";
}
