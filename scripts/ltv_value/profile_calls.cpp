// Optional LD_PRELOAD timing of the frozen shared library; no numerical replacements.
#include "state/StateHelper.h"
#include "update/UpdaterLTV.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <zlib.h>

namespace {
using Clock = std::chrono::steady_clock;
template <class T> T original(const char *symbol) {
  auto p = dlsym(RTLD_NEXT, symbol);
  if (!p) {
    std::fprintf(stderr, "profiling symbol not found: %s\n", symbol);
    std::abort();
  }
  return reinterpret_cast<T>(p);
}
void record(const char *name, double stamp, Clock::time_point begin) {
  const double seconds = std::chrono::duration<double>(Clock::now() - begin).count();
  static FILE *out = [] {
    const char *path = std::getenv("LTV_PROFILE_OUTPUT");
    FILE *f = path ? std::fopen(path, "w") : nullptr;
    if (!f)
      std::abort();
    std::fprintf(f, "function,camera_time,seconds\n");
    return f;
  }();
  // Logging occurs after the measured function; output is flushed at process exit.
  std::fprintf(out, "%s,%.17g,%.17g\n", name, stamp, seconds);
}
} // namespace

namespace ov_msckf {
MeasurementBlock UpdaterLTV::build(const std::shared_ptr<State> &state, const LtvFrame &snapshot) const {
  using Function = MeasurementBlock (*)(const UpdaterLTV *, const std::shared_ptr<State> &, const LtvFrame &);
  static auto fn = original<Function>("_ZNK8ov_msckf10UpdaterLTV5buildERKSt10shared_ptrINS_5StateEERKNS_8LtvFrameE");
  const auto begin = Clock::now();
  auto result = fn(this, state, snapshot);
  record("auxiliary_build", state->_timestamp, begin);
  return result;
}
void StateHelper::EKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<ov_type::Type>> &order,
                            const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R) {
  using Function = void (*)(std::shared_ptr<State>, const std::vector<std::shared_ptr<ov_type::Type>> &, const Eigen::MatrixXd &,
                            const Eigen::VectorXd &, const Eigen::MatrixXd &);
  static auto fn = original<Function>("_ZN8ov_msckf11StateHelper9EKFUpdateESt10shared_ptrINS_5StateEERKSt6vectorIS1_IN7ov_type4TypeEESaIS7_"
                                      "EERKN5Eigen6MatrixIdLin1ELin1ELi0ELin1ELin1EEERKNSD_IdLin1ELi1ELi0ELin1ELi1EEESG_");
  const auto begin = Clock::now();
  fn(state, order, H, res, R);
  record("native_ekf", state->_timestamp, begin);
}
} // namespace ov_msckf

extern "C" int gzwrite(gzFile file, voidpc data, unsigned int length) {
  using Function = int (*)(gzFile, voidpc, unsigned int);
  static auto fn = original<Function>("gzwrite");
  const auto begin = Clock::now();
  const int result = fn(file, data, length);
  record("gzip_write", -1, begin);
  return result;
}
