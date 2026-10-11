#pragma once
#include "ltv/fusion/UpdaterLTV.h"
#include <chrono>
#include <sstream>
#include <zlib.h>
namespace ov_msckf {
// Read-only replay diagnostics. No GT, adapter, receipt consumption, or native update calls.
class ValueDiagnostics {
public:
  explicit ValueDiagnostics(const LtvOptions &options, double gravity);
  ~ValueDiagnostics();
  void before(const std::shared_ptr<State> &state, const MeasurementBlock &visual, const LtvFrame &frame);
  void after(const std::shared_ptr<State> &state, double observer_seconds, double update_seconds);
  static Eigen::VectorXd shadow(const std::shared_ptr<State> &state, const MeasurementBlock &block, Eigen::MatrixXd *posterior = nullptr);
  double compute_seconds = 0;

private:
  LtvOptions options_;
  double gravity_;
  gzFile file_ = nullptr;
  std::ostringstream row_;
  bool pending_ = false;
};
} // namespace ov_msckf
