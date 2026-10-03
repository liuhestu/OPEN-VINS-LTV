#include "ValueDiagnostics.h"
#include "state/StateHelper.h"
#include <iomanip>
namespace ov_msckf {
namespace {
void matrix(std::ostream &out, const Eigen::MatrixXd &m) {
  out << "[";
  for (int i = 0; i < m.rows(); ++i) {
    if (i)
      out << ",";
    out << "[";
    for (int j = 0; j < m.cols(); ++j) {
      if (j)
        out << ",";
      if (std::isfinite(m(i, j)))
        out << m(i, j);
      else
        out << "null";
    }
    out << "]";
  }
  out << "]";
}
void block(std::ostream &out, const MeasurementBlock &b) {
  out << "{\"order\":[";
  for (size_t i = 0; i < b.order.size(); ++i) {
    if (i)
      out << ",";
    out << "[" << b.order[i]->id() << "," << b.order[i]->size() << "]";
  }
  out << "],\"H\":";
  matrix(out, b.H);
  out << ",\"res\":";
  matrix(out, b.res);
  out << ",\"R\":";
  matrix(out, b.R);
  out << "}";
}
} // namespace
ValueDiagnostics::ValueDiagnostics(const LtvOptions &options, double gravity) : options_(options), gravity_(gravity) {
  file_ = gzopen(options.value_diagnostics_path.c_str(), "wb1");
  if (!file_)
    throw std::runtime_error("cannot open value diagnostics");
}
ValueDiagnostics::~ValueDiagnostics() {
  if (file_)
    gzclose(file_);
}
Eigen::VectorXd ValueDiagnostics::shadow(const std::shared_ptr<State> &s, const MeasurementBlock &b, Eigen::MatrixXd *posterior) {
  const Eigen::MatrixXd P = StateHelper::get_full_covariance(s);
  if (posterior)
    *posterior = P;
  if (b.empty())
    return s->_imu->value();
  const auto small = StateHelper::get_marginal_covariance(s, b.order);
  Eigen::MatrixXd S(b.R.rows(), b.R.cols());
  S.triangularView<Eigen::Upper>() = b.H * small * b.H.transpose();
  S.triangularView<Eigen::Upper>() += b.R;
  Eigen::MatrixXd inverse = Eigen::MatrixXd::Identity(S.rows(), S.cols());
  Eigen::LLT<Eigen::MatrixXd> solve(S.selfadjointView<Eigen::Upper>());
  if (solve.info() != Eigen::Success)
    throw std::runtime_error("shadow nonSPD innovation");
  solve.solveInPlace(inverse);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(P.rows(), b.res.size());
  int col = 0;
  for (const auto &type : b.order) {
    M.noalias() += P.middleCols(type->id(), type->size()) * b.H.middleCols(col, type->size()).transpose();
    col += type->size();
  }
  Eigen::MatrixXd K = M * inverse.selfadjointView<Eigen::Upper>();
  Eigen::VectorXd dx = K * b.res;
  ov_type::IMU imu;
  imu.set_value(s->_imu->value());
  imu.update(dx.segment(s->_imu->id(), 15));
  if (posterior) {
    posterior->triangularView<Eigen::Upper>() -= K * M.transpose();
    *posterior = posterior->selfadjointView<Eigen::Upper>();
  }
  return imu.value();
}
void ValueDiagnostics::before(const std::shared_ptr<State> &s, const MeasurementBlock &visual, const LtvFrame &f) {
  const auto start = std::chrono::steady_clock::now();
  if (pending_)
    throw std::runtime_error("unmatched value diagnostic event");
  pending_ = true;
  row_.str("");
  row_.clear();
  row_ << std::setprecision(17);
  row_ << "{\"schema\":1,\"camera_time\":" << s->_timestamp << ",\"imu_time\":" << s->_timestamp + s->_calib_dt_CAMtoIMU->value()(0)
       << ",\"epoch\":" << f.epoch << ",\"version\":" << f.version << ",\"sequence\":" << f.sequence << ",\"available\":" << f.available
       << ",\"valid\":" << f.snapshot.valid << ",\"velocity_valid\":" << f.snapshot.velocity_valid
       << ",\"gravity_valid\":" << f.snapshot.gravity_valid << ",\"features\":" << f.snapshot.state_features
       << ",\"observed\":" << f.snapshot.observed_features << ",\"warmup\":" << f.snapshot.healthy_camera_updates
       << ",\"visual_rows\":" << visual.res.size() << ",\"prior\":";
  matrix(row_, s->_imu->value());
  row_ << ",\"fej\":";
  matrix(row_, s->_imu->fej());
  row_ << ",\"ltv_v\":";
  matrix(row_, f.snapshot.velocity_body);
  row_ << ",\"ltv_g\":";
  matrix(row_, f.snapshot.gravity_body);
  row_ << ",\"shadow_visual\":";
  matrix(row_, shadow(s, visual));
  for (int mask = 1; mask < 4; ++mask) {
    auto opt = options_;
    opt.enable_gravity = (mask & 1);
    opt.enable_velocity = (mask & 2);
    // Pure build: no adapter claim and a fresh receipt for every counterfactual.
    UpdaterLTV u(opt, gravity_);
    const auto aux = u.build(s, f);
    auto merged = UpdaterLTV::merge(visual, aux);
    std::string why;
    const bool accepted = !aux.empty() && UpdaterLTV::validate(s, merged, why);
    if (!accepted)
      merged = visual;
    const std::string name = mask == 1 ? "G" : mask == 2 ? "V" : "GV";
    row_ << ",\"shadow_" << name << "\":";
    matrix(row_, shadow(s, merged));
    row_ << ",\"" << name << "_accepted\":" << accepted << ",\"" << name << "_G_reason\":\"" << aux.receipt->diagnostics.gravity_reason
         << "\",\"" << name << "_V_reason\":\"" << aux.receipt->diagnostics.velocity_reason << "\"";
    if (options_.value_diagnostics_matrices) {
      row_ << ",\"block_" << name << "\":";
      block(row_, aux);
    }
  }
  if (options_.value_diagnostics_matrices) {
    row_ << ",\"P\":";
    matrix(row_, StateHelper::get_full_covariance(s));
    row_ << ",\"visual\":";
    block(row_, visual);
  }
  compute_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
void ValueDiagnostics::after(const std::shared_ptr<State> &s, double observer_seconds, double update_seconds) {
  if (!pending_)
    throw std::runtime_error("missing value diagnostic prior");
  row_ << ",\"posterior\":";
  matrix(row_, s->_imu->value());
  row_ << ",\"observer_seconds\":" << observer_seconds << ",\"msckf_seconds_without_diagnostics\":" << update_seconds - compute_seconds
       << ",\"diagnostic_compute_seconds\":" << compute_seconds << "}\n";
  const auto data = row_.str();
  if (gzwrite(file_, data.data(), data.size()) != int(data.size()))
    throw std::runtime_error("value diagnostic write failed");
  pending_ = false;
}
} // namespace ov_msckf
