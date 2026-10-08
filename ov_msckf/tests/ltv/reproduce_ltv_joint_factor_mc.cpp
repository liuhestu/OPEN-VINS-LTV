#include <fstream>
#include <iomanip>
#define main original_mc_entry_not_called
#include "test_ltv_joint_factor_mc.cpp"
#undef main
void save(const std::string &path, const Eigen::MatrixXd &m) {
  std::ofstream out(path);
  out << std::setprecision(17);
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) {
      if (j)
        out << ',';
      out << m(i, j);
    }
    out << '\n';
  }
}
int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string dir = argv[1];
  Fixture nominal;
  const int width = nominal.bank->columns();
  const auto predicted = run(nominal, Eigen::VectorXd::Zero(width), true);
  const Eigen::MatrixXd sigma = predicted.factors * predicted.factors.transpose();
  constexpr int samples = 12000;
  std::mt19937 rng(20261008);
  std::normal_distribution<double> normal;
  Eigen::MatrixXd errors(33, samples);
  Fixture trial;
  for (int i = 0; i < samples; ++i) {
    Eigen::VectorXd draw(width);
    for (int k = 0; k < width; ++k)
      draw[k] = normal(rng);
    errors.col(i) = run(trial, draw, false).joint;
  }
  const Eigen::VectorXd mean = errors.rowwise().mean();
  errors.colwise() -= mean;
  const Eigen::MatrixXd empirical = errors * errors.transpose() / (samples - 1);
  save(dir + "/predicted_covariance.csv", sigma);
  save(dir + "/empirical_covariance.csv", empirical);
  save(dir + "/nominal_mean_error.csv", predicted.joint);
  save(dir + "/empirical_mean_error.csv", mean);
  const double relative = (empirical - sigma).norm() / sigma.norm();
  assert(relative < .05);
  const char *names[] = {"main_attitude",     "main_position",    "main_velocity",        "main_gyro_bias",
                         "main_accel_bias",   "history_attitude", "history_position",     "observer_landmark",
                         "observer_velocity", "observer_gravity", "saved_anchor_landmark"};
  const char *units[] = {"rad", "m", "m/s", "rad/s", "m/s^2", "rad", "m", "m", "m/s", "m/s^2", "m"};
  std::ofstream blocks(dir + "/physical_blocks.csv");
  blocks << std::setprecision(17)
         << "block,mean_units,covariance_units,predicted_trace,empirical_trace,relative_covariance_frobenius,nominal_mean_norm,empirical_"
            "mean_norm,mean_diff_norm,analysis_status\n";
  for (int b = 0; b < 11; ++b) {
    const auto p = sigma.block<3, 3>(3 * b, 3 * b), e = empirical.block<3, 3>(3 * b, 3 * b);
    blocks << names[b] << ',' << units[b] << ",(" << units[b] << ")^2," << p.trace() << ',' << e.trace() << ','
           << (e - p).norm() / std::max(1e-30, p.norm()) << ',' << predicted.joint.segment<3>(3 * b).norm() << ','
           << mean.segment<3>(3 * b).norm() << ',' << (mean.segment<3>(3 * b) - predicted.joint.segment<3>(3 * b)).norm()
           << ",DESCRIPTIVE_NO_RETROSPECTIVE_GATE\n";
  }
  std::ofstream summary(dir + "/reproduction.json");
  summary << std::setprecision(17)
          << "{\"samples\":12000,\"seed\":20261008,\"same_original_law\":true,\"aggregate_mixed_unit_relative_frobenius\":" << relative
          << ",\"frozen_aggregate_limit\":0.05,\"aggregate_pass\":true,\"block_tables\":\"descriptive; no new retrospective "
             "thresholds\",\"zero_mean_consistency\":\"NOT_PROVEN\",\"real_calibration\":\"NOT_EVALUATED\"}\n";
  std::cout << "same_seed_same_law_MC_reproduction_relative=" << relative << " samples=" << samples
            << " matrices_and_physical_block_means_persisted=TRUE no_parameter_change=TRUE\n";
}
