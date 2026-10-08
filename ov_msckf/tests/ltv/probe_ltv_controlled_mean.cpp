// Deterministic nominal fixture only: no Monte Carlo samples are rerun.
#include <iomanip>
#define main controlled_mc_entry_not_called
#include "test_ltv_joint_factor_mc.cpp"
#undef main
int main() {
  Fixture f;
  const auto result = run(f, Eigen::VectorXd::Zero(f.bank->columns()), true);
  const Eigen::Vector3d landmark = result.joint.segment<3>(21), velocity = result.joint.segment<3>(24),
                        gravity = result.joint.segment<3>(27);
  std::cout << std::setprecision(17) << "{\"kind\":\"deterministic_nominal_only_no_MC_rerun\",\"landmark_error_m\":[" << landmark.x() << ','
            << landmark.y() << ',' << landmark.z() << "],\"landmark_norm_m\":" << landmark.norm() << ",\"velocity_error_mps\":["
            << velocity.x() << ',' << velocity.y() << ',' << velocity.z() << "],\"velocity_norm_mps\":" << velocity.norm()
            << ",\"gravity_error_mps2\":[" << gravity.x() << ',' << gravity.y() << ',' << gravity.z()
            << "],\"gravity_norm_mps2\":" << gravity.norm() << ",\"mixed_unit_9D_norm\":" << result.joint.segment(21, 9).norm()
            << ",\"MC_empirical_mean_components\":\"NOT_PERSISTED_IN_ORIGINAL_RAW_SUMMARY\"}\n";
}
