#pragma once
#include "LtvSeedEstimator.h"

namespace ltv {
struct SeedUncertainty {
  bool valid = false;
  std::string reason = "UNCOMPUTED";
  Eigen::Matrix3d covariance_B = Eigen::Matrix3d::Zero();
  Eigen::MatrixXd jacobian;
  double sigma_parallel = 0.0;
  double relative_parallel_risk = 0.0;
  double covariance_min_eigenvalue = 0.0;
};
class LtvSeedUncertainty {
public:
  // Input columns: pose [right rotation(3), world translation(3)] for each unique pose,
  // then camera [right rotation(3), body translation(3)], then bearing tangent(2).
  // Caller supplies the FULL joint covariance (cross blocks included), in this convention.
  // Fixed extrinsics use zero rows/columns. No diagonal jitter is introduced.
  static int inputDimension(const SeedInput &u);
  static Eigen::MatrixXd jacobian(const SeedInput &u, const SeedEstimate &mean);
  static SeedUncertainty propagate(const SeedInput &u, const SeedEstimate &mean, const Eigen::MatrixXd &joint_covariance);
};
} // namespace ltv
