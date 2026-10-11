#pragma once
#include "ltv/features/LtvFeatureTypes.h"

namespace ltv {
class LtvSeedEstimator {
public:
  // Numerical validity only: quality admission thresholds belong to the selector.
  static SeedEstimate estimate(const SeedInput &input);
  static Eigen::Matrix<double, 3, 2> tangentBasis(const Eigen::Vector3d &unit_bearing);
  static Eigen::Matrix3d skew(const Eigen::Vector3d &v);
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
