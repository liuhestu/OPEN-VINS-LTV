#pragma once
#include "ltv/observer/LtvTypes.h"
#include <unordered_map>

namespace ltv {
// Dynamic mathematical state [landmarks..., v_B, g_B]. Runtime owns all clocks,
// health, epochs and lifecycle decisions. Failure leaves the committed x/P intact.
class LtvObserver {
public:
  LtvObserver() { reset(); }
  void configure(const LtvConfig &config);
  void reset();
  bool synchronize(const LtvControlledFeatures &control);
  bool propagateImu(double dt, const Eigen::Vector3d &acceleration, const Eigen::Vector3d &omega);
  bool correct(double dt, const std::vector<LtvFeatureObservation> &observations, const Eigen::Matrix3d &R_BC, const Eigen::Vector3d &p_BC);
  LtvSnapshot snapshot() const;
  const Eigen::VectorXd &state() const { return state_; }
  const Eigen::MatrixXd &covariance() const { return covariance_; }
  int slotForFeature(int id) const;

private:
  void rebuildState(const std::vector<int> &ids);
  int velocityOffset() const { return 3 * feature_to_slot_.size(); }
  int gravityOffset() const { return velocityOffset() + 3; }
  LtvConfig config_;
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
  std::unordered_map<int, int> feature_to_slot_;
  int observed_features_ = 0;
  double innovation_norm_ = 0;
};
} // namespace ltv
