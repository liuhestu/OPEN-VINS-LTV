#pragma once

#include <eigen3/Eigen/Dense>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ltv {

struct LtvConfig {
  int max_features = 30;
  double q_landmark = 1e-4;
  double v_landmark = 1e6;
  double v_velocity = 1e6;
  double v_gravity = 1e6;
  double initial_p_landmark = 1.0;
  double initial_p_velocity = 1.0;
  double initial_p_gravity = 1.0;
};

struct LtvFeatureObservation {
  int feature_id = -1;
  Eigen::Vector3d normalized_coordinate = Eigen::Vector3d::Zero();
};

struct LtvSnapshot {
  Eigen::Vector3d velocity_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_body = Eigen::Vector3d::Zero();
  int state_features = 0;
  int observed_features = 0;
  double innovation_norm = 0.0;
  double covariance_trace = 0.0;
  double covariance_min_diagonal = 0.0;
  double covariance_max_diagonal = 0.0;
};

// Value-only contract between lifecycle policy and the mathematical observer.
struct LtvLandmarkSeed {
  int feature_id = -1;
  bool apply_mean = true;
  Eigen::Vector3d mean_body = Eigen::Vector3d::Zero();
};

// Manager supplies the exact retained set, including unobserved coasting slots.
// Every new ID has exactly one birth; old IDs can never be reseeded in an epoch.
struct LtvControlledFeatures {
  std::vector<int> retained_ids;
  std::vector<LtvLandmarkSeed> births;
};

// Dynamic mathematical state [landmarks..., v_B, g_B]. The host owns all clocks,
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
