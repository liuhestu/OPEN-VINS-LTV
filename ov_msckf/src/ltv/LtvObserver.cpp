#include "ltv/LtvObserver.h"
#include "utils/quat_ops.h"
#include <Eigen/Cholesky>
#include <cmath>
#include <set>
#include <stdexcept>

namespace ltv {
void LtvObserver::configure(const LtvConfig &config) {
  const double positive[] = {config.q_landmark, config.initial_p_landmark, config.initial_p_velocity, config.initial_p_gravity};
  const double nonnegative[] = {config.v_landmark, config.v_velocity, config.v_gravity};
  for (double value : positive)
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("invalid observer design parameter");
  for (double value : nonnegative)
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("invalid observer process design parameter");
  if (config.max_features < 1)
    throw std::invalid_argument("invalid observer capacity");
  config_ = config;
  reset();
}

void LtvObserver::reset() {
  feature_to_slot_.clear();
  state_ = Eigen::VectorXd::Zero(6);
  covariance_ = Eigen::MatrixXd::Zero(6, 6);
  covariance_.topLeftCorner<3, 3>().diagonal().setConstant(config_.initial_p_velocity);
  covariance_.bottomRightCorner<3, 3>().diagonal().setConstant(config_.initial_p_gravity);
  observed_features_ = 0;
  innovation_norm_ = 0;
}

bool LtvObserver::synchronize(const LtvControlledFeatures &control) {
  if (state_.size() != velocityOffset() + 6 || covariance_.rows() != state_.size() || covariance_.cols() != state_.size() ||
      !state_.allFinite() || !covariance_.allFinite())
    return false;
  std::set<int> retained(control.retained_ids.begin(), control.retained_ids.end()), births;
  if (retained.size() != control.retained_ids.size() || retained.size() > static_cast<size_t>(config_.max_features) ||
      (!retained.empty() && *retained.begin() < 0))
    return false;
  for (const auto &birth : control.births)
    if (!retained.count(birth.feature_id) || !births.insert(birth.feature_id).second || feature_to_slot_.count(birth.feature_id) ||
        !birth.mean_body.allFinite())
      return false;
  for (int id : retained)
    if (!feature_to_slot_.count(id) && !births.count(id))
      return false;
  rebuildState(control.retained_ids);
  for (const auto &birth : control.births)
    if (birth.apply_mean)
      state_.segment<3>(3 * feature_to_slot_.at(birth.feature_id)) = birth.mean_body;
  return true;
}

void LtvObserver::rebuildState(const std::vector<int> &feature_ids) {
  const int old_feature_count = static_cast<int>(feature_to_slot_.size());
  const int old_velocity_offset = 3 * old_feature_count;
  const int old_gravity_offset = old_velocity_offset + 3;
  const int new_velocity_offset = 3 * static_cast<int>(feature_ids.size());
  const int new_gravity_offset = new_velocity_offset + 3;
  const int new_dimension = new_gravity_offset + 3;

  Eigen::VectorXd new_state = Eigen::VectorXd::Zero(new_dimension);
  Eigen::MatrixXd new_covariance = Eigen::MatrixXd::Zero(new_dimension, new_dimension);
  std::vector<int> old_index(new_dimension, -1);
  std::unordered_map<int, int> new_feature_to_slot;

  for (std::size_t slot = 0; slot < feature_ids.size(); ++slot) {
    const int feature_id = feature_ids[slot];
    new_feature_to_slot[feature_id] = static_cast<int>(slot);
    const auto old_slot = feature_to_slot_.find(feature_id);
    if (old_slot != feature_to_slot_.end()) {
      for (int axis = 0; axis < 3; ++axis)
        old_index[3 * static_cast<int>(slot) + axis] = 3 * old_slot->second + axis;
    }
  }
  for (int axis = 0; axis < 3; ++axis) {
    old_index[new_velocity_offset + axis] = old_velocity_offset + axis;
    old_index[new_gravity_offset + axis] = old_gravity_offset + axis;
  }

  for (int row = 0; row < new_dimension; ++row) {
    if (old_index[row] >= 0)
      new_state[row] = state_[old_index[row]];
    for (int column = 0; column < new_dimension; ++column) {
      if (old_index[row] >= 0 && old_index[column] >= 0)
        new_covariance(row, column) = covariance_(old_index[row], old_index[column]);
    }
  }

  for (std::size_t slot = 0; slot < feature_ids.size(); ++slot) {
    if (feature_to_slot_.count(feature_ids[slot]) == 0) {
      new_covariance.block<3, 3>(3 * static_cast<int>(slot), 3 * static_cast<int>(slot)) =
          config_.initial_p_landmark * Eigen::Matrix3d::Identity();
    }
  }

  state_.swap(new_state);
  covariance_.swap(new_covariance);
  feature_to_slot_.swap(new_feature_to_slot);
}

bool LtvObserver::propagateImu(double dt, const Eigen::Vector3d &acceleration, const Eigen::Vector3d &omega) {
  if (!std::isfinite(dt) || dt <= 0 || !acceleration.allFinite() || !omega.allFinite() || !state_.allFinite() || !covariance_.allFinite() ||
      covariance_.rows() != state_.size() || covariance_.cols() != state_.size())
    return false;
  const int v = velocityOffset(), g = gravityOffset(), n = state_.size();
  const Eigen::Matrix3d rotation = -ov_core::skew_x(omega);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd noise = Eigen::VectorXd::Constant(n, config_.v_landmark);
  for (int slot = 0; slot < v / 3; ++slot) {
    A.block<3, 3>(3 * slot, 3 * slot) = rotation;
    A.block<3, 3>(3 * slot, v) = -Eigen::Matrix3d::Identity();
  }
  A.block<3, 3>(v, v) = rotation;
  A.block<3, 3>(v, g).setIdentity();
  A.block<3, 3>(g, g) = rotation;
  noise.segment<3>(v).setConstant(config_.v_velocity);
  noise.segment<3>(g).setConstant(config_.v_gravity);
  // The first-order mean and congruence design propagation share F.
  // With nonnegative V this preserves PSD without spectral projection.
  const Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n) + dt * A;
  Eigen::VectorXd x = F * state_;
  x.segment<3>(v) += dt * acceleration;
  Eigen::MatrixXd P = F * covariance_ * F.transpose();
  P.diagonal() += dt * noise;
  P = (0.5 * (P + P.transpose())).eval();
  if (!x.allFinite() || !P.allFinite())
    return false;
  state_.swap(x);
  covariance_.swap(P);
  return true;
}

bool LtvObserver::correct(double dt, const std::vector<LtvFeatureObservation> &observations, const Eigen::Matrix3d &R_BC,
                          const Eigen::Vector3d &p_BC) {
  if (!std::isfinite(dt) || dt < 0 || !R_BC.allFinite() || !p_BC.allFinite() ||
      !(R_BC * R_BC.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) || R_BC.determinant() <= 0 || !state_.allFinite() ||
      !covariance_.allFinite() || covariance_.rows() != state_.size() || covariance_.cols() != state_.size())
    return false;
  const int m = 3 * observations.size(), n = state_.size();
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(m);
  std::set<int> seen;
  for (size_t i = 0; i < observations.size(); ++i) {
    const auto &o = observations[i];
    const int slot = slotForFeature(o.feature_id);
    if (slot < 0 || !seen.insert(o.feature_id).second || !o.normalized_coordinate.allFinite() ||
        !std::isfinite(o.normalized_coordinate.norm()) || o.normalized_coordinate.norm() < 1e-12)
      return false;
    const Eigen::Vector3d ray = R_BC * o.normalized_coordinate.normalized();
    const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity() - ray * ray.transpose();
    C.block<3, 3>(3 * i, 3 * slot) = projection;
    y.segment<3>(3 * i) = projection * p_BC;
  }
  const Eigen::VectorXd residual = y - C * state_;
  if (!residual.allFinite())
    return false;
  if (dt > 0 && m > 0) {
    // P is an observer design matrix, never the auxiliary measurement noise.
    const double variance = 1.0 / (config_.q_landmark * dt);
    if (!std::isfinite(variance) || variance <= 0)
      return false;
    const Eigen::MatrixXd PCt = covariance_ * C.transpose();
    Eigen::MatrixXd S = C * PCt;
    S.diagonal().array() += variance;
    Eigen::LLT<Eigen::MatrixXd> solver(S);
    if (!S.allFinite() || solver.info() != Eigen::Success)
      return false;
    const Eigen::MatrixXd K = solver.solve(PCt.transpose()).transpose();
    // Joseph form retains the positive measurement contribution even when
    // cancellation makes the subtractive covariance formula inaccurate.
    const Eigen::MatrixXd J = Eigen::MatrixXd::Identity(n, n) - K * C;
    Eigen::VectorXd x = state_ + K * residual;
    Eigen::MatrixXd P = J * covariance_ * J.transpose() + variance * K * K.transpose();
    P = (0.5 * (P + P.transpose())).eval();
    if (!K.allFinite() || !x.allFinite() || !P.allFinite())
      return false;
    state_.swap(x);
    covariance_.swap(P);
  }
  observed_features_ = observations.size();
  innovation_norm_ = residual.norm();
  return true;
}

LtvSnapshot LtvObserver::snapshot() const {
  LtvSnapshot result;
  result.state_features = feature_to_slot_.size();
  result.observed_features = observed_features_;
  result.innovation_norm = innovation_norm_;
  result.velocity_body = state_.segment<3>(velocityOffset());
  result.gravity_body = state_.segment<3>(gravityOffset());
  result.covariance_trace = covariance_.trace();
  result.covariance_min_diagonal = covariance_.diagonal().minCoeff();
  result.covariance_max_diagonal = covariance_.diagonal().maxCoeff();
  return result;
}

int LtvObserver::slotForFeature(int id) const {
  const auto slot = feature_to_slot_.find(id);
  return slot == feature_to_slot_.end() ? -1 : slot->second;
}
} // namespace ltv
