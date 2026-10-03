#include "native_test_support.h"
#include <fstream>
Eigen::MatrixXd read(std::istream &in) {
  int rows, cols;
  in >> rows >> cols;
  require(rows > 0 && cols > 0, "fixture shape");
  Eigen::MatrixXd m(rows, cols);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      in >> m(r, c);
  require(bool(in) && m.allFinite(), "fixture finite");
  return m;
}
int main() {
  std::ifstream file(LTV_TEST_FIXTURE_DIR "/joint_cancellation.txt");
  require(bool(file), "open cancellation fixture");
  const auto compact = read(file), H = read(file), R = read(file);
  auto state = make_state();
  std::vector<std::shared_ptr<ov_type::Type>> full_order = {state->_imu};
  MeasurementBlock block;
  for (int i = 0; i < 12; ++i) {
    state->_timestamp = i + 1;
    StateHelper::augment_clone(state, Eigen::Vector3d::Zero());
    auto clone = state->_clones_IMU.at(i + 1);
    block.order.push_back(clone);
    full_order.push_back(clone);
  }
  block.order.push_back(state->_imu->q());
  block.order.push_back(state->_imu->v());
  block.H = H;
  block.R = R;
  block.res = Eigen::VectorXd::Zero(H.rows());
  StateHelper::set_initial_covariance(state, 0.001 * Eigen::MatrixXd::Identity(87, 87), full_order);
  StateHelper::set_initial_covariance(state, compact, block.order);
  const auto small = StateHelper::get_marginal_covariance(state, block.order);
  const Eigen::MatrixXd raw = H * small * H.transpose() + R;
  std::cout << "raw_S_asymmetry=" << (raw - raw.transpose()).norm() << " raw_S_norm=" << raw.norm() << std::endl;
  std::string reason;
  require(UpdaterLTV::validate(state, block, reason), "native upper-triangle SPD precheck");
  Eigen::MatrixXd S(R.rows(), R.cols());
  S.triangularView<Eigen::Upper>() = H * small * H.transpose();
  S.triangularView<Eigen::Upper>() += R;
  S = S.selfadjointView<Eigen::Upper>();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(S);
  require(eig.eigenvalues().minCoeff() > 0.03, "actual innovation comfortably SPD");
  const auto P = StateHelper::get_full_covariance(state);
  Eigen::MatrixXd Hd = Eigen::MatrixXd::Zero(H.rows(), P.rows());
  int column = 0;
  for (const auto &type : block.order) {
    Hd.middleCols(type->id(), type->size()) = H.middleCols(column, type->size());
    column += type->size();
  }
  const Eigen::MatrixXd K = P * Hd.transpose() * S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.rows()));
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P.rows(), P.cols());
  const Eigen::MatrixXd joseph = (I - K * Hd) * P * (I - K * Hd).transpose() + K * R * K.transpose();
  StateHelper::EKFUpdate(state, block.order, H, block.res, R);
  const auto actual = StateHelper::get_full_covariance(state);
  metric("cancellation_native_Joseph", (actual - joseph).norm() / std::max(1.0, P.norm()), 1e-9);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> post(actual);
  metric("cancellation_P_negative", std::max(0.0, -post.eigenvalues().minCoeff()), 1e-10 * std::max(1.0, actual.norm()));
}
