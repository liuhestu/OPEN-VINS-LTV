#include "native_test_support.h"
int main() {
  for (bool singular : {false, true})
    for (bool zero : {false, true}) {
      auto s = make_state();
      Eigen::MatrixXd A(15, 15);
      for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j)
          A(i, j) = std::sin(1.3 * i + 0.7 * j + 0.2 * i * j);
      Eigen::MatrixXd P = A * A.transpose() + Eigen::MatrixXd::Identity(15, 15);
      StateHelper::set_initial_covariance(s, P, {s->_imu});
      StateHelper::augment_clone(s, Eigen::Vector3d::Zero());
      auto clone = s->_clones_IMU.at(0.0);
      P = StateHelper::get_full_covariance(s);
      if (!singular) {
        P += 0.1 * Eigen::MatrixXd::Identity(P.rows(), P.cols());
        StateHelper::set_initial_covariance(s, P, {s->_imu, clone});
      }
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> prior(P);
      require(!singular || std::abs(prior.eigenvalues().minCoeff()) < 1e-10, "exact clone is singular");
      Eigen::Matrix<double, 3, 2> T;
      T << 1, 0, 0, 1, 0, 0;
      const Eigen::MatrixXd full_h = gv_h(*s->_imu, Eigen::Vector3d(0, 0, -1), T, false);
      Eigen::MatrixXd H(5, 6);
      H << full_h.leftCols(3), full_h.middleCols(6, 3);
      Eigen::MatrixXd Hd = Eigen::MatrixXd::Zero(5, P.rows());
      Hd.leftCols(3) = H.leftCols(3);
      Hd.middleCols(6, 3) = H.rightCols(3);
      const Eigen::MatrixXd R = 0.3 * Eigen::MatrixXd::Identity(5, 5);
      const Eigen::MatrixXd S = Hd * P * Hd.transpose() + R;
      const Eigen::MatrixXd K = P * Hd.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(5, 5));
      Eigen::VectorXd residual(5);
      residual << 0.02, -0.01, 0.03, 0.015, -0.01;
      if (zero)
        residual.setZero();
      const Eigen::VectorXd delta = K * residual;
      ov_type::IMU imu_expected;
      imu_expected.set_value(s->_imu->value());
      imu_expected.update(delta.head(15));
      ov_type::PoseJPL clone_expected;
      clone_expected.set_value(clone->value());
      clone_expected.update(delta.tail(6));
      const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P.rows(), P.cols());
      const Eigen::MatrixXd joseph = (I - K * Hd) * P * (I - K * Hd).transpose() + K * R * K.transpose();
      const Eigen::MatrixXd dense = P - K * S * K.transpose();
      StateHelper::EKFUpdate(s, {s->_imu->q(), s->_imu->v()}, H, residual, R);
      const Eigen::MatrixXd actual = StateHelper::get_full_covariance(s);
      metric("native_vs_Joseph", (actual - joseph).norm() / std::max(1.0, P.norm()), 1e-9);
      metric("native_vs_dense", (actual - dense).norm() / std::max(1.0, P.norm()), 1e-9);
      metric("native_mean_imu", (s->_imu->value() - imu_expected.value()).norm(), 1e-9);
      metric("native_mean_clone", (clone->value() - clone_expected.value()).norm(), 1e-9);
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(actual);
      const double bound = 1e-10 * std::max(1.0, eigen.eigenvalues().cwiseAbs().maxCoeff());
      metric("negative_eigenvalue", std::max(0.0, -eigen.eigenvalues().minCoeff()), bound);
      if (!zero)
        require(delta.segment(3, 3).norm() > 1e-8 && delta.segment(9, 6).norm() > 1e-8 && delta.tail(6).norm() > 1e-8,
                "full cross-state correction");
      std::cout << "singular=" << singular << " zero_innovation=" << zero << std::endl;
    }
}
