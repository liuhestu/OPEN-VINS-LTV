#include "ltv/ltv_observer.h"
#include <iostream>
int main() {
  ltv::LtvObserver o;
  ltv::LtvConfig c;
  c.enable = true;
  c.max_missed_frames = 0;
  c.q_landmark = 0;
  o.configure(c);
  o.start(0);
  std::vector<ltv::LtvFeatureObservation> features;
  for (int id : {10, 20, 30}) {
    ltv::LtvFeatureObservation f;
    f.feature_id = id;
    f.normalized_coordinate << 0.1 * id, 0.2, 1;
    features.push_back(f);
  }
  o.updateFeatures(0, 0, features, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  Eigen::MatrixXd A(15, 15);
  for (int i = 0; i < 15; ++i)
    for (int j = 0; j < 15; ++j)
      A(i, j) = std::sin(0.7 * i + 1.3 * j + 0.1 * i * j);
  const Eigen::MatrixXd P = A * A.transpose() + Eigen::MatrixXd::Identity(15, 15);
  const Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(15, -1, 2);
  const_cast<Eigen::MatrixXd &>(o.covariance()) = P;
  const_cast<Eigen::VectorXd &>(o.state()) = x;
  const auto frozen = o.snapshot();
  features.erase(features.begin() + 1);
  ltv::LtvFeatureObservation added;
  added.feature_id = 40;
  added.normalized_coordinate << 0, 0, 1;
  features.push_back(added);
  o.updateFeatures(0.001, 0, features, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  const int indices[] = {0, 1, 2, 6, 7, 8, -1, -1, -1, 9, 10, 11, 12, 13, 14};
  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(15, 15);
  Eigen::VectorXd expected_x = Eigen::VectorXd::Zero(15);
  for (int i = 0; i < 15; ++i) {
    if (indices[i] >= 0)
      expected_x(i) = x(indices[i]);
    for (int j = 0; j < 15; ++j)
      if (indices[i] >= 0 && indices[j] >= 0)
        expected(i, j) = P(indices[i], indices[j]);
  }
  expected.block<3, 3>(6, 6) = c.initial_p_landmark * Eigen::Matrix3d::Identity();
  const double error = (expected - o.covariance()).cwiseAbs().maxCoeff();
  std::cout << "compaction_all_cross_blocks_error=" << error << " state_error=" << (expected_x - o.state()).norm() << std::endl;
  if (error > 1e-10 || (expected_x - o.state()).norm() > 1e-12 || o.slotForFeature(20) != -1 || o.slotForFeature(30) != 1 ||
      o.slotForFeature(40) != 2)
    return 1;
  o.reset();
  if ((frozen.velocity_body - x.segment<3>(9)).norm() != 0 || o.started() || o.state().size() != 6)
    return 1;
  return 0;
}
