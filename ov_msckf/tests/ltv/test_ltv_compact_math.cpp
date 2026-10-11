#include "ltv/observer/LtvObserver.h"
#include <cassert>
#include <iostream>
#include <limits>
using namespace ltv;
int main() {
  LtvConfig c;
  c.q_landmark = 2;
  c.initial_p_landmark = 3;
  c.v_landmark = c.v_velocity = c.v_gravity = 0;
  LtvObserver o;
  o.configure(c);
  LtvControlledFeatures control;
  control.retained_ids = {7, 8};
  control.births = {{7, true, Eigen::Vector3d(2, 4, 5)}, {8, true, Eigen::Vector3d(1, 3, 4)}};
  assert(o.synchronize(control));
  const std::vector<LtvFeatureObservation> obs = {{7, Eigen::Vector3d::UnitZ()}};
  const Eigen::VectorXd original = o.state();
  const Eigen::MatrixXd P0 = o.covariance();
  assert(o.correct(0, obs, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
  assert((o.state() - original).norm() == 0 && (o.covariance() - P0).norm() == 0);
  assert(o.correct(.5, obs, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
  assert(std::abs(o.state()(0) - .5) < 1e-12 && std::abs(o.state()(1) - 1) < 1e-12);
  assert(std::abs(o.covariance()(0, 0) - .75) < 1e-12 && o.state()(2) == 5);
  assert(o.propagateImu(.02, Eigen::Vector3d(.1, .2, .3), Eigen::Vector3d(.5, -.3, .2)));
  const Eigen::VectorXd before = o.state();
  const Eigen::MatrixXd P = o.covariance();
  assert((P.block<3, 3>(0, 6).norm() > 0));
  control.retained_ids = {8, 9, 7};
  control.births = {{9, true, Eigen::Vector3d(3, 2, 1)}};
  assert(o.synchronize(control));
  const int indices[] = {3, 4, 5, -1, -1, -1, 0, 1, 2, 6, 7, 8, 9, 10, 11};
  for (int i = 0; i < 15; ++i)
    for (int j = 0; j < 15; ++j)
      if (indices[i] >= 0 && indices[j] >= 0)
        assert(o.covariance()(i, j) == P(indices[i], indices[j]));
  control.retained_ids = {7};
  control.births.clear();
  assert(o.synchronize(control));
  for (int k = 0; k < 200; ++k) {
    assert(o.propagateImu(.01, Eigen::Vector3d(.2, -.1, .5), Eigen::Vector3d(.1, -.2, .3)));
    assert(o.correct(.05, {{7, Eigen::Vector3d(.1, .2, 1)}}, Eigen::Matrix3d::Identity(), Eigen::Vector3d(.05, 0, 0)));
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(o.covariance());
    assert(eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() >= -1e-12);
  }
  const Eigen::VectorXd saved = o.state();
  const Eigen::MatrixXd saved_P = o.covariance();
  assert(o.correct(.05, {}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
  assert((o.state() - saved).norm() == 0 && (o.covariance() - saved_P).norm() == 0);
  assert(!o.propagateImu(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  assert(!o.correct(-1, obs, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
  assert(!o.correct(.1, {{7, Eigen::Vector3d::Zero()}}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
  assert((o.state() - saved).norm() == 0 && (o.covariance() - saved_P).norm() == 0);
  // Finite overflowing variance is rejected atomically before LLT.
  assert(!o.correct(std::numeric_limits<double>::denorm_min(), {{7, Eigen::Vector3d::UnitZ()}}, Eigen::Matrix3d::Identity(),
                    Eigen::Vector3d::Zero()));
  // Inject a finite non-SPD design matrix to exercise the actual LLT failure path.
  auto &bad = const_cast<Eigen::MatrixXd &>(o.covariance());
  bad = -100 * Eigen::MatrixXd::Identity(9, 9);
  assert(!o.correct(.5, {{7, Eigen::Vector3d::UnitZ()}}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
  assert((o.state() - saved).norm() == 0);
  std::cout << "PASS analytical correction, PSD, cross-block reorder/delete, first event, no observations, input and LLT failure\n";
}
