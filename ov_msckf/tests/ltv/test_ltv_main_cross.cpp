#include "ltv/observer/LtvMainCrossShadow.h"
#include <cassert>
#include <iostream>
#include <random>
int main(int argc, char **argv) {
  assert(argc == 2);
  setenv("LTV_MAIN_CROSS_SHADOW_PATH", argv[1], 1);
  auto &s = ltv::LtvMainCrossShadow::instance();
  int owner = 1;
  Eigen::MatrixXd p = Eigen::MatrixXd::Identity(2, 2);
  p(0, 1) = p(1, 0) = .2;
  s.initialize(&owner, p);
  Eigen::MatrixXd jx(1, 2);
  jx << .5, -.3;
  Eigen::MatrixXd jz(1, 1);
  jz << .8;
  Eigen::MatrixXd q(1, 1);
  q << .1;
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2, 1), old_aux = Eigen::MatrixXd::Zero(0, 1);
  s.seed(jx, jz, q, b, old_aux);
  assert((s.cross() + p * jx.transpose()).norm() < 1e-12);
  Eigen::MatrixXd h(1, 2);
  h << 1., .4;
  Eigen::MatrixXd k(2, 1);
  k << .2, .1;
  Eigen::MatrixXd d = jz * q;
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(2, 2) - k * h;
  Eigen::MatrixXd newp = f * p * f.transpose() + k * q * k.transpose();
  s.visual(&owner, p, h, k, newp, q, &d, &b);
  const Eigen::MatrixXd expected_cross = -f * p * jx.transpose() - k * q * jz.transpose();
  assert((s.cross() - expected_cross).norm() < 1e-12);
  // Seed and visual reuse the SAME noisy source. Forgetting D demonstrably
  // changes the joint covariance; deterministic/PSD checks would miss it.
  assert((s.cross() + f * p * jx.transpose()).norm() > .01);
  constexpr int samples = 12000;
  std::mt19937 rng(20261008);
  std::normal_distribution<double> norm;
  Eigen::MatrixXd errors(3, samples);
  auto lp = p.llt().matrixL().toDenseMatrix();
  for (int i = 0; i < samples; ++i) {
    Eigen::Vector2d raw(norm(rng), norm(rng));
    Eigen::Vector2d x = lp * raw;
    const double z = std::sqrt(q(0, 0)) * norm(rng);
    errors.col(i).head<2>() = f * x - k * z;
    errors(2, i) = (-jx * x)[0] + jz(0, 0) * z;
  }
  errors.colwise() -= errors.rowwise().mean();
  Eigen::MatrixXd empirical = errors * errors.transpose() / (samples - 1);
  Eigen::MatrixXd predicted(3, 3);
  predicted.topLeftCorner(2, 2) = s.mainCovariance();
  predicted.topRightCorner(2, 1) = s.cross();
  predicted.bottomLeftCorner(1, 2) = s.cross().transpose();
  predicted.bottomRightCorner(1, 1) = s.auxCovariance();
  double relative = (predicted - empirical).norm() / predicted.norm();
  assert(relative < .05);
  // Main marginalization selects rows while the seed/historical variable is
  // preserved; this is different from deleting the history's error variable.
  Eigen::MatrixXd selector(1, 2);
  selector << 0, 1;
  s.mainMap(&owner, newp, selector, selector * newp * selector.transpose(), "controlled_marginalization");
  assert(s.auxCovariance().rows() == 1);
  assert((s.cross() - selector * expected_cross).norm() < 1e-12);
  std::cout << "seed_sign=PASS shared_visual_source=PASS MC_known_joint_law_relative=" << relative << " samples=" << samples
            << " seed=20261008 historical_aux_retained=PASS real_calibration=NOT_EVALUATED\n";
}
