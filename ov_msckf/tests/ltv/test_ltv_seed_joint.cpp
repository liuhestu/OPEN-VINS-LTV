// LC-01/06 actual seed map, with native JPL pose updates and common physical sources.
#include "ltv/landmark_adapter/LtvSeedUncertainty.h"
#include "types/PoseJPL.h"
#include "utils/quat_ops.h"
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>

static void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
static Eigen::Matrix3d exp3(const Eigen::Vector3d &d) {
  if (d.norm() == 0)
    return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(d.norm(), d.normalized()).toRotationMatrix();
}
static ltv::SeedInput fixture(double angle) {
  ltv::SeedInput u;
  for (int i = 0; i < 3; ++i) {
    ltv::SeedPose p;
    p.R_WB = exp3(Eigen::Vector3d(.1, -.15, angle + .03 * i));
    p.p_WB = Eigen::Vector3d(.18 * i, .03 * i, .02 * i);
    p.t = .05 * i;
    u.poses.push_back(p);
  }
  ltv::SeedCamera c;
  c.R_BC = exp3(Eigen::Vector3d(.01, .02, -.03));
  c.p_BC = Eigen::Vector3d(.01, .02, .03);
  u.cameras.push_back(c);
  c.p_BC.x() += .15;
  u.cameras.push_back(c);
  for (size_t i = 0; i < u.poses.size(); ++i)
    for (size_t j = 0; j < u.cameras.size(); ++j) {
      const auto &p = u.poses[i];
      const auto &cam = u.cameras[j];
      Eigen::Vector3d b = cam.R_BC.transpose() * (p.R_WB.transpose() * (Eigen::Vector3d(.5, .3, 4) - p.p_WB) - cam.p_BC);
      u.observations.push_back({i, j, b.normalized()});
    }
  u.execution_pose_index = 2;
  return u;
}
static ltv::SeedInput perturb(const ltv::SeedInput &u, const Eigen::VectorXd &d) {
  auto out = u;
  int offset = 0;
  for (auto &p : out.poses) {
    ov_type::PoseJPL native;
    Eigen::Matrix<double, 7, 1> value;
    value << ov_core::rot_2_quat(p.R_WB.transpose()), p.p_WB;
    native.set_value(value);
    native.set_fej(value);
    native.update(d.segment(offset, 6));
    p.R_WB = native.Rot().transpose();
    p.p_WB = native.pos();
    offset += 6;
  }
  for (auto &c : out.cameras) {
    // SeedCamera error contract: right rotation, additive body-frame lever arm.
    c.R_BC = c.R_BC * exp3(d.segment<3>(offset));
    c.p_BC += d.segment<3>(offset + 3);
    offset += 6;
  }
  for (auto &b : out.observations) {
    b.bearing_C = (b.bearing_C + ltv::LtvSeedEstimator::tangentBasis(b.bearing_C) * d.segment<2>(offset)).normalized();
    offset += 2;
  }
  return out;
}
int main(int argc, char **argv) {
  try {
    double worst = 0;
    for (double angle : {-.5, .2, 1.1}) {
      const auto u = fixture(angle);
      const auto mean = ltv::LtvSeedEstimator::estimate(u);
      check(mean.valid, "seed fixture valid");
      const auto j = ltv::LtvSeedUncertainty::jacobian(u, mean);
      for (double step : {1e-5, 1e-6}) {
        Eigen::MatrixXd finite(3, j.cols());
        for (int k = 0; k < j.cols(); ++k) {
          Eigen::VectorXd d = Eigen::VectorXd::Zero(j.cols());
          d(k) = step;
          auto plus = ltv::LtvSeedEstimator::estimate(perturb(u, d));
          auto minus = ltv::LtvSeedEstimator::estimate(perturb(u, -d));
          check(plus.valid && minus.valid, "finite difference seed validity");
          finite.col(k) = (plus.landmark_B - minus.landmark_B) / (2 * step);
        }
        worst = std::max(worst, (finite - j).cwiseAbs().maxCoeff());
        check((finite - j).cwiseAbs().maxCoeff() < 2e-5, "native seed Jacobian finite difference");
      }
    }
    const auto u = fixture(.2);
    const auto mean = ltv::LtvSeedEstimator::estimate(u);
    const auto j = ltv::LtvSeedUncertainty::jacobian(u, mean);
    const int n = j.cols(), sources = 12;
    Eigen::MatrixXd m(n, sources), main_map(6, sources);
    for (int i = 0; i < n; ++i)
      for (int k = 0; k < sources; ++k)
        m(i, k) = .2 * std::sin(.13 * i + .7 * k + .1 * i * k);
    // Main true-minus-estimate is opposite of perturbed estimate-minus-true.
    main_map = -m.block(12, 0, 6, sources);
    Eigen::MatrixXd joint_map(9, sources);
    joint_map << main_map, j * m;
    const Eigen::MatrixXd source_reference = joint_map * joint_map.transpose();
    const auto propagated = ltv::LtvSeedUncertainty::propagate(u, mean, m * m.transpose());
    check(propagated.valid, "actual seed joint input valid");
    check((propagated.covariance_B - source_reference.bottomRightCorner(3, 3)).norm() < 1e-9, "seed Sigma agrees with direct source map");
    const Eigen::MatrixXd cx_seed = main_map * m.transpose() * j.transpose();
    check((cx_seed - source_reference.topRightCorner(6, 3)).norm() < 1e-10, "main seed cross agrees with source map");
    check(cx_seed.norm() > .1, "fixture contains substantial main seed dependency");
    if (argc > 1) {
      std::ofstream predicted(std::string(argv[1]) + ".predicted.csv");
      check(predicted.good(), "prediction output opens");
      predicted << std::setprecision(17);
      for (int i = 0; i < 9; ++i) {
        for (int k = 0; k < 9; ++k)
          predicted << (k ? "," : "") << source_reference(i, k);
        predicted << '\n';
      }
      std::ofstream output(argv[1]);
      check(output.good(), "sample output opens");
      output << std::setprecision(17) << "amplitude,draw";
      for (int i = 0; i < 9; ++i)
        output << ",linear_" << i;
      for (int i = 0; i < 9; ++i)
        output << ",actual_" << i;
      output << '\n';
      std::mt19937_64 generator(20261008);
      std::normal_distribution<double> gaussian;
      for (double amplitude : {1e-5, 1e-4, 1e-3})
        for (int draw = 0; draw < 12000; ++draw) {
          Eigen::VectorXd z(sources);
          for (int k = 0; k < sources; ++k)
            z(k) = gaussian(generator);
          const Eigen::VectorXd noise = amplitude * m * z;
          const auto estimate = ltv::LtvSeedEstimator::estimate(perturb(u, noise));
          check(estimate.valid, "controlled actual seed remains valid");
          Eigen::VectorXd actual(9);
          actual << amplitude * main_map * z, estimate.landmark_B - mean.landmark_B;
          const Eigen::VectorXd linear = amplitude * joint_map * z;
          output << amplitude << ',' << draw;
          for (int i = 0; i < 9; ++i)
            output << ',' << linear(i);
          for (int i = 0; i < 9; ++i)
            output << ',' << actual(i);
          output << '\n';
        }
    }
    std::cout << "PASS actual_seed_native_J_max=" << worst << " C_x_seed_norm=" << cx_seed.norm()
              << " real_history_joint_covariance=UNKNOWN real_landmark_truth=UNAVAILABLE\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
