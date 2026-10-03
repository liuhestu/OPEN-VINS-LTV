// Native OpenVINS perturbation contract for the read-only LTV geometry bridge.
#include "types/PoseJPL.h"
#include "utils/quat_ops.h"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

static Eigen::Matrix3d exp3(const Eigen::Vector3d &x) {
  return x.norm() == 0 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(x.norm(), x.normalized()).toRotationMatrix();
}
static void require(bool pass, const char *message) {
  if (!pass)
    throw std::runtime_error(message);
}
static Eigen::Matrix<double, 6, 1> measured_error(const ov_type::PoseJPL &nominal, const Eigen::VectorXd &dx) {
  ov_type::PoseJPL perturbed;
  perturbed.set_value(nominal.value());
  perturbed.set_fej(nominal.fej());
  perturbed.update(dx);
  const Eigen::Matrix3d R_WB = nominal.Rot().transpose();
  const Eigen::Matrix3d relative = R_WB.transpose() * perturbed.Rot().transpose();
  const Eigen::Matrix3d antisymmetric = .5 * (relative - relative.transpose());
  Eigen::Matrix<double, 6, 1> error;
  error << antisymmetric(2, 1), antisymmetric(0, 2), antisymmetric(1, 0), perturbed.pos() - nominal.pos();
  require((perturbed.fej() - nominal.fej()).norm() == 0, "native update must not change FEJ");
  return error;
}
int main() {
  try {
    Eigen::Matrix<double, 12, 12> measured = Eigen::Matrix<double, 12, 12>::Zero();
    double worst = 0;
    for (int k = 0; k < 2; ++k) {
      ov_type::PoseJPL pose;
      Eigen::Matrix<double, 7, 1> value;
      const Eigen::Matrix3d R_WB = exp3(Eigen::Vector3d(.3 + k * .2, -.2, .4 - k * .1));
      value << ov_core::rot_2_quat(R_WB.transpose()), 1.2 + k, -.7, .3;
      pose.set_value(value);
      pose.set_fej(value);
      for (double h : {1e-4, 1e-5}) {
        Eigen::Matrix<double, 6, 6> J;
        for (int axis = 0; axis < 6; ++axis) {
          Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
          delta(axis) = h;
          J.col(axis) = (measured_error(pose, delta) - measured_error(pose, -delta)) / (2 * h);
        }
        const double error = (J - Eigen::Matrix<double, 6, 6>::Identity()).cwiseAbs().maxCoeff();
        worst = std::max(worst, error);
        require(error < 2e-8, "native errors must map to right body rotation and world position without sign flip");
        measured.block<6, 6>(6 * k, 6 * k) = J;
        ov_type::PoseJPL updated;
        updated.set_value(value);
        Eigen::Matrix<double, 6, 1> dx;
        dx << h, -.5 * h, .2 * h, .3 * h, -.2 * h, .1 * h;
        updated.update(dx);
        // Native normalized delta quaternion agrees with Exp to third order.
        require((updated.Rot().transpose() - R_WB * exp3(dx.head<3>())).norm() < 1e-11, "finite positive-right rotation sign");
        require((updated.pos() - pose.pos() - dx.tail<3>()).norm() < 1e-14, "position increment is in world coordinates");
      }
    }
    Eigen::Matrix<double, 12, 12> A;
    for (int i = 0; i < 12; ++i)
      for (int j = 0; j < 12; ++j)
        A(i, j) = std::sin(.3 * i + .7 * j + .1 * i * j);
    const Eigen::Matrix<double, 12, 12> P = A * A.transpose() + Eigen::Matrix<double, 12, 12>::Identity();
    const double relative = (measured * P * measured.transpose() - P).norm() / P.norm();
    require(P.block<6, 6>(0, 6).norm() > .1, "test must contain nonzero cross-clone covariance");
    require(relative < 4e-8, "full joint pose P needs neither sign changes nor dropped cross blocks");
    std::cout << "PASS native_pose_error_J_max=" << worst << " full_cross_P_relative=" << relative << "\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
