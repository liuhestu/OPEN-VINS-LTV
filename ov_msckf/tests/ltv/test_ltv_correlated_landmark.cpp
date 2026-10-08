#include "ltv/fusion/LtvCorrelatedLandmark.h"
#include <cassert>
#include <iostream>
int main() {
  Eigen::MatrixXd x(4, 8), t(3, 8), a(3, 8), v(2, 8);
  x.setRandom();
  t.setRandom();
  a.setRandom();
  v.setRandom();
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.3, Eigen::Vector3d(.2, .4, .8).normalized()).toRotationMatrix();
  const Eigen::MatrixXd noise = t - rotation * a;
  const auto moments = ltv::correlatedLandmarkMoments(t * t.transpose(), a * a.transpose(), t * a.transpose(), rotation, x * t.transpose(),
                                                      x * a.transpose());
  assert((moments.R - noise * noise.transpose()).norm() < 1e-12);
  assert((moments.N - x * noise.transpose()).norm() < 1e-12);
  assert((ltv::landmarkVisualNoiseCross(t * v.transpose(), a * v.transpose(), rotation) - noise * v.transpose()).norm() < 1e-12);
  Eigen::MatrixXd h(3, 4);
  h.setRandom();
  const auto result = ltv::correlatedUpdate(x * x.transpose(), h, moments.R, moments.N, Eigen::Vector3d(.01, -.02, .03));
  const Eigen::MatrixXd residual = h * x + noise, cross = x * residual.transpose();
  assert((result.innovation - residual * residual.transpose()).norm() < 1e-12);
  assert((result.gain - cross * (residual * residual.transpose()).ldlt().solve(Eigen::Matrix3d::Identity())).norm() < 1e-12);
  assert((result.posterior - (x * x.transpose() - cross * (residual * residual.transpose()).ldlt().solve(cross.transpose()))).norm() <
         1e-12);
  // Complete duplicate information: r=x+(-x)=0. Calling an independent-noise
  // update would claim an arbitrary covariance reduction; the correlated solve
  // rejects its singular innovation rather than regularizing it into a PASS.
  bool rejects = false;
  try {
    ltv::correlatedUpdate(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
                          -Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  } catch (const std::invalid_argument &) {
    rejects = true;
  }
  assert(rejects);
  const Eigen::Matrix3d rt = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()).toRotationMatrix(),
                        ra = Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d pt(.1, -.2, .3), pa(-.2, .1, 0), la(.2, .4, 4), lt(.3, .2, 3.8);
  const auto jac = ltv::landmarkProgramPoseJacobian(rt, pt, ra, pa, la);
  auto residual_fn = [&](const Eigen::Matrix<double, 12, 1> &m) {
    Eigen::Matrix3d tr = Eigen::Matrix3d::Identity(), ar = tr;
    if (m.head<3>().norm() > 0)
      tr = Eigen::AngleAxisd(m.head<3>().norm(), m.head<3>().normalized()).toRotationMatrix();
    if (m.segment<3>(6).norm() > 0)
      ar = Eigen::AngleAxisd(m.segment<3>(6).norm(), m.segment<3>(6).normalized()).toRotationMatrix();
    return (lt - tr * rt * ((ar * ra).transpose() * la + pa - m.tail<3>() - pt + m.segment<3>(3))).eval();
  };
  double worst = 0;
  for (int k = 0; k < 12; ++k) {
    Eigen::Matrix<double, 12, 1> p = Eigen::Matrix<double, 12, 1>::Zero(), m = p;
    p[k] = 1e-6;
    m[k] = -1e-6;
    worst = std::max(worst, ((residual_fn(p) - residual_fn(m)) / 2e-6 - jac.col(k)).norm());
  }
  assert(worst < 1e-8);
  std::cout << "correlated_R_N_visual_cross=PASS stable_gain_posterior=PASS duplicate_singular_rejected=PASS landmark_program_pose_FD="
            << worst << " production_State_mutation=NONE physical_chart_consumption=UNQUALIFIED\n";
}
