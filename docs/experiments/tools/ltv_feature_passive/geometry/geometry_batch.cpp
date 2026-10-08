#include "ltv/landmark_adapter/LtvSeedEstimator.h"
#include "ltv/landmark_adapter/LtvSeedUncertainty.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace ltv;
int main(int argc, char **argv) {
  if (argc != 3)
    return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output)
    return 3;
  output << std::setprecision(17);
  size_t np, nc, no, execution, anchor;
  int uncertainty;
  while (input >> np >> nc >> no >> execution >> anchor >> uncertainty) {
    if (np > 100 || nc > 8 || no > 200 || np == 0 || nc == 0)
      return 4;
    SeedInput u;
    u.poses.resize(np);
    u.cameras.resize(nc);
    u.observations.resize(no);
    u.execution_pose_index = execution;
    u.anchor_observation = anchor;
    for (auto &p : u.poses) {
      input >> p.t;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          input >> p.R_WB(r, c);
      for (int r = 0; r < 3; ++r)
        input >> p.p_WB(r);
    }
    for (auto &c : u.cameras) {
      for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k)
          input >> c.R_BC(r, k);
      for (int r = 0; r < 3; ++r)
        input >> c.p_BC(r);
    }
    for (auto &o : u.observations) {
      input >> o.pose_index >> o.camera_index;
      for (int r = 0; r < 3; ++r)
        input >> o.bearing_C(r);
    }
    Eigen::MatrixXd covariance;
    if (uncertainty) {
      int n = LtvSeedUncertainty::inputDimension(u);
      covariance.resize(n, n);
      for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c)
          input >> covariance(r, c);
    }
    if (!input)
      return 5;
    auto mean = LtvSeedEstimator::estimate(u);
    SeedUncertainty risk;
    if (uncertainty && mean.valid)
      risk = LtvSeedUncertainty::propagate(u, mean, covariance);
    output << "{\"valid\":" << (mean.valid ? "true" : "false") << ",\"reason\":\"" << mean.reason << "\",\"landmark_B\":["
           << mean.landmark_B.x() << "," << mean.landmark_B.y() << "," << mean.landmark_B.z() << "],\"condition\":" << mean.condition
           << ",\"max_angle_rad\":" << mean.max_angle_rad << ",\"max_residual_rad\":" << mean.max_residual_rad
           << ",\"range\":" << mean.range << ",\"min_ray\":" << (std::isfinite(mean.min_ray) ? mean.min_ray : 0)
           << ",\"risk_valid\":" << (risk.valid ? "true" : "false") << ",\"sigma_parallel\":" << risk.sigma_parallel
           << ",\"relative_parallel_risk\":" << risk.relative_parallel_risk << ",\"covariance_B\":[";
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) {
        if (r || c)
          output << ",";
        output << risk.covariance_B(r, c);
      }
    output << "]}\n";
  }
  return input.eof() ? 0 : 6;
}
