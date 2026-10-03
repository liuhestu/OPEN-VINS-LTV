// Deterministic noise-free diagnosis, linked to the actual unchanged observer.
#include "ltv/ltv_observer.h"
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using V3 = Eigen::Vector3d;
struct Truth {
  Eigen::Matrix3d R;
  V3 p, v, a, omega;
};
Truth truth(double t, double scale, bool moving) {
  Truth x;
  const V3 axis = V3(.3, -.4, .8).normalized();
  const Eigen::Matrix3d base = (Eigen::AngleAxisd(.23, V3::UnitX()) * Eigen::AngleAxisd(-.18, V3::UnitY())).toRotationMatrix();
  x.R = base * Eigen::AngleAxisd(moving ? .6 * std::sin(.8 * t) : 0, axis).toRotationMatrix();
  x.omega = moving ? (.48 * std::cos(.8 * t) * axis).eval() : V3::Zero();
  x.p = scale * V3(1.8 * std::sin(.7 * t), 1.2 * std::sin(1.1 * t), .7 * std::sin(.9 * t));
  x.v = scale * V3(1.26 * std::cos(.7 * t), 1.32 * std::cos(1.1 * t), .63 * std::cos(.9 * t));
  x.a = scale * V3(-.882 * std::sin(.7 * t), -1.452 * std::sin(1.1 * t), -.567 * std::sin(.9 * t));
  if (!moving) {
    x.p.setZero();
    x.v.setZero();
    x.a.setZero();
  }
  return x;
}
int main(int argc, char **argv) {
  if (argc != 7) {
    std::cerr << "output.csv duration dt track_lifetime scale moving\n";
    return 2;
  }
  const double duration = std::stod(argv[2]), dt = std::stod(argv[3]), lifetime = std::stod(argv[4]), scale = std::stod(argv[5]);
  const bool moving = std::stoi(argv[6]);
  const int camera_step = std::lround(.05 / dt), n = 30;
  if (camera_step < 1 || std::abs(camera_step * dt - .05) > 1e-12)
    throw std::invalid_argument("dt must divide 20 Hz camera interval");
  ltv::LtvConfig config;
  config.enable = true;
  ltv::LtvObserver observer;
  observer.configure(config);
  observer.start(0);
  const V3 g(0, 0, -9.81), pc(.1, -.04, .06);
  const Eigen::Matrix3d Rbc = Eigen::AngleAxisd(.2, V3::UnitY()).toRotationMatrix();
  std::vector<V3> landmarks;
  for (int j = 0; j < n; ++j)
    landmarks.emplace_back((j % 6 - 2.5) * 1.4, (j / 6 - 2) * 1.3, 12 + .17 * j);
  std::ofstream out(argv[1]);
  out << std::setprecision(17)
      << "t,v_error,g_error_deg,speed_true,speed_ltv,eta_norm,features,observed,valid,substeps,reset,model_residual,propagation_error\n";
  double max_prop = 0, max_model = 0;
  for (int i = 0; i <= std::lround(duration / dt); ++i) {
    const double t = i * dt;
    const Truth x = truth(t, scale, moving);
    if (i) {
      const Truth mid = truth(t - dt / 2, scale, moving);
      const V3 acc = mid.R.transpose() * (mid.a - g), w = mid.omega;
      const Eigen::VectorXd before = observer.state();
      const int offset = before.size() - 6;
      Eigen::VectorXd expected = before;
      const V3 v = before.segment<3>(offset), eta = before.tail<3>();
      for (int col = 0; col < offset; col += 3)
        expected.segment<3>(col) = before.segment<3>(col) - dt * (w.cross(V3(before.segment<3>(col))) + v);
      expected.segment<3>(offset) = v + dt * (acc + eta - w.cross(v));
      expected.tail<3>() = eta - dt * w.cross(eta);
      observer.propagateImu(dt, acc, w, V3::Zero(), V3::Zero());
      if (!observer.started())
        throw std::runtime_error("core reset during IMU");
      max_prop = std::max(max_prop, (expected - observer.state()).norm());
    }
    if (i % camera_step)
      continue;
    std::vector<ltv::LtvFeatureObservation> observations;
    for (int j = 0; j < n; ++j) {
      const int generation = lifetime > 0 ? int(std::floor(t / lifetime + double(j) / n)) : 0;
      const V3 body = x.R.transpose() * (landmarks[j] - x.p);
      const V3 camera = Rbc.transpose() * (body - pc);
      if (camera.z() <= .5)
        throw std::runtime_error("landmark behind camera");
      ltv::LtvFeatureObservation feature;
      feature.feature_id = generation * n + j;
      feature.normalized_coordinate = camera / camera.z();
      observations.push_back(feature);
      const V3 b = Rbc * camera.normalized();
      max_model = std::max(max_model, ((Eigen::Matrix3d::Identity() - b * b.transpose()) * (body - pc)).norm());
    }
    const auto snap = observer.updateFeatures(t, t, observations, Rbc, pc);
    if (!observer.started())
      throw std::runtime_error("core reset during camera");
    const V3 gt_v = x.R.transpose() * x.v, gt_g = x.R.transpose() * g;
    double angle = -1;
    if (snap.gravity_body.norm() > 1e-12)
      angle = std::acos(std::max(-1., std::min(1., snap.gravity_body.normalized().dot(gt_g.normalized())))) * 180 / M_PI;
    out << t << ',' << (snap.velocity_body - gt_v).norm() << ',' << angle << ',' << gt_v.norm() << ',' << snap.velocity_body.norm() << ','
        << snap.gravity_body.norm() << ',' << snap.state_features << ',' << snap.observed_features << ',' << snap.valid << ','
        << snap.camera_substeps << ',' << ltv::toString(snap.last_reset_reason) << ',' << max_model << ',' << max_prop << '\n';
  }
  if (max_model > 1e-10 || max_prop > 1e-10)
    throw std::runtime_error("model/retraction contract mismatch");
  std::cout << "PASS physical measurement residual=" << max_model << " actual propagation error=" << max_prop << '\n';
}
