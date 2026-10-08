#include "ltv/landmark_adapter/LtvSeedUncertainty.h"
#include "ltv/observer/ltv_observer.h"
#include "utils/quat_ops.h"
#include <cassert>
#include <iostream>
#include <random>
using namespace ltv;
Eigen::Matrix3d rotation(const Eigen::Vector3d &theta) {
  Eigen::Vector4d q;
  q << .5 * theta, 1.;
  return ov_core::quat_2_Rot(q.normalized()).transpose();
}
struct Fixture {
  SeedInput input;
  SeedEstimate seed;
  Eigen::MatrixXd prior, h, k, seed_main, seed_pixel;
  std::shared_ptr<LtvJointFactors> bank;
  std::shared_ptr<LtvObserverSourceSensitivity> sensitivity;
  Fixture() {
    input.poses.resize(2);
    input.poses[0].p_WB = Eigen::Vector3d(-.1, 0, 0);
    input.poses[0].t = -.05;
    input.poses[1].t = 0;
    input.execution_pose_index = 1;
    input.cameras.resize(1);
    const Eigen::Vector3d point(.2, -.1, 4.);
    input.observations = {{0, 0, (point - input.poses[0].p_WB).normalized()}, {1, 0, point.normalized()}};
    seed = LtvSeedEstimator::estimate(input);
    assert(seed.valid);
    prior = 1e-8 * Eigen::MatrixXd::Identity(21, 21);
    prior.block<6, 6>(0, 15) = .3e-8 * Eigen::MatrixXd::Identity(6, 6);
    prior.block<6, 6>(15, 0) = prior.block<6, 6>(0, 15).transpose();
    prior.block<3, 3>(3, 12) = .2e-8 * Eigen::Matrix3d::Identity();
    prior.block<3, 3>(12, 3) = prior.block<3, 3>(3, 12).transpose();
    const Eigen::MatrixXd jac = LtvSeedUncertainty::jacobian(input, seed);
    seed_main = Eigen::MatrixXd::Zero(3, 21);
    seed_main.middleCols(15, 6) = jac.leftCols(6);
    seed_main.leftCols(6) = jac.middleCols(6, 6);
    seed_pixel = Eigen::MatrixXd::Zero(3, 4);
    for (int i = 0; i < 2; ++i) {
      const auto unit = input.observations[i].bearing_C;
      const Eigen::Vector3d raw = unit / unit.z();
      const Eigen::Matrix<double, 3, 2> dnorm = LtvDiscreteSensitivity::normalizedBearing(raw).leftCols<2>() / 500.;
      seed_pixel.middleCols(2 * i, 2) = jac.middleCols(18 + 2 * i, 2) * LtvSeedEstimator::tangentBasis(unit).transpose() * dnorm;
    }
    Eigen::Matrix<double, 2, 3> proj;
    proj << 500. / 4, 0, -500. * point.x() / 16, 0, 500. / 4, -500. * point.y() / 16;
    h = Eigen::MatrixXd::Zero(2, 21);
    h.leftCols(3) = proj * LtvSeedEstimator::skew(point);
    h.middleCols(3, 3) = -proj;
    k = prior * h.transpose() * (h * prior * h.transpose() + .0025 * Eigen::Matrix2d::Identity()).inverse();
    bank = std::make_shared<LtvJointFactors>();
    bank->initial("main", prior);
    for (int i = 0; i < 4; ++i)
      bank->source("pixel" + std::to_string(i), .0025 * Eigen::Matrix2d::Identity());
    for (int i = 0; i <= 10; ++i)
      bank->source("imu" + std::to_string(i), 1e-8 * Eigen::Matrix<double, 6, 6>::Identity());
    sensitivity = std::make_shared<LtvObserverSourceSensitivity>(bank);
  }
};
struct Result {
  Eigen::VectorXd joint;
  Eigen::MatrixXd factors;
};
Result run(Fixture &f, const Eigen::VectorXd &draw, bool derivatives) {
  auto value = [&](const std::string &key) { return (f.bank->block(key) * draw).eval(); };
  Eigen::VectorXd main = value("main");
  SeedInput input = f.input;
  input.poses[0].R_WB *= rotation(-main.segment<3>(15));
  input.poses[0].p_WB -= main.segment<3>(18);
  input.poses[1].R_WB *= rotation(-main.segment<3>(0));
  input.poses[1].p_WB -= main.segment<3>(3);
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector3d ray = f.input.observations[i].bearing_C;
    ray /= ray.z();
    ray.head<2>() += value("pixel" + std::to_string(i)) / 500.;
    input.observations[i].bearing_C = ray.normalized();
  }
  const auto seed = LtvSeedEstimator::estimate(input);
  assert(seed.valid);
  LtvObserver observer(false);
  LtvConfig config;
  config.enable = true;
  config.min_features = config.max_features = 1;
  observer.configure(config);
  observer.start(0);
  assert(observer.enableControlledFeatures(1));
  if (derivatives) {
    observer.attachSourceSensitivity(f.sensitivity);
    Eigen::MatrixXd seed_map = -f.seed_main * f.bank->block("main");
    seed_map += f.seed_pixel.leftCols(2) * f.bank->block("pixel0") + f.seed_pixel.rightCols(2) * f.bank->block("pixel1");
    f.sensitivity->seedInput(seed_map);
  }
  LtvControlledFeatures control;
  control.epoch = 1;
  control.imu_timestamp = 0;
  control.retained_ids = {1};
  LtvLandmarkSeed birth;
  birth.feature_id = 1;
  birth.mean_body = seed.landmark_B;
  control.births = {birth};
  LtvFeatureObservation observation;
  observation.feature_id = 1;
  observation.normalized_coordinate = f.input.observations[1].bearing_C / f.input.observations[1].bearing_C.z();
  observation.normalized_coordinate.head<2>() += value("pixel1") / 500.;
  assert(observer.updateFeaturesControlled(0, 0, {observation}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), control).accepted);
  control.births.clear();
  Eigen::Vector3d anchor;
  for (int frame = 1; frame <= 2; ++frame) {
    const Eigen::Vector3d ba = main.segment<3>(12), bg = main.segment<3>(9);
    if (derivatives) {
      Eigen::MatrixXd bias(6, f.bank->columns());
      bias.topRows(3) = f.bank->block("main").middleRows(12, 3);
      bias.bottomRows(3) = f.bank->block("main").middleRows(9, 3);
      f.bank->set("bias_prior", bias);
    }
    for (int step = 0; step < 5; ++step) {
      const int left = 5 * (frame - 1) + step, right = left + 1;
      const Eigen::VectorXd noise = .5 * (value("imu" + std::to_string(left)) + value("imu" + std::to_string(right)));
      if (derivatives)
        f.sensitivity->imuInput(.5 * (f.bank->block("imu" + std::to_string(left)) + f.bank->block("imu" + std::to_string(right))) +
                                f.bank->block("bias_prior"));
      observer.propagateImu(.01, Eigen::Vector3d(0, 0, 9.81) + ba + noise.head<3>(), bg + noise.tail<3>(), Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero());
      assert(observer.started());
    }
    const std::string pixel = "pixel" + std::to_string(frame + 1);
    const Eigen::Vector3d raw = f.input.observations[1].bearing_C / f.input.observations[1].bearing_C.z();
    observation.normalized_coordinate = raw;
    observation.normalized_coordinate.head<2>() += value(pixel) / 500.;
    if (derivatives) {
      Eigen::MatrixXd ray = Eigen::MatrixXd::Zero(3, f.bank->columns());
      ray.topRows(2) = f.bank->block(pixel) / 500.;
      f.sensitivity->cameraInput(raw, ray, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    }
    control.imu_timestamp = .05 * frame;
    assert(observer
               .updateFeaturesControlled(.05 * frame, .05 * frame, {observation}, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                                         control)
               .accepted);
    if (frame == 1) {
      anchor = observer.state().head<3>();
      if (derivatives)
        f.bank->set("anchor", f.bank->block("observer_mean").topRows(3));
      main = (Eigen::MatrixXd::Identity(21, 21) - f.k * f.h) * main - f.k * value(pixel);
      if (derivatives)
        f.bank->set("main", ((Eigen::MatrixXd::Identity(21, 21) - f.k * f.h) * f.bank->block("main") - f.k * f.bank->block(pixel)).eval());
    }
  }
  const Eigen::Vector3d truth(.2, -.1, 4.);
  Eigen::VectorXd actual_truth(9);
  actual_truth << truth, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, -9.81);
  Result out;
  out.joint.resize(33);
  out.joint.head(21) = main;
  out.joint.segment(21, 9) = observer.state() - actual_truth;
  out.joint.tail(3) = anchor - truth;
  if (derivatives) {
    out.factors.resize(33, f.bank->columns());
    out.factors.topRows(21) = f.bank->block("main");
    out.factors.middleRows(21, 9) = f.bank->block("observer_mean");
    out.factors.bottomRows(3) = f.bank->block("anchor");
  }
  return out;
}
int main() {
  Fixture nominal;
  const int width = nominal.bank->columns();
  const auto predicted = run(nominal, Eigen::VectorXd::Zero(width), true);
  const Eigen::MatrixXd sigma = predicted.factors * predicted.factors.transpose();
  constexpr int samples = 12000;
  std::mt19937 rng(20261008);
  std::normal_distribution<double> normal;
  Eigen::MatrixXd errors(33, samples);
  // Every trial uses the known stationary landmark truth and the same source
  // identity graph, not an estimated real landmark or a shadow-as-truth.
  Fixture trial;
  for (int i = 0; i < samples; ++i) {
    Eigen::VectorXd draw(width);
    for (int k = 0; k < width; ++k)
      draw[k] = normal(rng);
    errors.col(i) = run(trial, draw, false).joint;
  }
  const Eigen::VectorXd mean = errors.rowwise().mean();
  errors.colwise() -= mean;
  const Eigen::MatrixXd empirical = errors * errors.transpose() / (samples - 1);
  const double relative = (empirical - sigma).norm() / sigma.norm();
  assert(relative < .05);
  const Eigen::MatrixXd n = predicted.factors.middleRows(21, 3) - predicted.factors.bottomRows(3);
  const double full = n.squaredNorm(), zero = sigma.block<3, 3>(21, 21).trace() + sigma.bottomRightCorner<3, 3>().trace();
  assert(zero > full * 2);
  // Source handles and anchors are retained during QR compression. ALL joint
  // cross blocks, including endpoint reuse, must be invariant.
  nominal.bank->source("extra", Eigen::MatrixXd::Identity(400, 400));
  nominal.bank->erase("extra");
  const auto before = nominal.bank->covariance("main", "observer_mean");
  const int before_columns = nominal.bank->columns();
  nominal.bank->compress();
  assert(nominal.bank->columns() < before_columns);
  assert((before - nominal.bank->covariance("main", "observer_mean")).norm() < 1e-12);
  std::cout << "controlled_known_truth_native_Observer_joint_MC_relative=" << relative << " samples=" << samples
            << " seed=20261008 tolerance=.05 source_endpoint_reuse=PASS seed_history=PASS main_visual_shared_pixel=PASS "
               "prior_bias_receipt=PASS anchor_cross=PASS QR_sufficient_statistics=PASS\\n";
  std::cout << "residual_noise_full_trace=" << full << " zero_anchor_cross_trace=" << zero
            << " deterministic_known_mean_error_norm=" << predicted.joint.segment(21, 9).norm()
            << " empirical_mean_minus_model_norm=" << (mean - predicted.joint).norm() << " real_landmark_calibration=NOT_EVALUATED\\n";
}
