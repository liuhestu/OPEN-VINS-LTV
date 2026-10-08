#include "ltv/observer/ltv_observer.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
using namespace ltv;

namespace {
const Eigen::Vector3d seeds[2] = {Eigen::Vector3d(.8, .3, 4), Eigen::Vector3d(-.5, .6, 3)};
Eigen::Vector3d nominalBearing(int point) { return seeds[point].normalized() + Eigen::Vector3d(point ? .03 : -.04, point ? -.02 : .02, 0); }
LtvObserver initial() {
  LtvObserver observer;
  LtvConfig config;
  config.enable = true; // All production numeric/readiness defaults are frozen.
  observer.configure(config);
  observer.start(0);
  if (!observer.enableControlledFeatures(1))
    throw std::runtime_error("controlled start rejected");
  LtvControlledFeatures control;
  control.epoch = 1;
  control.imu_timestamp = 0;
  control.retained_ids = {1, 2};
  std::vector<LtvFeatureObservation> observations;
  for (int point = 0; point < 2; ++point) {
    LtvLandmarkSeed seed;
    seed.feature_id = point + 1;
    seed.apply_mean = true;
    seed.mean_body = seeds[point];
    control.births.push_back(seed);
    LtvFeatureObservation observation;
    observation.feature_id = point + 1;
    observation.normalized_coordinate = nominalBearing(point);
    observations.push_back(observation);
  }
  if (!observer.updateFeaturesControlled(0, 0, observations, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), control).accepted)
    throw std::runtime_error("seed event rejected");
  return observer;
}
Eigen::VectorXd fragment(const Eigen::VectorXd &source, Eigen::VectorXd *before_camera = nullptr, Eigen::MatrixXd *fixed_gain_map = nullptr,
                         bool direct_bearing = false) {
  auto observer = initial();
  const int dimension = observer.state().size();
  Eigen::VectorXd output(3 * dimension);
  for (int step = 0; step < 2; ++step) {
    Eigen::Vector3d acceleration(.2 + .03 * step, -.1, 9.7), gyro(.03, -.02 + .01 * step, .04);
    if (!direct_bearing) {
      acceleration += source.head<3>();
      gyro += source.segment<3>(3);
    }
    observer.propagateImu(.01, acceleration, gyro, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (!observer.started())
      throw std::runtime_error("native IMU reset");
    output.segment(step * dimension, dimension) = observer.state();
  }
  if (before_camera)
    *before_camera = observer.state();
  LtvControlledFeatures control;
  control.epoch = 1;
  control.imu_timestamp = .02;
  control.retained_ids = {1, 2};
  std::vector<LtvFeatureObservation> observations;
  for (int point = 0; point < 2; ++point) {
    Eigen::Vector3d noise = Eigen::Vector3d::Zero();
    if (direct_bearing)
      noise = source.segment<3>(3 * point);
    else if (!point)
      noise << .4 * source[0] + source[6], .4 * source[3] + source[7], 0;
    else
      noise << .2 * source[0] - .5 * source[6], .3 * source[4] + .7 * source[7], 0;
    LtvFeatureObservation observation;
    observation.feature_id = point + 1;
    observation.normalized_coordinate = nominalBearing(point) + noise;
    observations.push_back(observation);
  }
  if (!observer.updateFeaturesControlled(.02, .02, observations, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), control).accepted)
    throw std::runtime_error("native camera rejected");
  output.tail(dimension) = observer.state();
  if (fixed_gain_map)
    *fixed_gain_map = observer.errorShadow().cameraSourceMap();
  if (!output.allFinite())
    throw std::runtime_error("nonfinite fragment");
  return output;
}
void matrixCsv(const std::string &path, const Eigen::MatrixXd &matrix) {
  std::ofstream out(path);
  out << std::setprecision(17);
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      if (col)
        out << ',';
      out << matrix(row, col);
    }
    out << '\n';
  }
  if (!out)
    throw std::runtime_error("matrix write failed");
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("usage: test_ltv_observer_mc existing-output-directory");
    const std::string root = argv[1];
    unsetenv("LTV_JOINT_SHADOW_PATH");
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(8);
    const Eigen::VectorXd nominal = fragment(zero);
    std::vector<Eigen::MatrixXd> jacobians;
    for (double h : {1e-6, 1e-7}) {
      Eigen::MatrixXd j(nominal.size(), 8);
      for (int axis = 0; axis < 8; ++axis) {
        Eigen::VectorXd plus = zero, minus = zero;
        plus[axis] += h;
        minus[axis] -= h;
        j.col(axis) = (fragment(plus) - fragment(minus)) / (2 * h);
      }
      jacobians.push_back(j);
    }
    matrixCsv(root + "/full_native_fd_jacobian.csv", jacobians.back());
    const double fd_difference = (jacobians[0] - jacobians[1]).norm();
    // Capture fixed-gain camera derivative once; no sampling consumes shadow maps.
    setenv("LTV_JOINT_SHADOW_PATH", (root + "/nominal_shadow.csv").c_str(), 1);
    Eigen::MatrixXd fixed_gain;
    fragment(zero, nullptr, &fixed_gain);
    unsetenv("LTV_JOINT_SHADOW_PATH");
    const int dimension = nominal.size() / 3;
    Eigen::MatrixXd full_bearing_fd(dimension, 6), fixed_gain_raw = fixed_gain;
    const Eigen::VectorXd bearing_zero = Eigen::VectorXd::Zero(6);
    for (int axis = 0; axis < 6; ++axis) {
      Eigen::VectorXd plus = bearing_zero, minus = bearing_zero;
      plus[axis] += 1e-7;
      minus[axis] -= 1e-7;
      full_bearing_fd.col(axis) =
          (fragment(plus, nullptr, nullptr, true).tail(dimension) - fragment(minus, nullptr, nullptr, true).tail(dimension)) / 2e-7;
    }
    // Shadow columns are unit-bearing tangents, whereas raw-coordinate input
    // perturbations include normalization Jacobian's 1/||raw bearing|| factor.
    for (int point = 0; point < 2; ++point)
      fixed_gain_raw.middleCols(3 * point, 3) /= nominalBearing(point).norm();
    matrixCsv(root + "/full_native_camera_bearing_fd.csv", full_bearing_fd);
    matrixCsv(root + "/fixed_gain_camera_raw_map.csv", fixed_gain_raw);
    matrixCsv(root + "/gain_schedule_difference.csv", full_bearing_fd - fixed_gain_raw);
    std::mt19937 generator(202610081);
    std::normal_distribution<double> normal(0, 1);
    std::ofstream summary(root + "/summary.csv");
    summary << std::setprecision(17)
            << "amplitude,draws,fd_step_difference,covariance_relative_error,error_rms,linear_prediction_rms,nonlinear_remainder_rms,"
               "shared_cross_time_covariance_norm,gain_schedule_difference_norm\n";
    for (double amplitude : {1e-5, 1e-4, 1e-3}) {
      Eigen::MatrixXd errors(nominal.size(), 4000), linear(nominal.size(), 4000), sources(8, 4000);
      for (int sample = 0; sample < 4000; ++sample) {
        for (int axis = 0; axis < 8; ++axis)
          sources(axis, sample) = amplitude * normal(generator);
        errors.col(sample) = fragment(sources.col(sample)) - nominal;
        linear.col(sample) = jacobians.back() * sources.col(sample);
      }
      const Eigen::MatrixXd remainder = errors - linear;
      const double rms = errors.norm() / std::sqrt(4000.), linear_rms = linear.norm() / std::sqrt(4000.);
      errors.colwise() -= errors.rowwise().mean();
      const Eigen::MatrixXd sample_cov = errors * errors.transpose() / 3999.;
      const Eigen::MatrixXd predicted_cov = amplitude * amplitude * jacobians.back() * jacobians.back().transpose();
      const std::string label = amplitude == 1e-5 ? "1e-5" : amplitude == 1e-4 ? "1e-4" : "1e-3";
      matrixCsv(root + "/sources_" + label + ".csv", sources.transpose());
      matrixCsv(root + "/centered_errors_" + label + ".csv", errors.transpose());
      matrixCsv(root + "/linear_predictions_" + label + ".csv", linear.transpose());
      matrixCsv(root + "/nonlinear_remainders_" + label + ".csv", remainder.transpose());
      matrixCsv(root + "/predicted_covariance_" + label + ".csv", predicted_cov);
      matrixCsv(root + "/sample_covariance_" + label + ".csv", sample_cov);
      summary << amplitude << ",4000," << fd_difference << ',' << (sample_cov - predicted_cov).norm() / predicted_cov.norm() << ',' << rms
              << ',' << linear_rms << ',' << remainder.norm() / std::sqrt(4000.) << ','
              << sample_cov.topRightCorner(dimension, dimension).norm() << ',' << (full_bearing_fd - fixed_gain_raw).norm() << '\n';
    }
    std::cout << "EXECUTED three amplitudes x 4000 IID native nonlinear fragments; diagnostic only; no real landmark calibration claim\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
