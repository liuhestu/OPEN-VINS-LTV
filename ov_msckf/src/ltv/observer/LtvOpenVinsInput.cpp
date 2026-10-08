#include "ltv/observer/LtvOpenVinsInput.h"
#include "feat/Feature.h"
#include "ltv/diagnostics/joint/LtvMainCrossShadow.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include <algorithm>
#include <stdexcept>
namespace ov_msckf {
LtvOpenVinsInput makeLtvOpenVinsInput(const std::shared_ptr<State> &state, const std::vector<std::shared_ptr<ov_core::Feature>> &features,
                                      double camera_time, uint64_t version, bool managed_features, double pixel_noise_variance) {
  LtvOpenVinsInput input;
  auto &c = input.calibration;
  c.Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  c.Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
  c.Tg = State::Tg(state->_calib_imu_tg->value());
  c.R_ACCtoIMU = state->_calib_imu_ACCtoIMU->Rot();
  c.R_GYROtoIMU = state->_calib_imu_GYROtoIMU->Rot();
  c.R_BC = state->_calib_IMUtoCAM.at(0)->Rot().transpose();
  c.p_BC = -c.R_BC * state->_calib_IMUtoCAM.at(0)->pos();
  c.offset = state->_calib_dt_CAMtoIMU->value()(0);
  auto &feature_context = input.feature_context;
  feature_context.pixel_noise_variance = pixel_noise_variance;
  if (managed_features) {
    feature_context.version = version;
    feature_context.time = camera_time + c.offset;
    std::vector<std::shared_ptr<ov_type::Type>> pose_order;
    bool execution_pose_found = false;
    for (const auto &entry : state->_clones_IMU) {
      ltv::SeedPose pose;
      pose.t = entry.first + c.offset;
      pose.R_WB = entry.second->Rot().transpose();
      pose.p_WB = entry.second->pos();
      if (entry.first == camera_time) {
        feature_context.execution_pose_index = feature_context.poses.size();
        execution_pose_found = true;
      }
      feature_context.poses.push_back(pose);
      pose_order.push_back(entry.second);
    }
    if (!execution_pose_found)
      throw std::runtime_error("Feature context lacks current execution clone");
    // JPL's left delta on R_BW is exp(-[dtheta]) R_BW. Thus R_WB
    // has the right positive body delta used by the geometric Jacobian.
    // PoseJPL position error is additive in world coordinates. No sign flip
    // or block-diagonal approximation: retain all clone cross-covariances.
    feature_context.pose_covariance = StateHelper::get_marginal_covariance(state, pose_order);
    if (ltv::LtvMainCrossShadow::instance().enabled()) {
      feature_context.pose_error_selector = Eigen::MatrixXd::Zero(6 * pose_order.size(), state->max_covariance_size());
      for (size_t i = 0; i < pose_order.size(); ++i)
        feature_context.pose_error_selector.block(6 * i, pose_order[i]->id(), 6, 6).setIdentity();
    }
    for (int camera = 0; camera < state->_options.num_cameras; ++camera) {
      ltv::SeedCamera seed_camera;
      if (camera == 0) {
        seed_camera.R_BC = c.R_BC;
        seed_camera.p_BC = c.p_BC;
      } else {
        const auto &extrinsic = state->_calib_IMUtoCAM.at(camera);
        seed_camera.R_BC = extrinsic->Rot().transpose();
        seed_camera.p_BC = -seed_camera.R_BC * extrinsic->pos();
      }
      feature_context.cameras.push_back(seed_camera);
    }
  }
  auto &bearings = input.bearings;
  for (const auto &feature : features) {
    const int camera_count = managed_features ? state->_options.num_cameras : 1;
    for (int camera = 0; camera < camera_count; ++camera) {
      const auto history = feature->timestamps.find(camera);
      if (history == feature->timestamps.end())
        continue;
      const auto current = std::find(history->second.begin(), history->second.end(), camera_time);
      if (current == history->second.end())
        continue;
      const auto uv = feature->uvs_norm.at(camera).at(std::distance(history->second.begin(), current));
      const Eigen::Vector3d bearing(uv(0), uv(1), 1);
      if (managed_features) {
        ltv::HistoryObservation observation;
        observation.feature_id = feature->featid;
        observation.camera_id = camera;
        observation.bearing = bearing;
        if (ltv::LtvMainCrossShadow::instance().enabled()) {
          observation.pixel_source_key = ltv::LtvMainCrossShadow::pixelKey(feature->featid, camera, camera_time + c.offset);
          Eigen::MatrixXd dist, cal;
          state->_cam_intrinsics_cameras.at(camera)->compute_distort_jacobian(bearing.head<2>(), dist, cal);
          const Eigen::Vector3d unit = bearing.normalized();
          Eigen::Matrix<double, 3, 2> dnorm = (Eigen::Matrix3d::Identity() - unit * unit.transpose()).leftCols<2>() / bearing.norm();
          observation.pixel_to_tangent = ltv::LtvSeedEstimator::tangentBasis(unit).transpose() * dnorm * dist.inverse();
        }

        feature_context.observations.push_back(observation);
      }
      if (camera == 0)
        bearings.push_back({feature->featid, bearing});
    }
  }
  std::sort(bearings.begin(), bearings.end(), [](const LtvBearing &a, const LtvBearing &b) { return a.id < b.id; });
  return input;
}
} // namespace ov_msckf
