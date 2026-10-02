#include "cam/CamRadtan.h"
#include "native_test_support.h"
#include "update/UpdaterHelper.h"
int main() {
  for (auto integration : {StateOptions::DISCRETE, StateOptions::RK4, StateOptions::ANALYTICAL}) {
    StateOptions options;
    options.integration_method = integration;
    auto s = make_state(options);
    TestPropagator prop, live_prop;
    auto live = make_state(options);
    for (int sample = 0; sample <= 16; ++sample) {
      ImuData data;
      data.timestamp = sample * 0.01;
      data.am << 0.2, -0.1, 9.7;
      data.wm << 0.06, -0.04, 0.08;
      live_prop.feed_imu(data);
    }
    Eigen::MatrixXd N0 = gauge(*s->_imu), Phi = Eigen::MatrixXd::Identity(15, 15), Qsum = Eigen::MatrixXd::Zero(15, 15);
    Eigen::Matrix<double, 3, 2> T;
    T << 1, 0, 0, 1, 0, 0;
    Eigen::Vector3d gamma(0, 0, -1);
    metric("instant_HN", (gv_h(*s->_imu, gamma, T, true) * N0).norm(), 1e-10);
    const Eigen::VectorXd original = s->_imu->value();
    const auto prediction = gv_prediction(*s->_imu, gamma, T);
    auto transformed = original;
    const Eigen::Matrix3d yaw = exp_so3(0.4 * gamma);
    transformed.head<4>() = rot_2_quat(s->_imu->Rot() * yaw.transpose());
    transformed.segment<3>(4) = yaw * s->_imu->pos() + Eigen::Vector3d(1, 2, 3);
    transformed.segment<3>(7) = yaw * s->_imu->vel();
    ov_type::IMU alternate;
    alternate.set_value(transformed);
    metric("finite_yaw_translation_invariance", (gv_prediction(alternate, gamma, T) - prediction).norm(), 1e-10);
    std::vector<Eigen::MatrixXd> clone_gauges;
    for (int frame = 0; frame < 3; ++frame) {
      if (frame == 1) {
        const Eigen::VectorXd saved = s->_imu->fej();
        Eigen::VectorXd dx = Eigen::VectorXd::Zero(15);
        dx.head<3>() << 0.02, -0.01, 0.015;
        dx.segment<3>(3) << 0.1, -0.05, 0.04;
        dx.segment<3>(6) << -0.04, 0.03, 0.02;
        s->_imu->update(dx);
        live->_imu->update(dx);
        require((s->_imu->fej() - saved).norm() == 0, "current update preserves FEJ");
      }
      for (int step = 0; step < 5; ++step) {
        ImuData a, b;
        a.timestamp = (frame * 5 + step) * 0.01;
        b.timestamp = a.timestamp + 0.01;
        a.am << 0.2, -0.1, 9.7;
        a.wm << 0.06, -0.04, 0.08;
        b.am = a.am;
        b.wm = a.wm;
        Eigen::MatrixXd F, Q;
        prop.predict_and_compute(s, a, b, F, Q);
        Phi = F * Phi;
        StateHelper::EKFPropagation(s, {s->_imu}, {s->_imu}, F, Q);
        const Eigen::MatrixXd raw = F * Qsum * F.transpose() + Q;
        const Eigen::MatrixXd safe = (0.5 * (raw + raw.transpose())).eval();
        Qsum = raw;
        Qsum = 0.5 * (Qsum + Qsum.transpose()); // exact original accumulated-Q expression
        metric("native_Q_symmetry", (Q - Q.transpose()).norm(), 1e-12);
        metric("accumulated_Q_alias_error", (Qsum - safe).norm(), 1e-12);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> qe(Qsum);
        metric("native_Q_negative_eigenvalue", std::max(0.0, -qe.eigenvalues().minCoeff()), 1e-10 * std::max(1.0, Qsum.norm()));
        const Eigen::MatrixXd expected = gauge(*s->_imu);
        metric("Phi_gauge", (Phi * N0 - expected).norm() / std::max(1.0, expected.norm()), 1e-10);
      }
      s->_timestamp = (frame + 1) * 0.05;
      StateHelper::augment_clone(s, Eigen::Vector3d::Zero());
      live_prop.propagate_and_clone(live, s->_timestamp);
      metric("native_accumulated_propagation_mean", (live->_imu->value() - s->_imu->value()).norm(), 1e-10);
      const Eigen::MatrixXd live_P = StateHelper::get_full_covariance(live);
      const Eigen::MatrixXd step_P = StateHelper::get_full_covariance(s);
      metric("native_accumulated_propagation_P", (live_P - step_P).norm() / std::max(1.0, step_P.norm()), 1e-10);
      const auto clone = s->_clones_IMU.at(s->_timestamp);
      Eigen::MatrixXd Nc(6, 4);
      Nc = gauge(*s->_imu).topRows(6);
      Eigen::MatrixXd mapped = (Phi * N0).topRows(6);
      metric("clone_augmented_gauge", (mapped - Nc).norm() / std::max(1.0, Nc.norm()), 1e-10);
      const Eigen::MatrixXd P = StateHelper::get_full_covariance(s);
      metric("native_clone_covariance_mapping", (P.block(clone->id(), 0, 6, 15) - P.block(0, 0, 6, 15)).norm(), 1e-10);
      clone_gauges.push_back(mapped);
      metric("cross_frame_GV_HPhiN", (gv_h(*s->_imu, gamma, T, true) * Phi * N0).norm() / std::max(1.0, (Phi * N0).norm()), 1e-10);
    }
    auto camera = std::make_shared<CamRadtan>(640, 480);
    Eigen::VectorXd intrinsics(8);
    intrinsics << 300, 300, 320, 240, 0, 0, 0, 0;
    camera->set_value(intrinsics);
    s->_cam_intrinsics_cameras[0] = camera;
    s->_cam_intrinsics[0]->set_value(intrinsics);
    UpdaterHelper::UpdaterHelperFeature f;
    f.featid = 1;
    f.feat_representation = ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;
    f.p_FinG << 0.5, 0.2, 5;
    f.p_FinG_fej = f.p_FinG;
    for (const auto &entry : s->_clones_IMU) {
      f.timestamps[0].push_back(entry.first);
      Eigen::VectorXf uv(2);
      uv << 320, 240;
      f.uvs[0].push_back(uv);
      f.uvs_norm[0].push_back(Eigen::Vector2f::Zero());
      Eigen::VectorXd dx = Eigen::VectorXd::Zero(6);
      dx(0) = 0.003;
      dx(3) = 0.01;
      entry.second->update(dx);
    }
    Eigen::MatrixXd Hf, Hx;
    Eigen::VectorXd residual;
    std::vector<std::shared_ptr<ov_type::Type>> order;
    UpdaterHelper::get_feature_jacobian_full(s, f, Hf, Hx, residual, order);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(Hx.cols(), 4);
    int offset = 0;
    for (const auto &variable : order) {
      bool found = false;
      int index = 0;
      for (const auto &entry : s->_clones_IMU) {
        if (entry.second == variable) {
          N.block(offset, 0, 6, 4) = clone_gauges.at(index);
          found = true;
        }
        ++index;
      }
      require(found, "visual order contains only clones");
      offset += variable->size();
    }
    Eigen::MatrixXd Nf(3, 4);
    Nf.leftCols(3).setIdentity();
    Nf.col(3) = -skew_x(f.p_FinG_fej) * gamma;
    metric("native_visual_joint_HN", (Hx * N + Hf * Nf).norm() / std::max(1.0, Hx.norm() * N.norm() + Hf.norm() * Nf.norm()), 1e-10);
    UpdaterHelper::nullspace_project_inplace(Hf, Hx, residual);
    metric("native_visual_projected_HPhiN", (Hx * N).norm() / std::max(1.0, Hx.norm() * N.norm()), 1e-10);
    std::cout << "integration=" << integration << " PASS" << std::endl;
  }
}
