#include "ltv/observer/ltv_observer.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>

using Json = nlohmann::json;

Eigen::Vector3d vector3(const Json &value)
{
    return Eigen::Vector3d(value.at(0), value.at(1), value.at(2));
}

Json matrix(const Eigen::MatrixXd &value)
{
    Json result = Json::array();
    for (int row = 0; row < value.rows(); ++row)
    {
        Json entries = Json::array();
        for (int column = 0; column < value.cols(); ++column)
            entries.push_back(value(row, column));
        result.push_back(entries);
    }
    return result;
}

Json vector(const Eigen::VectorXd &value)
{
    Json result = Json::array();
    for (int row = 0; row < value.size(); ++row)
        result.push_back(value(row));
    return result;
}

int main(int argc, char **argv)
{
    if (argc != 3)
        throw std::runtime_error("usage: harness inputs.jsonl matrices.jsonl");
    std::ifstream inputs(argv[1]);
    std::ofstream matrices(argv[2]);
    if (!inputs || !matrices)
        throw std::runtime_error("cannot open fixture files");
    ltv::LtvObserver observer;
    const std::vector<int> knownIds = {10, 20, 30, 40, 50};
    int epoch = 0;
    std::string line;
    while (std::getline(inputs, line))
    {
        const Json event = Json::parse(line);
        const std::string op = event.at("op");
        Json design = {{"event", event.at("event")}, {"op", op}};
        if (op == "configure")
        {
            ltv::LtvConfig config;
            const Json c = event.at("config");
#define CONFIG(field) config.field = c.at(#field).get<decltype(config.field)>()
            CONFIG(enable); CONFIG(max_features); CONFIG(min_features);
            CONFIG(max_missed_frames); CONFIG(warmup_camera_updates);
            CONFIG(max_imu_dt); CONFIG(reset_gap); CONFIG(timestamp_tolerance);
            CONFIG(q_landmark); CONFIG(v_landmark); CONFIG(v_velocity); CONFIG(v_gravity);
            CONFIG(initial_p_landmark); CONFIG(initial_p_velocity); CONFIG(initial_p_gravity);
            CONFIG(covariance_floor); CONFIG(covariance_failure_threshold);
            CONFIG(camera_euler_safety); CONFIG(max_camera_substeps);
#undef CONFIG
            observer.configure(config);
            ++epoch;
        }
        else if (op == "start")
        {
            observer.start(event.at("imu_timestamp"));
            ++epoch;
        }
        else if (op == "reset")
        {
            observer.reset(ltv::LtvResetReason::EstimatorReset);
            ++epoch;
        }
        else if (op == "imu")
        {
            const Eigen::Vector3d omega = vector3(event.at("gyro")) - vector3(event.at("gyro_bias"));
            Eigen::Matrix3d skew;
            skew << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(), omega.x(), 0;
            const int d = observer.state().size();
            const int n = (d - 6) / 3;
            Eigen::MatrixXd a = Eigen::MatrixXd::Zero(d, d);
            Eigen::MatrixXd b = Eigen::MatrixXd::Zero(d, 3);
            for (int slot = 0; slot < n; ++slot)
            {
                a.block<3, 3>(3 * slot, 3 * slot) = -skew;
                a.block<3, 3>(3 * slot, 3 * n) = -Eigen::Matrix3d::Identity();
            }
            a.block<3, 3>(3 * n, 3 * n) = -skew;
            a.block<3, 3>(3 * n, 3 * n + 3) = Eigen::Matrix3d::Identity();
            a.block<3, 3>(3 * n + 3, 3 * n + 3) = -skew;
            b.block<3, 3>(3 * n, 0) = Eigen::Matrix3d::Identity();
            design["A_before_event"] = matrix(a);
            design["B_before_event"] = matrix(b);
            observer.propagateImu(event.at("dt"), vector3(event.at("acc")), vector3(event.at("gyro")),
                                  vector3(event.at("accel_bias")), vector3(event.at("gyro_bias")));
        }
        else if (op == "camera")
        {
            std::vector<ltv::LtvFeatureObservation> observations;
            for (const auto &item : event.at("observations"))
            {
                ltv::LtvFeatureObservation observation;
                observation.feature_id = item.at("id");
                observation.normalized_coordinate = vector3(item.at("coordinate"));
                observations.push_back(observation);
            }
            Eigen::Matrix3d rotation;
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    rotation(row, column) = event.at("R_BC").at(row).at(column);
            observer.updateFeatures(event.at("frame_timestamp"), event.at("imu_timestamp"), observations,
                                    rotation, vector3(event.at("p_BC")));
            // Descriptive matrix reconstruction; golden state/P below come only from source observer.
            std::map<int, Eigen::Vector3d> bySlot;
            for (const auto &observation : observations)
            {
                const int slot = observer.slotForFeature(observation.feature_id);
                if (slot >= 0 && observation.normalized_coordinate.allFinite() &&
                    observation.normalized_coordinate.norm() > 1e-12)
                    bySlot.emplace(slot, observation.normalized_coordinate);
            }
            Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3 * bySlot.size(), observer.state().size());
            Eigen::VectorXd y = Eigen::VectorXd::Zero(3 * bySlot.size());
            int index = 0;
            for (const auto &item : bySlot)
            {
                const Eigen::Vector3d bearing = rotation * item.second.normalized();
                const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity() - bearing * bearing.transpose();
                c.block<3, 3>(3 * index, 3 * item.first) = projection;
                y.segment<3>(3 * index) = projection * vector3(event.at("p_BC"));
                ++index;
            }
            design["C_reconstructed_after_lifecycle"] = matrix(c);
            design["y_reconstructed_after_lifecycle"] = vector(y);
        }
        else
            throw std::runtime_error("unknown event");

        const double frame = event.value("frame_timestamp", -1.0);
        const ltv::LtvSnapshot snapshot = observer.snapshot(frame);
        Json slots = Json::object();
        for (int id : knownIds)
            slots[std::to_string(id)] = observer.slotForFeature(id);
        Json result = {{"event", event.at("event")}, {"op", op}, {"adapter_epoch", epoch},
                       {"started", observer.started()}, {"imu_timestamp", snapshot.imu_timestamp},
                       {"frame_timestamp", snapshot.frame_timestamp}, {"slots", slots},
                       {"state", vector(observer.state())}, {"velocity_body", vector(snapshot.velocity_body)},
                       {"gravity_body", vector(snapshot.gravity_body)}, {"P", matrix(observer.covariance())},
                       {"state_features", snapshot.state_features}, {"observed_features", snapshot.observed_features},
                       {"healthy_camera_updates", snapshot.healthy_camera_updates}, {"valid", snapshot.valid},
                       {"velocity_valid", snapshot.velocity_valid}, {"gravity_valid", snapshot.gravity_valid},
                       {"innovation_norm", snapshot.innovation_norm}, {"camera_substeps", snapshot.camera_substeps},
                       {"reset_reason", ltv::toString(snapshot.last_reset_reason)}};
        std::cout << result.dump() << '\n';
        matrices << design.dump() << '\n';
    }
}
