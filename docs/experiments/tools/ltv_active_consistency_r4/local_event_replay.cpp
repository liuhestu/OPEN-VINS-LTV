// TEST ONLY: cached after-lifecycle anchor, not a production state setter.
#include "ltv/landmark_adapter/LtvActiveConsistency.h"
#include "ltv/landmark_adapter/LtvFeaturePipeline.h"
#include "ltv/observer/ltv_observer.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace ltv;
static Eigen::Vector3d vec(const json &j) { return Eigen::Vector3d(j.at(0), j.at(1), j.at(2)); }
static Eigen::Matrix3d rot(const json &j) {
  Eigen::Matrix3d r;
  for (int a = 0; a < 3; ++a)
    for (int b = 0; b < 3; ++b)
      r(a, b) = j.at(3 * a + b);
  return r;
}
static Eigen::VectorXd vector(const json &j) {
  Eigen::VectorXd r(j.size());
  for (size_t k = 0; k < j.size(); ++k)
    r(k) = j[k];
  return r;
}
static Eigen::MatrixXd matrix(const json &j) {
  Eigen::MatrixXd r(j.size(), j.at(0).size());
  for (int a = 0; a < r.rows(); ++a)
    for (int b = 0; b < r.cols(); ++b)
      r(a, b) = j[a][b];
  return r;
}
static json pack(const Eigen::VectorXd &x) {
  json j = json::array();
  for (int k = 0; k < x.size(); ++k)
    j.push_back(x(k));
  return j;
}
static json packmat(const Eigen::MatrixXd &x) {
  json j = json::array();
  for (int k = 0; k < x.rows(); ++k) {
    json row = json::array();
    for (int l = 0; l < x.cols(); ++l)
      row.push_back(x(k, l));
    j.push_back(row);
  }
  return j;
}
static void require(bool v, const char *why) {
  if (!v)
    throw std::runtime_error(why);
}
int main(int argc, char **argv) {
  try {
    require(argc == 3, "input/output paths required");
    json j;
    std::ifstream(argv[1]) >> j;
    const auto &e = j.at("event"), &raw = j.at("actual"), &cj = j.at("context");
    double t = e.at("time"), previous = e.at("predecessor_time");
    int target = j.at("feature_id");
    FeaturePipelineContext ctx;
    ctx.time = cj.at("time");
    ctx.epoch = cj.at("epoch");
    ctx.version = cj.at("version");
    ctx.execution_pose_index = cj.at("execution_pose_index");
    for (const auto &p : cj.at("poses")) {
      SeedPose s;
      s.t = p.at("time");
      s.R_WB = rot(p.at("R_WB"));
      s.p_WB = vec(p.at("p_WB"));
      ctx.poses.push_back(s);
    }
    for (const auto &c : cj.at("cameras")) {
      SeedCamera s;
      s.R_BC = rot(c.at("R_BC"));
      s.p_BC = vec(c.at("p_BC"));
      ctx.cameras.push_back(s);
    }
    auto observation = [](const json &a) {
      HistoryObservation o;
      o.feature_id = a.at("id");
      o.camera_id = a.at("camera");
      o.bearing = vec(a.at("bearing"));
      o.match_valid = a.at("valid").get<int>() != 0;
      return o;
    };
    for (const auto &o : cj.at("observations"))
      ctx.observations.push_back(observation(o));
    FeatureTrackHistory h;
    for (const auto &p : j.at("history")) {
      HistorySample s;
      s.time = p.at("time");
      s.context_version = p.at("version");
      for (const auto &o : p.at("observations"))
        s.observations.push_back(observation(o));
      h.samples.push_back(s);
    }
    ActiveConsistencyConfig ac;
    ac.enabled = true;
    LtvActiveConsistency gate(ac);
    auto decision = gate.evaluate(target, t, h, ctx);
    require(decision.evaluable && !decision.pass, "known target must be causally rejected");
    auto cfgj = j.at("core_config");
    LtvConfig cfg;
    cfg.enable = true;
#define SET(k) cfg.k = cfgj.at(#k)
    SET(max_imu_dt);
    SET(reset_gap);
    SET(q_landmark);
    SET(v_landmark);
    SET(v_velocity);
    SET(v_gravity);
    SET(initial_p_landmark);
    SET(covariance_floor);
    SET(covariance_failure_threshold);
    SET(camera_euler_safety);
    SET(max_camera_substeps);
#undef SET
    Eigen::VectorXd x = vector(e.at("after_lifecycle_x"));
    Eigen::MatrixXd P = matrix(e.at("after_lifecycle_P"));
    Eigen::VectorXd expected = vector(raw.at("x"));
    Eigen::MatrixXd expectedP = matrix(j.at("actual_P"));
    auto R = rot(raw.at("calibration").at("R_BC"));
    auto p = vec(raw.at("calibration").at("p_BC"));
    std::vector<LtvFeatureObservation> obs;
    for (const auto &o : raw.at("observations")) {
      LtvFeatureObservation z;
      z.feature_id = o.at("id");
      z.normalized_coordinate = vec(o.at("bearing"));
      obs.push_back(z);
    }
    LtvObserver anchor;
    anchor.configure(cfg);
    anchor.start(previous);
    uint64_t epoch = e.at("epoch");
    require(anchor.enableControlledFeatures(epoch), "controlled enable");
    LtvControlledFeatures ctl;
    ctl.epoch = epoch;
    ctl.imu_timestamp = previous;
    for (const auto &id : e.at("slots")) {
      ctl.retained_ids.push_back(id);
      LtvLandmarkSeed s;
      s.feature_id = id;
      s.apply_mean = false;
      ctl.births.push_back(s);
    }
    // The controlled birth contract requires visible IDs even for allocation.
    // These dummy rays are used only on the first, zero-duration camera event;
    // the complete cached x/P replaces the allocation before either experiment.
    std::vector<LtvFeatureObservation> allocation_observations;
    for (int id : ctl.retained_ids) {
      LtvFeatureObservation z;
      z.feature_id = id;
      z.normalized_coordinate = Eigen::Vector3d::UnitZ();
      allocation_observations.push_back(z);
    }
    require(anchor.updateFeaturesControlled(previous, previous, allocation_observations, R, p, ctl).accepted, "install slots");
    ctl.births.clear();
    double cursor = previous;
    while (cursor < t) {
      double next = std::min(t, cursor + .005);
      anchor.propagateImu(next - cursor, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero());
      cursor = next;
    }
    require(anchor.started() && anchor.state().size() == x.size(), "anchor dimensions/time");
    // Object is nonconst. Only this standalone test injects the full validated
    // after-lifecycle snapshot; no production setter or seed/mean patch is added.
    const_cast<Eigen::VectorXd &>(anchor.state()) = x;
    const_cast<Eigen::MatrixXd &>(anchor.covariance()) = P;
    for (size_t k = 0; k < ctl.retained_ids.size(); ++k)
      require(anchor.slotForFeature(ctl.retained_ids[k]) == static_cast<int>(k), "exact slot order");
    ctl.imu_timestamp = t;
    LtvObserver oldcore = anchor, newcore = anchor;
    auto old = oldcore.updateFeaturesControlled(t, t, obs, R, p, ctl);
    require(old.accepted, "original update accepted");
    double xerr = (oldcore.state() - expected).norm() / std::max(1., expected.norm()),
           perr = (oldcore.covariance() - expectedP).norm() / std::max(1., expectedP.norm());
    require(xerr <= 1e-9 && perr <= 1e-9, "actual cached full x/P reproduction");
    require(old.snapshot.camera_substeps == e.at("recorded_substeps"), "actual substeps");
    std::vector<LtvFeatureObservation> filtered;
    for (const auto &z : obs)
      if (z.feature_id != target)
        filtered.push_back(z);
    require(filtered.size() + 1 == obs.size(), "target uniquely removed from observations");
    require((newcore.state() - x).norm() == 0 && (newcore.covariance() - P).norm() == 0, "new input exact complete anchor");
    auto changed = newcore.updateFeaturesControlled(t, t, filtered, R, p, ctl);
    require(changed.accepted && newcore.slotForFeature(target) >= 0, "first skip retains slot");
    Eigen::VectorXd delta = newcore.state() - x;
    require((x + delta - newcore.state()).norm() < 1e-10, "prediction plus correction telescopes");
    json out = {{"camera_ns", e.at("camera_ns")},
                {"feature_id", target},
                {"scope", "SINGLE_EVENT_AFTER_LIFECYCLE_SNAPSHOT_TARGET_ONLY_FIRST_SKIP"},
                {"reason", toString(decision.reason)},
                {"history_count", decision.history_count},
                {"history_span", decision.history_span_s},
                {"history_max_residual", decision.history_max_residual_rad},
                {"holdout_residual", std::isfinite(decision.holdout_residual_rad) ? json(decision.holdout_residual_rad) : json(nullptr)},
                {"old_x_relative_error", xerr},
                {"old_P_relative_error", perr},
                {"old_substeps", old.snapshot.camera_substeps},
                {"new_substeps", changed.snapshot.camera_substeps},
                {"slots", ctl.retained_ids},
                {"target_retained", true},
                {"old_observations", obs.size()},
                {"new_observations", filtered.size()},
                {"pre_x", pack(x)},
                {"old_x", pack(oldcore.state())},
                {"new_x", pack(newcore.state())},
                {"new_P", packmat(newcore.covariance())},
                {"new_delta", pack(delta)}};
    std::ofstream(argv[2]) << out.dump(2) << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
