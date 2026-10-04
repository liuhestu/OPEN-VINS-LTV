#include "ltv/landmark_adapter/LtvActiveConsistency.h"
#include "ltv/landmark_adapter/LtvFeaturePipeline.h"
#include <iostream>
#include <stdexcept>
using namespace ltv;
static void require(bool v, const char *why) {
  if (!v) throw std::runtime_error(why);
}
int main() {
  // A previously legal short observation history cannot become invalid with the new feature OFF.
  FeaturePipelineConfig off;
  off.manager.history.history_window = .1;
  off.active_consistency.history_window_s = .1;
  LtvFeaturePipeline disabled(off);
  ActiveConsistencyConfig config;
  config.enabled = true;
  LtvActiveConsistency eval(config);
  FeaturePipelineContext ctx;
  ctx.epoch = 1;
  ctx.version = 10;
  ctx.time = 1;
  ctx.cameras.push_back(SeedCamera{});
  FeatureTrackHistory history;
  Eigen::Vector3d point(.3,.2,4);
  for (int i=0;i<7;++i) {
    SeedPose pose;
    pose.t=.25+i*.125;
    pose.p_WB=Eigen::Vector3d(.1*i,.02*i*i,0);
    ctx.poses.push_back(pose);
    HistoryObservation observation;
    observation.feature_id=17;
    observation.bearing=(point-pose.p_WB).normalized();
    if (i<6) history.samples.push_back({pose.t,static_cast<uint64_t>(i),{observation}});
    else ctx.observations.push_back(observation);
  }
  ctx.execution_pose_index=6;
  const auto clean=eval.evaluate(17,1,history,ctx);
  require(clean.evaluable && clean.pass && clean.history_count==6,"clean fixture");
  auto negative=history;
  for(auto &s:negative.samples) s.observations.front().bearing*=-1;
  const auto n=eval.evaluate(17,1,negative,ctx);
  require(!n.evaluable && n.pass && n.reason==ActiveConsistencyReason::FitFailed && n.fit_reason=="NONPOSITIVE_RAY", "negative signed rays are unavailable, not fabricated history failure");
  // Large history residual is an evaluable failure, never swallowed as FitFailed.
  auto inconsistent=history;
  for(size_t i=0;i<inconsistent.samples.size();++i) {
    auto &z=inconsistent.samples[i].observations.front().bearing;
    z.y()+=i%2?-.10:.10;
    z.normalize();
  }
  const auto bad=eval.evaluate(17,1,inconsistent,ctx);
  std::cout << "history fixture reason=" << toString(bad.reason) << " solver=" << bad.fit_reason << " residual=" << bad.history_max_residual_rad << std::endl;
  require(bad.evaluable && !bad.pass && bad.reason==ActiveConsistencyReason::HistoryInconsistent,"history inconsistency classified separately");
  auto altered=ctx;
  altered.observations.front().bearing=Eigen::Vector3d::UnitX();
  const auto held=eval.evaluate(17,1,history,altered);
  require((held.point_W-clean.point_W).norm()==0,"current ray leaked into past solution");
  auto poison=history;
  auto event=history.samples.front();
  event.time=1;
  event.observations.front().bearing=Eigen::Vector3d::UnitY();
  poison.samples.push_back(event);
  event.time=1.1;
  poison.samples.push_back(event);
  const auto unchanged=eval.evaluate(17,1,poison,altered);
  require((held.point_W-unchanged.point_W).norm()==0 && unchanged.history_count==6,"current/future samples entered past support");
  std::cout<<"independent active consistency tests PASS\n";
}
