#define main fixture_main
#include "../../../ov_msckf/tests/ltv/test_ltv_adapter_hardened.cpp"
#undef main
#include "ltv/diagnostics/LtvPassiveCache.h"
#include <cstdint>
int main(int argc,char **argv){try{
 if(argc!=2)throw std::runtime_error("unique cache output path required");
 LtvOptions options;options.enabled=options.feature_readiness_enabled=options.passive_hardening_enabled=true;options.feature_seed_source="STEREO";options.feature_manager.max_active=15;
 Harness fixture(options);LtvPassiveCacheWriter writer(argv[1]);int imu=-1;bool active_retired=false,candidate_ttl=false,guard_rejected=false;size_t candidate_id=16;
 const size_t first=static_cast<size_t>(UINT64_C(0x8000000000000000));
 for(int k=0;k<=65;++k){const double t=k*.05;
  while(imu<=k*10){ov_core::ImuData s;s.timestamp=imu*.005;s.am=Eigen::Vector3d(0,0,9.81);s.wm.setZero();fixture.adapter.feed_imu(s);writer.imu(s);++imu;}
  // Exactly one right endpoint, retained for the next camera event.
  if(imu==k*10+1){ov_core::ImuData s;s.timestamp=imu*.005;s.am=Eigen::Vector3d(0,0,9.81);s.wm.setZero();fixture.adapter.feed_imu(s);writer.imu(s);++imu;}
  auto ctx=fixture.context(k,k<61||k>=64);
  if(k>=5&&k<61)ctx.observations.erase(std::remove_if(ctx.observations.begin(),ctx.observations.end(),[candidate_id](const ltv::HistoryObservation&o){return o.feature_id==candidate_id;}),ctx.observations.end());
  for(auto &o:ctx.observations)o.feature_id+=first;
  std::vector<LtvBearing> bearings;for(const auto&o:ctx.observations)if(o.camera_id==0)bearings.push_back({o.feature_id,o.bearing});
  auto f=fixture.adapter.process(t,bearings,fixture.cal,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),ctx.version,&ctx);
  if(f.available)require(fixture.adapter.claim(f),"claim failed");
  const auto&m=fixture.adapter.feature_frame().management;
  if(k==4){for(size_t id=0;id<16;++id)if(std::find(m.retained_ids.begin(),m.retained_ids.end(),first+id)==m.retained_ids.end())candidate_id=id;require(candidate_id<16&&m.retained_ids.size()==15,"expected one unadmitted TTL candidate");}
  for(const auto&e:m.retirement_events){require(e.feature_id>=first,"retirement ID truncated");if(e.kind==ltv::LandmarkRetirementKind::ActiveRetired)active_retired=true;else candidate_ttl=true;}
  for(auto id:m.identity_guard_rejected_ids){require(id>=first,"guard ID truncated");guard_rejected=true;}
  writer.process(t,bearings,fixture.cal,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),ctx.version,&ctx,f,fixture.adapter);
 }
 writer.finish();require(active_retired&&candidate_ttl&&guard_rejected,"fixture did not exercise every large-ID serialization category");
 auto result=replayLtvPassiveCache(argv[1],options);require(result.compared_events==66,"cache event mismatch");
 std::cout<<"PASS exact hardened cache with uint64 high-bit candidate TTL, active retirement and guarded IDs\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
