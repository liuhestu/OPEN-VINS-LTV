#define main manager_fixture_main
#include "../../../ov_msckf/tests/ltv/test_ltv_manager_bounded.cpp"
#undef main
#include "ltv/ltv_observer.h"
#include <limits>

static void core_checks() {
 LtvConfig config;config.enable=true;LtvObserver bounded,legacy;bounded.configure(config);legacy.configure(config);bounded.start(0);legacy.start(0);
 require(bounded.enableControlledFeatures(7,true)&&legacy.enableControlledFeatures(7),"core setup");
 LtvControlledFeatures c;c.epoch=7;c.imu_timestamp=0;std::vector<LtvFeatureObservation> z;
 for(int id:{9,8}){LtvLandmarkSeed s;s.feature_id=id;s.mean_body=Eigen::Vector3d(id,.2,3);c.births.push_back(s);c.retained_ids.push_back(id);z.push_back({id,s.mean_body.normalized()});}
 auto apply=[&](LtvObserver&o,double t,const LtvControlledFeatures&control,const std::vector<LtvFeatureObservation>&obs){return o.updateFeaturesControlled(t,t,obs,Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),control);};
 require(apply(bounded,0,c,z).accepted&&apply(legacy,0,c,z).accepted,"unordered batch above previous high water");
 require(bounded.admissionHighWaterMark()==9&&bounded.admissionHistorySize()==0&&legacy.admissionHistorySize()==2,"constant-space history distinct from legacy");
 require((bounded.state().array()==legacy.state().array()).all()&&(bounded.covariance().array()==legacy.covariance().array()).all(),"mode changes x/P");
 bounded.propagateImu(.01,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
 c.imu_timestamp=.01;c.births.clear();
 auto before=bounded.state();auto P=bounded.covariance();
 LtvLandmarkSeed invalid;invalid.feature_id=10;invalid.mean_body=Eigen::Vector3d(1,0,3);c.births.push_back(invalid);c.retained_ids.push_back(10);auto zz=z;zz.push_back({10,invalid.mean_body.normalized()});
 invalid.feature_id=11;invalid.mean_body.x()=std::numeric_limits<double>::quiet_NaN();c.births.push_back(invalid);c.retained_ids.push_back(11);zz.push_back({11,Eigen::Vector3d(0,0,1)});
 require(!apply(bounded,.01,c,zz).accepted,"malformed second birth accepted");
 require(bounded.admissionHighWaterMark()==9&&(before.array()==bounded.state().array()).all()&&(P.array()==bounded.covariance().array()).all(),"failed batch consumed high water/x/P");
 c.births.pop_back();c.retained_ids.pop_back();zz.pop_back();require(apply(bounded,.01,c,zz).accepted&&bounded.admissionHighWaterMark()==10,"rejected batch prevented legitimate retry");
 bounded.propagateImu(.01,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
 c.imu_timestamp=.02;c.births.clear();c.retained_ids.clear();require(apply(bounded,.02,c,{}).accepted,"retirement");
 bounded.propagateImu(.01,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
 for(int id:{8,9,10,7}){c.imu_timestamp=.03;c.births.clear();c.retained_ids={id};LtvLandmarkSeed s;s.feature_id=id;s.mean_body=Eigen::Vector3d(0,0,3);c.births.push_back(s);require(!apply(bounded,.03,c,{{id,Eigen::Vector3d(0,0,1)}}).accepted,"retired or skipped-below-H identity resurrected");}
 require(bounded.admissionHighWaterMark()==10&&bounded.admissionHistorySize()==0,"rejections corrupt high water");
}
static void manager_checks(){
 LandmarkManagerConfig legacy_small;legacy_small.history.max_candidates=1;LtvLandmarkManager legacy_compat(legacy_small);
 legacy_small.bounded_memory=true;bool limited=false;try{LtvLandmarkManager invalid(legacy_small);}catch(const std::invalid_argument&){limited=true;}require(limited,"bounded capacity configuration not enforced");
 LandmarkManagerConfig c;c.bounded_memory=true;c.max_active=15;c.history.max_candidates=30;LtvLandmarkManager m(c);m.reset(1);LandmarkManagerFrame f;
 for(int k=0;k<3;++k)f=m.step(1,k*.05,k,observations(0,30),seeds(1,k*.05,0,15));
 require(f.births.size()==15,"manager setup births");auto saved=m;
 for(int k=3;k<=5;++k)f=m.step(1,k*.05,k,{},{});
 require(f.retirement_events.size()==15&&f.active_retired_total==15&&f.candidate_ttl_total==0,"active retirement category");
 for(const auto&e:f.retirement_events)require(e.kind==LandmarkRetirementKind::ActiveRetired&&e.record.entered>=0,"active event metadata");
 f=m.step(1,2.2,6,{},{});require(f.retirement_events.size()==15&&f.candidate_ttl_total==15,"candidate TTL count");
 for(const auto&e:f.retirement_events)require(e.kind==LandmarkRetirementKind::NeverAdmittedCandidateTtl&&e.record.entered<0,"TTL event metadata");
 const auto bits=f.identity_guard_set_bits,insertions=f.identity_guard_insertions;auto bad=observations(0,1);bad[0].bearing.x()=std::numeric_limits<double>::quiet_NaN();
 require(!m.step(1,2.25,7,bad,{}).accepted_input&&m.history().time()==2.2,"guarded old ID hid invalid packet");
 f=m.step(1,2.25,7,observations(0,30),seeds(1,2.25,0,30));
 require(f.births.empty()&&f.identity_guard_rejected_ids.size()==30&&f.exact_records==0,"old external IDs reseeded after exact records erased");
 require(f.identity_guard_insertions==insertions&&f.identity_guard_set_bits==bits,"reject/query changed membership bits");
 require(saved.landmarks().size()==30&&saved.landmarks().at(0).entered==.1,"copied guard/map state not independent");
 for(int k=0;k<3;++k)f=m.step(1,2.3+k*.05,8+k,observations(100,15),seeds(1,2.3+k*.05,100,15));
 require(f.births.size()==15&&f.admitted_tracks==30&&f.exact_records==15,"new lower/unrelated IDs incorrectly blocked");
 require(!m.step(0,2.5,11,{},{}).accepted_input&&m.history().time()==2.4,"stale epoch clears protection");
}
int main(){try{core_checks();manager_checks();std::cout<<"PASS independent bounded core watermark/atomic batch/legacy xP/typed retirement/external ID protection\n";}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
