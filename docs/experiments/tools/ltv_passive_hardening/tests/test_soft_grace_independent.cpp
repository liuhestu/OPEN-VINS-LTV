#define main earlier_independent_main
#include "test_readiness_independent.cpp"
#undef main
#include <cmath>

static LtvReadinessConfig config(bool grace) {
  LtvReadinessConfig c; c.enabled=true;c.initial_unseeded_warmup=true;c.preserve_constrained_state=true;c.ready_soft_grace=grace;return c;
}
static double ready(LtvReadiness &r,double base){
  auto a=input(base);auto o=r.update(a);check(o.request_bootstrap,"warmup request");a.bootstrap_source="BOUNDED_INITIAL_ZERO_WARMUP";r.acknowledgeBootstrap(a);
  for(int k=1;k<=20;++k){o=r.update(input(base+.05*k));check(o.joint_ready==(k==20),"exact20 real-good confirmations");}return base+1.;
}
int main(){try{
 for(double base:{0.,1413394887.3557606}) {
  LtvReadiness r(config(true));double good=ready(r,base);auto s=input(good+.05);s.prediction_angle_p95_rad=.03;auto o=r.update(s);check(o.joint_ready,"first soft hold");
  const double deadline=std::nextafter(good+.10,std::numeric_limits<double>::infinity());s=input(deadline);s.prediction_angle_p95_rad=.04;o=r.update(s);check(o.joint_ready,"oneULP deadline is inclusive");
  s=input(std::nextafter(deadline,std::numeric_limits<double>::infinity()));s.prediction_angle_p95_rad=.03;o=r.update(s);check(!o.ready_G&&!o.ready_V,"third soft never holds");
  for(int k=1;k<=20;++k){o=r.update(input(s.time+.05*k));check(o.joint_ready==(k==20),"revoked branch requires20good");}
  LtvReadiness expired(config(true));good=ready(expired,base);s=input(std::nextafter(std::nextafter(good+.10,INFINITY),INFINITY));s.prediction_angle_p95_rad=.03;o=expired.update(s);check(!o.joint_ready,"firstsoft after deadline rejected");
 }
 {LtvReadiness r(config(true));double good=ready(r,0.);for(int k=1;k<=3;++k){auto s=input(good+.02*k);s.gravity_correction_rate=1.5;auto o=r.update(s);check(o.joint_ready==(k<3),"thirdsoft inside .10 rejected");}}
 {LtvReadiness r(config(true));double good=ready(r,0.);auto s=input(good+.05);s.velocity_correction_rate=.75;auto o=r.update(s);check(o.ready_G&&o.ready_V,"V-only grace");s=input(good+.10);s.velocity_correction_rate=std::nextafter(1.,INFINITY);o=r.update(s);check(o.ready_G&&!o.ready_V,"V excess cannot revoke G");s=input(good+.15);s.gravity_correction_rate=1.5;o=r.update(s);check(o.ready_G&&!o.ready_V,"G hold cannot resurrect V");}
 for(int fault=0;fault<5;++fault){LtvReadiness r(config(true));double good=ready(r,0.);auto s=input(good+.05);s.prediction_angle_p95_rad=.03;check(r.update(s).joint_ready,"precondition grace");s=input(good+.10);if(fault==0)s.observed_features=14;if(fault==1)s.mature_observed_features=14;if(fault==2)s.physical_input_fault=true;if(fault==3)s.gravity.setZero();if(fault==4)s.velocity_correction_rate=std::numeric_limits<double>::quiet_NaN();auto o=r.update(s);check(!o.ready_G&&!o.ready_V,"hardfault duringgrace must revoke both");}
 {LtvReadiness r(config(false));double good=ready(r,0.);auto s=input(good+.05);s.prediction_angle_p95_rad=.021;check(!r.update(s).joint_ready,"OFF has immediate original rejection");}
 std::cout<<"PASS independent grace epoch deadline/nextfloat/thirdsoft/20reconfirm/branch isolation/hardfault/OFF\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
