#define main earlier_readiness_main
#include "test_readiness_independent.cpp"
#undef main
int main(){try{
 LtvReadinessConfig c;c.enabled=true;c.initial_unseeded_warmup=true;c.preserve_constrained_state=true;c.ready_soft_grace=true;
 for(bool backward:{false,true}){
  LtvReadiness r(c);auto a=input(0);check(r.update(a).request_bootstrap,"initial request");a.bootstrap_source="BOUNDED_INITIAL_ZERO_WARMUP";r.acknowledgeBootstrap(a);
  for(int k=1;k<=20;++k)check(r.update(input(.05*k)).joint_ready==(k==20),"initial20 confirmations");
  auto soft=input(1.05);soft.prediction_angle_p95_rad=.03;check(r.update(soft).grace_G,"latched grace precondition");
  auto pause=input(backward?1.:1.05);pause.observer_started=false;pause.physical_input_fault=true;
  auto out=r.update(pause);check(!out.accepted_time&&!out.ready_G&&!out.ready_V&&!out.grace_G&&!out.grace_V,"invalid pause must not publish ready/grace");
  // Adapter has reset its core: next epoch enters policy with a genuinely cold core.
  auto cold=input(1.10);cold.observer_started=false;out=r.update(cold);check(!out.ready_G&&!out.ready_V&&!out.request_bootstrap,"new epoch cannot inherit latch");
  for(double t:{1.15,1.20,30.,30.05,30.10}){cold=input(t);cold.observer_started=false;out=r.update(cold);check(!out.joint_ready,"cold core cannot ready");}
  check(out.request_bootstrap,"eventual legitimate bootstrap request");auto ack=input(30.10);ack.bootstrap_source="ZERO_DELAYED_START";out=r.acknowledgeBootstrap(ack);check(!out.joint_ready&&out.soft_failure_frames_G==0&&out.soft_failure_frames_V==0,"ack clears confirmations/grace");
  for(int k=1;k<=20;++k){out=r.update(input(30.10+.05*k));check(out.joint_ready==(k==20),"new epoch needs full20good");}
 }
 std::cout<<"PASS invalid/duplicate pause returns no-ready; cold new epoch cannot inherit grace and needs20good\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
