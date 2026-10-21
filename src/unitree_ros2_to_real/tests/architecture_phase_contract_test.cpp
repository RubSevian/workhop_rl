// Synthetic FSM/packet contracts only. No DDS, SDK connection or serial IO.
#include "r3_commissioning.hpp"
#include <cassert>
#include <algorithm>
#include <iostream>
using namespace sim2real;
SafetyTime time_at(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
R3Inputs healthy(double s){
 R3Inputs in;in.ready={true,true,true,true,true,true,true,true,false,true};
 in.measured_q.fill(.3F);in.sport=SportMode::RELEASED;
 in.sport_stamp=in.lowstate_stamp=in.remote_stamp=in.arm_stamp=in.target_stamp=time_at(s);
 in.arm_static_hold=in.arm_home_ready=in.arm_control_ready=in.navigation_ready=in.perception_ready=in.own_output_healthy=true;
 return in;
}
void start(R3Supervisor& s){
 s.Observe(healthy(100),time_at(100));assert(s.StartRemoteSequence(time_at(100)).success);
 s.Observe(healthy(100),time_at(100));
 assert(s.EnableOutput(true,time_at(100)).success);assert(s.Tick(time_at(100)));
 assert(s.RequestStand(time_at(100)).success);
}
void pd_phase_contract(const R3Profile& p){
 R3Supervisor s(p,OperationProfile::FULL_MISSION);start(s);
 for(int tick=1;tick<=5000;++tick){
  double t=100+tick*.002;auto in=healthy(t);in.navigation_ready=in.perception_ready=false;
  in.arm_stamp=in.target_stamp=time_at(100);in.ready.arm_feedback_ready=in.ready.arm_target_ready=false;
  in.arm_control_ready=in.arm_home_ready=in.arm_static_hold=false;
  s.Observe(in,time_at(t));auto packet=s.Tick(time_at(t));
  assert(packet);assert(!s.fault_latched());assert(packet->motor_cmd[0].kp==p.kp[0]);
 }
 assert(s.state()==R3State::HOLDING);assert(!s.RequestRl(time_at(110)).success);
 // A restored fresh HOME is necessary even if the leg trajectory completed.
 auto in=healthy(110.002);in.navigation_ready=in.perception_ready=false;s.Observe(in,time_at(110.002));
 assert(s.RequestRl(time_at(110.002)).success);assert(s.ConsumePolicyReset());
 in=healthy(110.004);in.ready.arm_feedback_ready=false;s.Observe(in,time_at(110.004));
 assert(s.fault_latched());assert(s.fault_phase()==SystemPhase::RL_ZERO);
 assert(std::find(s.fault_blockers().begin(),s.fault_blockers().end(),"arm_feedback_ready")!=s.fault_blockers().end());
 auto first=s.last_fault();s.Fault("consequence",time_at(111));assert(s.last_fault()==first);
}
void critical_sources(const R3Profile& p){
 for(bool arm_fault:{false,true}){
  R3Supervisor s(p,OperationProfile::FULL_MISSION);start(s);auto in=healthy(101);
  if(arm_fault)in.arm_fault_confirmed=true;else in.lowstate_stamp=time_at(100.499);
  s.Observe(in,time_at(101));assert(s.fault_latched());auto packet=s.Tick(time_at(101));assert(packet);
  for(int j=0;j<12;++j){assert(packet->motor_cmd[j].kp==0);assert(packet->motor_cmd[j].kd==3);}
 }
 R3Supervisor s(p,OperationProfile::FULL_MISSION);auto in=healthy(100);in.perception_ready=false;
 s.Observe(in,time_at(100));assert(!s.StartRemoteSequence(time_at(100)).success);assert(!s.output_enabled());
}
void emergency_priority(const R3Profile& p){
 for(auto phase:{SystemPhase::PRECHECK,SystemPhase::STAND_TRANSITION,SystemPhase::HOLDING,
       SystemPhase::RL_ZERO,SystemPhase::RL_ACTIVE,SystemPhase::ARM_RETURN_HOME,
       SystemPhase::PD_CAPTURE,SystemPhase::LIE_DOWN,SystemPhase::LIE_DOWN_PASSIVE}){
  R3Supervisor s(p,OperationProfile::FULL_MISSION);start(s);
  s.Dispatch(SystemEvent::ADVANCE_PHASE,time_at(100),phase);
  const std::array<SystemEvent,3> requests{SystemEvent::REQUEST_A,SystemEvent::REQUEST_X,SystemEvent::REQUEST_B};
  assert(s.DispatchEvents(requests,time_at(100)).success);assert(s.fault_latched());
  assert(s.system_state()==SystemState::EMERGENCY_FAULT);
  assert(!s.Dispatch(SystemEvent::REQUEST_A,time_at(100)).success);
  const auto ports=s.ConsumePortRequests();assert(ports.arm_emergency);assert(!ports.arm_return_home);
 }
 R3Supervisor s(p,OperationProfile::FULL_MISSION);
 assert(s.Dispatch(SystemEvent::REQUEST_B,time_at(100)).success);assert(s.fault_latched());
 assert(!s.output_enabled());assert(!s.Tick(time_at(100)));
}
int main(int argc,char**argv){
 assert(argc==2);auto p=LoadR3Profile(YAML::LoadFile(argv[1]));
 assert(p.lowstate_timeout_s==.5&&p.arm_timeout_s==.25&&p.policy_observation_timeout_s==.06);
 assert(p.policy_inference_timeout_s==.04&&p.policy_result_timeout_s==.04);
 pd_phase_contract(p);critical_sources(p);emergency_priority(p);
 std::cout<<"PASS v4 phase contracts, critical input separation, fresh RL boundary, first cause, B priority; no hardware\n";
}
