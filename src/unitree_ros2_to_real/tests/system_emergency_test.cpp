#include "system_fsm_fixture.hpp"
#include <iostream>
int main(){auto p=system_fixture();
 // B latches before any publisher and outranks A/X even during release.
 for(auto phase:{SystemPhase::DISARMED,SystemPhase::PRECHECK,SystemPhase::SPORT_RELEASE_REQUIRED,SystemPhase::SPORT_RELEASE_VERIFIED}){
  R3Supervisor f(p);f.Observe(system_facts(0),time_at(0));f.Dispatch(SystemEvent::ADVANCE_PHASE,{},phase);
  std::array events{SystemEvent::REQUEST_A,SystemEvent::REQUEST_X,SystemEvent::REQUEST_B};f.DispatchEvents(events,time_at(0));
  assert(f.system_state()==SystemState::EMERGENCY_FAULT&&f.fault_latched()&&!f.NeedsPolicy()&&!f.Tick(time_at(0)));
  auto ports=f.ConsumePortRequests();assert(ports.arm_emergency&&ports.cancel_navigation&&ports.cancel_manipulation&&!ports.arm_return_home);
  f.Observe(system_facts(.1),time_at(.1));assert(!f.StartRemoteSequence(time_at(.1)).success);
 }
 for(auto phase:{SystemPhase::RL_ZERO,SystemPhase::STAND_TRANSITION,SystemPhase::HOLDING,SystemPhase::ARM_RETURN_HOME,SystemPhase::ARM_HOME_BLOCKED,SystemPhase::ARM_HOME_SETTLE,SystemPhase::PD_CAPTURE,SystemPhase::LIE_DOWN,SystemPhase::LIE_DOWN_VERIFY,SystemPhase::LIE_DOWN_BLOCKED,SystemPhase::LIE_DOWN_HOLD}){
  R3Supervisor f(p);enter_active(f);auto work=f.BeginPolicy(time_at(6));f.Dispatch(SystemEvent::ADVANCE_PHASE,{},phase);
  f.Emergency(time_at(6));assert(f.system_state()==SystemState::EMERGENCY_FAULT&&!f.NeedsPolicy());
  assert(!f.BeginPolicy(time_at(6)));if(work)assert(!f.PolicyResult(*work,p.stand,2,time_at(6)));
  f.ArmHomeRequestAccepted(true,time_at(6));auto packet=f.Tick(time_at(6));assert(packet&&f.AllowsPacket(*packet,time_at(6)));
  for(int j=0;j<12;++j)assert(packet->motor_cmd[j].kp==0&&packet->motor_cmd[j].kd==3&&packet->motor_cmd[j].dq==0&&packet->motor_cmd[j].tau==0);
  assert(!f.RequestRl(time_at(6)).success&&!f.StartRemoteSequence(time_at(6)).success);
  auto first=f.ConsumePortRequests();assert(first.arm_emergency&&!first.arm_return_home);f.Emergency(time_at(6));assert(!f.ConsumePortRequests().arm_emergency);
 }
 R3Supervisor ro(p,OperationProfile::READ_ONLY);ro.Emergency({});auto ports=ro.ConsumePortRequests();assert(!ports.arm_emergency&&!ports.cancel_navigation&&!ports.cancel_manipulation);
 std::cout<<"PASS B all global/stop phases, priority, latch before output, passive0/3, stale policy/HOME rejection, idempotent ports and read-only isolation\n";
}
