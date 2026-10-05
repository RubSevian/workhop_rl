#include "system_fsm_fixture.hpp"
#include <iostream>
int main(){auto p=system_fixture();for(auto op:{OperationProfile::RL_ZERO_TEST,OperationProfile::REMOTE_TEST,OperationProfile::FULL_MISSION}){
 R3Supervisor f(p,op);auto i=system_facts(0);i.navigation_ready=i.perception_ready=i.arm_emergency_validated=true;
 f.Observe(i,time_at(0));assert(f.Takeover(time_at(0)).success);f.Observe(i,time_at(0));assert(f.EnableOutput(true,time_at(0)).success);
 f.Tick(time_at(0));assert(f.RequestStand(time_at(0)).success);i=system_facts(6);i.navigation_ready=i.perception_ready=i.arm_emergency_validated=true;f.Observe(i,time_at(6));f.Tick(time_at(6));
 assert(f.RequestRl(time_at(6)).success&&f.ConsumePolicyReset());
 i=system_facts(6.002,false);f.Observe(i,time_at(6.002));assert(!f.fault_latched()&&f.Blockers(time_at(6.002)).empty());
 assert(!f.RequestHold(time_at(6.002)).success&&!f.fault_latched());
 auto w=f.BeginPolicy(time_at(6.002));assert(w&&f.PolicyResult(*w,p.stand,2,time_at(6.002)));assert(f.Tick(time_at(6.002))->motor_cmd[0].kp==25);
 i=system_facts(6.004,false);i.arm_control_ready=false;f.Observe(i,time_at(6.004));assert(f.system_state()==SystemState::EMERGENCY_FAULT&&!f.NeedsPolicy());
 }
 R3Supervisor takeover(p);auto away=system_facts(0,false);takeover.Observe(away,{});assert(!takeover.StartRemoteSequence({}).success);
 R3Supervisor capture(p);capture.Observe(system_facts(0),{});capture.Takeover({});capture.Observe(system_facts(0),{});capture.EnableOutput(true,{});
 capture.Observe(away,{});assert(capture.fault_latched()); // PD takeover still requires HOME
 std::cout<<"PASS runtime arm away HOME allowed in RL_ZERO/REMOTE/FULL, control health remains critical, takeover still HOME-gated\n";
}
