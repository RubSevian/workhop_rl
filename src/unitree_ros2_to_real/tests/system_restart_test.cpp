#include "system_fsm_fixture.hpp"
#include <iostream>
int main(){auto p=system_fixture();R3Supervisor f(p);enter_active(f);f.ControlledAbort(time_at(6));f.ArmHomeRequestAccepted(true,time_at(6));
 for(int n=1;n<100;++n){double s=6+n*.002;auto i=system_facts(s);i.measured_q=*p.lie_down;assert(run_tick(f,s,i));}
 assert(f.system_state()==SystemState::SYSTEM_HOLD);
 auto bad=system_facts(7);bad.measured_q=*p.lie_down;bad.own_output_healthy=false;f.Observe(bad,time_at(7));assert(!f.StartRemoteSequence(time_at(7)).success);
 bad.own_output_healthy=true;bad.sport_stamp=time_at(6);f.Observe(bad,time_at(7));assert(!f.StartRemoteSequence(time_at(7)).success); // stale Sport latches existing watchdog
 // Use a fresh system after the intentionally critical stale Sport case.
 R3Supervisor g(p);enter_active(g);g.ControlledAbort(time_at(6));g.ArmHomeRequestAccepted(true,time_at(6));
 for(int n=1;n<100;++n){double s=6+n*.002;auto i=system_facts(s);i.measured_q=*p.lie_down;run_tick(g,s,i);}
 auto i=system_facts(7);for(int j=0;j<12;++j)i.measured_q[j]=(*p.lie_down)[j]+.03F;
 g.Observe(i,time_at(7));assert(g.Dispatch(SystemEvent::REQUEST_A,time_at(7)).success&&g.output_enabled());
 auto packet=g.Tick(time_at(7));for(int j=0;j<12;++j)assert(packet->motor_cmd[j].q==i.measured_q[j]);
 int releases=0,enables=0,resets=0;double stand_at=0,hold_at=0,rl_at=0;
 for(int n=0;n<5150;++n){double s=7+n*.002;auto input=system_facts(s);for(int j=0;j<12;++j)input.measured_q[j]=(*p.lie_down)[j]+.04F;
  g.Observe(input,time_at(s));packet=g.Tick(time_at(s));assert(packet&&!g.fault_latched());
  if(g.phase()==SystemPhase::HOLDING&&hold_at==0)hold_at=s;
  if(n%10==0){auto action=g.RemoteSequenceNext(time_at(s));
   if(action==R3SequenceAction::RELEASE_SPORT)++releases;if(action==R3SequenceAction::ENABLE_OUTPUT)++enables;
   if(action==R3SequenceAction::STAND){stand_at=s;assert(g.RequestStand(time_at(s)).success);auto first=g.Tick(time_at(s));for(int j=0;j<12;++j)assert(first->motor_cmd[j].q==input.measured_q[j]);}
   if(action==R3SequenceAction::RL){rl_at=s;assert(g.RequestRl(time_at(s)).success);}
   if(g.ConsumePolicyReset())++resets;
   if(g.NeedsPolicy()){auto work=g.BeginPolicy(time_at(s));if(work)assert(g.PolicyResult(*work,p.stand,2,time_at(s)));}
  }
 }
 assert(releases==0&&enables==0&&resets==1&&stand_at>=7.02&&std::abs(hold_at-stand_at-6)<.003&&rl_at-hold_at>=4&&rl_at-hold_at<4.03);
 assert(g.system_state()==SystemState::ACTIVE&&g.NeedsPolicy());
 std::cout<<"PASS SYSTEM_HOLD A: healthy own output, fresh Sport/measured capture, no release/enable, 6s stand/4s hold/one reset\n";
}
