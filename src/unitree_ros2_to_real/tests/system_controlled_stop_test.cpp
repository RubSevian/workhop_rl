#include "system_fsm_fixture.hpp"
#include <iostream>
int main(){auto p=system_fixture();R3Supervisor f(p);enter_active(f);
 assert(f.RemoteTestCommand({.1,0,0},time_at(6)).success);
 assert(f.Dispatch(SystemEvent::REQUEST_X,time_at(6)).success);
 assert(f.system_state()==SystemState::CONTROLLED_STOP&&f.NeedsPolicy());assert((f.command()==std::array<double,3>{}));
 auto ports=f.ConsumePortRequests();assert(ports.cancel_navigation&&ports.cancel_manipulation&&ports.arm_return_home);
 assert(!f.ConsumePortRequests().arm_return_home);assert(f.ControlledAbort(time_at(6)).success&&!f.ConsumePortRequests().arm_return_home);
 assert(!f.RemoteTestCommand({.1,0,0},time_at(6)).success&&!f.NavigationCommand({.1,0,0},time_at(6)).success);
 assert(!f.RemoteTestCommand({},time_at(6)).success&&!f.RequestHold(time_at(6)).success);
 for(int n=1;n<=50;++n){double s=6+n*.002;auto packet=run_tick(f,s,system_facts(s,false));assert(packet&&packet->motor_cmd[0].kp==25&&f.NeedsPolicy());}
 assert(f.phase()==SystemPhase::ARM_HOME_BLOCKED&&f.stop_blocker()=="arm_home_timeout"&&!f.fault_latched());
 f.ArmHomeRequestAccepted(true,time_at(6.10));
 auto home=system_facts(6.102);home.measured_q.fill(.41F);run_tick(f,6.102,home);assert(f.NeedsPolicy());
 double captured=0;std::array<float,12> handoff{};
 for(int n=52;n<=70;++n){double s=6+n*.002;auto i=system_facts(s);for(int j=0;j<12;++j)i.measured_q[j]=.4F+j*.01F;
  auto packet=run_tick(f,s,i);assert(packet);if(f.phase()==SystemPhase::PD_CAPTURE){captured=s;handoff=i.measured_q;
   for(int j=0;j<12;++j)assert(packet->motor_cmd[j].q==i.measured_q[j]&&packet->motor_cmd[j].kp==40&&packet->motor_cmd[j].kd==1);
   assert(!f.NeedsPolicy());break;}}
 assert(captured>0);
 for(int n=1;n<=60;++n){double s=captured+n*.002;auto i=system_facts(s);float u=std::clamp(float((s-captured)/p.lie_down_s),0.F,1.F);
  for(int j=0;j<12;++j)i.measured_q[j]=handoff[j]*(1-u)+(*p.lie_down)[j]*u;
  auto packet=run_tick(f,s,i);
  if(!packet){assert(f.phase()==SystemPhase::LIE_DOWN_OUTPUT_STOPPING&&!f.output_enabled());break;}
  assert(f.AllowsPacket(*packet,time_at(s)));
  for(int j=0;j<12;++j){assert(std::abs(packet->motor_cmd[j].q-i.measured_q[j])<1e-5);assert(packet->motor_cmd[j].kp==40&&packet->motor_cmd[j].kd==1);}}
 assert(f.system_state()==SystemState::CONTROLLED_STOP&&!f.output_enabled()&&!f.NeedsPolicy()&&!f.output_stopped());
 f.EnableOutput(false,time_at(captured+.120));f.ConfirmOutputStopped();
 assert(f.system_state()==SystemState::SYSTEM_HOLD&&f.output_stopped());
 assert(f.ControlledAbort(time_at(captured+.120)).success&&f.system_state()==SystemState::SYSTEM_HOLD);
 auto i=system_facts(captured+.122);i.measured_q=*p.lie_down;assert(!run_tick(f,captured+.122,i));for(int j=0;j<12;++j)assert(f.fixed_target()[j]==(*p.lie_down)[j]);
 // Critical inputs/policy faults while HOME is moving remain fail-closed.
 for(int bad=0;bad<3;++bad){R3Supervisor x(p);enter_active(x);x.ControlledAbort(time_at(6));auto i=system_facts(6.002,false);
  if(bad==0)i.arm_control_ready=false;if(bad==1)i.arm_stamp=time_at(5);if(bad==2)i.target_stamp=time_at(5);
  x.Observe(i,time_at(6.002));assert(x.fault_latched()&&x.system_state()==SystemState::EMERGENCY_FAULT&&!x.NeedsPolicy());}
 R3Supervisor stale(p);enter_active(stale);stale.ControlledAbort(time_at(6));stale.Observe(system_facts(6.042,false),time_at(6.042));stale.Tick(time_at(6.042));assert(stale.last_fault()=="policy_result_stale");
 R3Supervisor rejected(p);enter_active(rejected);rejected.ControlledAbort(time_at(6));rejected.ArmHomeRequestAccepted(false,time_at(6));assert(rejected.NeedsPolicy()&&!rejected.fault_latched());
 // A trajectory that does not reach target retains exact last planned target/output.
 R3Supervisor blocked(p);enter_active(blocked);blocked.ControlledAbort(time_at(6));blocked.ArmHomeRequestAccepted(true,time_at(6));
 for(int n=1;n<140;++n){double s=6+n*.002;assert(run_tick(blocked,s,system_facts(s)));}
 assert(blocked.phase()==SystemPhase::LIE_DOWN_BLOCKED&&!blocked.fault_latched()&&blocked.output_enabled());
 auto b=blocked.Tick(time_at(6.278));for(int j=0;j<12;++j)assert(b->motor_cmd[j].q==(*p.lie_down)[j]&&b->motor_cmd[j].kp==40);
 std::cout<<"PASS X: zero RL, idempotent HOME, timeout RL, measured PD handoff, smooth approved pose, confirmed output OFF, critical faults and explicit lie timeout target\n";
}
