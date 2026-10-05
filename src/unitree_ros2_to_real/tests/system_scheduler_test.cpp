#include "system_fsm_fixture.hpp"
#include <iostream>
#include <limits>
int main(){auto p=system_fixture();
 // Virtual 500Hz IO / 50Hz policy. Single pending job, no catch-up queue.
 for(double delay:{.008,.030,.100,.200,1.0}){
  R3Supervisor f(p);enter_active(f);std::optional<PolicyTicket> pending=f.BeginPolicy(time_at(6));
  int due_tick=std::lround(delay/.002),began=1,completed=0;bool timed_out=false;
  for(int n=1;n<=600;++n){double s=6+n*.002;const auto now=SafetyTime{}+std::chrono::seconds(6)+std::chrono::milliseconds(n*2);
   auto input=system_facts(s);input.sport_stamp=input.lowstate_stamp=input.remote_stamp=input.arm_stamp=input.target_stamp=now;f.Observe(input,now);
   if(pending&&n>=due_tick){auto accepted=f.PolicyResult(*pending,p.stand,delay*1000,now);if(accepted)++completed;pending.reset();}
   auto packet=f.Tick(now);assert(packet);
   if(f.fault_latched()){timed_out=true;assert(!f.BeginPolicy(now));break;}
   if(n%10==0){auto ticket=f.BeginPolicy(now);if(ticket){assert(!pending);pending=ticket;due_tick=n+std::lround(delay/.002);++began;assert(!f.BeginPolicy(now));}}
  }
  if(delay<.02)assert(!timed_out&&began>50&&completed>50);
  else if(delay<.04)assert(timed_out&&f.last_fault()=="policy_deadline_burst");
  else assert(timed_out&&f.last_fault()=="policy_inference_timeout");
 }
 // Handoff wins over a valid still-computing zero-policy job.
 p.arm_home_settle_s=0;R3Supervisor handoff(p);enter_active(handoff);handoff.ControlledAbort(time_at(6));handoff.ArmHomeRequestAccepted(true,time_at(6));
 auto i=system_facts(6.002);i.measured_q.fill(.43F);handoff.Observe(i,time_at(6.002));auto work=handoff.BeginPolicy(time_at(6.002));assert(work);
 auto captured=handoff.Tick(time_at(6.002));assert(captured&&handoff.phase()==SystemPhase::PD_CAPTURE&&!handoff.NeedsPolicy());
 assert(captured->motor_cmd[0].q==.43F&&!handoff.PolicyResult(*work,p.stand,8,time_at(6.010)));
 // Critical policy failure while HOME is outstanding remains explicit emergency.
 R3Supervisor failed(p);enter_active(failed);failed.ControlledAbort(time_at(6));failed.Observe(system_facts(6.002,false),time_at(6.002));work=failed.BeginPolicy(time_at(6.002));assert(work);
 assert(failed.PolicyFailed(*work,"stop_policy_exception",time_at(6.003))&&failed.system_state()==SystemState::EMERGENCY_FAULT);
 // Dynamics gate is independent of the approved target, never faked by HOME.
 p.lie_down_validated=false;R3Supervisor gate(p);enter_active(gate);gate.ControlledAbort(time_at(6));gate.ArmHomeRequestAccepted(true,time_at(6));
 assert(run_tick(gate,6.002,system_facts(6.002)));assert(gate.LieDownTargetApproved()&&gate.NeedsPolicy()&&gate.stop_blocker()=="lie_down_dynamics_not_commissioned");
 std::cout<<"PASS virtual 500Hz/50Hz scheduling, 8/30/100/200/1000ms delays, no job queue/catch-up, X in-flight handoff, stop policy failure and dynamics gate\n";
}
