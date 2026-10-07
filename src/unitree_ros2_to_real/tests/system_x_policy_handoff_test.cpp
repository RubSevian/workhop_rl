#include "system_fsm_fixture.hpp"
#include <iostream>

int main() {
 // X at several offsets relative to the last accepted result and a running
 // worker. Policy jobs start only on 20ms ticks; IO runs every 2ms.
 for(double x:{6.002,6.020,6.038}) {
  auto p=system_fixture();R3Supervisor f(p);enter_active(f);
  f.Observe(system_facts(x-.001,false),time_at(x-.001));
  auto old=f.BeginPolicy(time_at(x-.001));assert(old);
  auto before=f.Tick(time_at(x-.001));assert(before);
  const auto stamp_age=f.PolicyAgeSeconds(time_at(x));
  assert(f.ControlledAbort(time_at(x)).success&&f.zero_policy_handoff_pending());
  assert(f.PolicyAgeSeconds(time_at(x))==stamp_age); // No fake freshness.
  assert(!f.PolicyResult(*old,p.stand,5,time_at(x+.004))&&!f.fault_latched());
  assert(f.ControlledAbort(time_at(x+.004)).success); // Idempotent, no renewal.
  std::optional<PolicyTicket> fresh;
  for(int n=1;n<=14;++n) {
   double s=x+n*.002;f.Observe(system_facts(s,false),time_at(s));
   if(n==10){fresh=f.BeginPolicy(time_at(s));assert(fresh);}
   auto held=f.Tick(time_at(s));assert(held&&!f.fault_latched());
   assert(SerializeLowCmd(*held)==SerializeLowCmd(*before));
   assert(f.AllowsPacket(*held,time_at(s)));
  }
  f.Observe(system_facts(x+.029,false),time_at(x+.029));
  assert(f.PolicyResult(*fresh,p.stand,9,time_at(x+.029)));
  assert(!f.zero_policy_handoff_pending());
  auto zero=f.Tick(time_at(x+.030));assert(zero&&!f.fault_latched());
  assert(zero->motor_cmd[0].q==p.stand[0]&&zero->motor_cmd[0].kp==25);
  // Ordinary watchdog resumes immediately after the accepted zero result.
  f.Observe(system_facts(x+.070,false),time_at(x+.070));f.Tick(time_at(x+.070));
  assert(f.last_fault()=="policy_result_stale");
 }
 // Missing zero response is bounded from X even if X is pressed again.
 for(bool start_worker:{false,true}) {
  auto p=system_fixture();R3Supervisor f(p);enter_active(f);
  assert(f.ControlledAbort(time_at(6)).success);
  if(start_worker){f.Observe(system_facts(6.020,false),time_at(6.020));assert(f.BeginPolicy(time_at(6.020)));}
  assert(f.ControlledAbort(time_at(6.038)).success);
  f.Observe(system_facts(6.042,false),time_at(6.042));auto damp=f.Tick(time_at(6.042));
  assert(f.last_fault()=="policy_zero_handoff_timeout"&&damp);
  assert(damp->motor_cmd[0].kp==0&&damp->motor_cmd[0].kd==3);
 }
 // A late completion cannot clear the handoff deadline even if IO was delayed.
 {
  auto p=system_fixture();R3Supervisor f(p);enter_active(f);f.ControlledAbort(time_at(6));
  f.Observe(system_facts(6.020,false),time_at(6.020));auto job=f.BeginPolicy(time_at(6.020));assert(job);
  f.Observe(system_facts(6.041,false),time_at(6.041));
  assert(!f.PolicyResult(*job,p.stand,21,time_at(6.041))&&f.last_fault()=="policy_zero_handoff_timeout");
 }
 {
  auto p=system_fixture();R3Supervisor f(p);enter_active(f);
  f.Observe(system_facts(6.041,false),time_at(6.041));
  assert(!f.ControlledAbort(time_at(6.041)).success&&f.last_fault()=="policy_result_stale");
 }
 // B and bad inputs still win during handoff; stale jobs cannot buy grace.
 for(int event=0;event<3;++event) {
  auto p=system_fixture();R3Supervisor f(p);enter_active(f);
  auto old=f.BeginPolicy(time_at(6));assert(old);
  if(event==2){f.Observe(system_facts(6.041,false),time_at(6.041));assert(!f.ControlledAbort(time_at(6.041)).success);assert(f.last_fault()=="policy_inference_timeout");}
  else {
   assert(f.ControlledAbort(time_at(6.001)).success);
   if(event==0)assert(f.Emergency(time_at(6.002)).success);
   else {auto bad=system_facts(6.002,false);bad.arm_control_ready=false;f.Observe(bad,time_at(6.002));}
   assert(f.fault_latched()&&!f.zero_policy_handoff_pending());
  }
  assert(!f.PolicyResult(*old,p.stand,5,time_at(6.045)));
 }
 std::cout<<"PASS X during inference: IO500/policy50, frozen packet, truthful ages, fresh zero acceptance, bounded40ms handoff, repeated X/B/stale worker/input gates\n";
}
