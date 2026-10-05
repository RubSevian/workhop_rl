#include "system_fsm_fixture.hpp"
#include "output_lease.hpp"
#include <filesystem>
#include <iostream>
#include <memory>
#include <unistd.h>

int main(){auto p=system_fixture();p.lie_down_validated=false;p.controlled_stop_lie_down_trial=true;
 // Trial grants only controlled X, never claims physical validation.
 R3Supervisor f(p);assert(!f.profile().lie_down_validated&&f.profile().controlled_stop_lie_down_trial);
 enter_active(f);f.ControlledAbort(time_at(6));f.ArmHomeRequestAccepted(true,time_at(6));
 auto low=system_facts(6.002);low.measured_q.fill(.43F);f.Observe(low,time_at(6.002));
 auto late=f.BeginPolicy(time_at(6.002));assert(late);const auto generation=f.orchestration_generation();
 const std::string path="/tmp/go2-b1-output-"+std::to_string(getpid());
 auto lease=std::make_unique<OutputLease>(path);assert(lease->acquired());
 auto publisher=std::make_unique<int>(1); // No DDS publisher or physical transport in this test.
 int packets=0;bool reached=false;
 for(int n=1;n<160;++n){double s=6+n*.002;auto i=system_facts(s);i.measured_q=*p.lie_down;
  f.Observe(i,time_at(s));
  if(f.NeedsPolicy()){auto work=f.BeginPolicy(time_at(s));if(work)assert(f.PolicyResult(*work,p.stand,2,time_at(s)));}
  auto packet=f.Tick(time_at(s));
  if(packet){assert(f.AllowsPacket(*packet,time_at(s)));f.NotifyPacketPublished(*packet,time_at(s));}
  if(packet){++packets;assert(f.output_enabled());}
  if(f.phase()==SystemPhase::LIE_DOWN_VERIFY){reached=true;assert(f.output_enabled()&&lease&&publisher);}
  if(f.phase()==SystemPhase::LIE_DOWN_OUTPUT_STOPPING)break;
 }
 assert(reached&&packets>0&&f.system_state()==SystemState::CONTROLLED_STOP);
 assert(!f.output_enabled()&&!f.output_stopped()&&!f.NeedsPolicy());
 assert(!f.PolicyResult(*late,p.stand,2,time_at(6.2))&&f.orchestration_generation()>generation);
 f.ArmHomeRequestAccepted(true,time_at(6.2));assert(!f.NeedsPolicy());
 // StopPublisher lifecycle: stop -> destroy publisher -> unlock lease -> confirm.
 f.EnableOutput(false,time_at(6.2));assert(f.system_state()==SystemState::CONTROLLED_STOP);
 publisher.reset();lease.reset();f.ConfirmOutputStopped();
 assert(f.system_state()==SystemState::SYSTEM_HOLD&&f.output_stopped());
 assert(!f.Tick(time_at(6.2))&&!f.BeginPolicy(time_at(6.2)));
 assert(f.ControlledAbort(time_at(6.2)).success);f.ConfirmOutputStopped();
 assert(f.system_state()==SystemState::SYSTEM_HOLD&&!f.output_enabled());
 {OutputLease foreign(path);assert(foreign.acquired());OutputLease ours(path);assert(!ours.acquired());}
 // Existing lease mechanism reacquires after clean unlock.
 lease=std::make_unique<OutputLease>(path);assert(lease->acquired());
 auto now=system_facts(7);now.own_output_healthy=false;now.measured_q.fill(-.17F);
 f.Observe(now,time_at(7));assert(f.StartRemoteSequence(time_at(7)).success);
 f.Observe(now,time_at(7));assert(f.RemoteSequenceNext(time_at(7))==R3SequenceAction::ENABLE_OUTPUT);
 assert(f.EnableOutput(true,time_at(7)).success);publisher=std::make_unique<int>(1);
 auto first=f.Tick(time_at(7));assert(first);for(int j=0;j<12;++j)assert(first->motor_cmd[j].mode==1&&first->motor_cmd[j].q==-.17F&&first->motor_cmd[j].kp==40&&first->motor_cmd[j].kd==1);
 f.EnableOutput(false,time_at(7));publisher.reset();lease.reset();f.ConfirmOutputStopped();std::filesystem::remove(path);
 // Trial never authorizes another target.
 auto wrong=p;(*wrong.lie_down)[0]=.2F;R3Supervisor bad(wrong);enter_active(bad);bad.ControlledAbort(time_at(6));bad.ArmHomeRequestAccepted(true,time_at(6));
 assert(run_tick(bad,6.002,system_facts(6.002)));assert(bad.NeedsPolicy()&&bad.stop_blocker()=="lie_down_target_not_approved");
 // Reached is continuous through settle: losing the target cannot turn output off.
 auto settling=p;settling.arm_home_settle_s=0;settling.lie_down_s=.02;settling.lie_down_settle_s=.04;settling.lie_down_timeout_s=.10;
 R3Supervisor lost(settling);enter_active(lost);lost.ControlledAbort(time_at(6));lost.ArmHomeRequestAccepted(true,time_at(6));
 for(int n=1;n<=80;++n){double s=6+n*.002;auto i=system_facts(s);if(n<25)i.measured_q=*settling.lie_down;assert(run_tick(lost,s,i));}
 assert(lost.phase()==SystemPhase::LIE_DOWN_BLOCKED&&lost.output_enabled()&&!lost.fault_latched());
 // Successful OFF hold + B cannot recreate a publisher or resume work.
 R3Supervisor stopped(p);enter_active(stopped);stopped.ControlledAbort(time_at(6));stopped.ArmHomeRequestAccepted(true,time_at(6));
 for(int n=1;n<120;++n){auto i=system_facts(6+n*.002);i.measured_q=*p.lie_down;run_tick(stopped,6+n*.002,i);}
 stopped.EnableOutput(false,time_at(6.24));stopped.ConfirmOutputStopped();assert(stopped.system_state()==SystemState::SYSTEM_HOLD);
 std::array events{SystemEvent::REQUEST_A,SystemEvent::REQUEST_X,SystemEvent::REQUEST_B};stopped.DispatchEvents(events,time_at(6.24));
 stopped.ConfirmOutputStopped();assert(stopped.system_state()==SystemState::EMERGENCY_FAULT&&!stopped.output_enabled()&&!stopped.Tick(time_at(6.24)));
 assert(!stopped.StartRemoteSequence(time_at(6.24)).success);
 std::cout<<"PASS B1 explicit trial/approved target, settle/dropout/timeout, stop acknowledgement, real file lease unlock/reacquire, measured first packet, late result/HOME rejection and B after output OFF\n";
}
