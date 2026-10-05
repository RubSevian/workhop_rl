#include "system_fsm_fixture.hpp"
#include <iostream>

static void inspect_passive(const unitree_go::msg::LowCmd& packet){
 const auto bytes=SerializeLowCmd(packet);assert(packet.crc==Go2Crc(std::span(bytes).first(808)));
 const auto baseline=MakeLowCmd({}, {}, {});
 for(int j=0;j<12;++j){const auto& m=packet.motor_cmd[j];assert(m.mode==0&&m.kp==0&&m.kd==0&&m.tau==0);assert(m.q==baseline.motor_cmd[12].q&&m.dq==baseline.motor_cmd[12].dq);}
 for(int j=12;j<20;++j)assert(packet.motor_cmd[j]==baseline.motor_cmd[j]);
}
static void start_stop(R3Supervisor& f){enter_active(f);assert(f.ControlledAbort(time_at(6)).success);f.ArmHomeRequestAccepted(true,time_at(6));}
int main(){auto p=system_fixture();p.arm_home_settle_s=0;p.lie_down_s=.02;p.lie_down_settle_s=.004;
 R3Supervisor f(p);start_stop(f);std::optional<PolicyTicket> old;
 int emitted=0;double passive_at=0;
 for(int n=1;n<=100;++n){double s=6+n*.002;auto now=time_at(s);auto i=system_facts(s);i.measured_q=*p.lie_down;f.Observe(i,now);
  if(f.NeedsPolicy()){auto w=f.BeginPolicy(now);if(w){old=w;assert(f.PolicyResult(*w,p.stand,2,now));}}
  auto packet=f.Tick(now);
  if(f.phase()==SystemPhase::LIE_DOWN_PASSIVE){
   assert(packet&&f.output_enabled()&&!f.output_stopped()&&!f.NeedsPolicy());
   if(passive_at==0)passive_at=s;
   inspect_passive(*packet);assert(f.AllowsPacket(*packet,now));
   assert(f.passive_packets_sent()==size_t(emitted)); // generated != sent
   auto corrupt=*packet;corrupt.crc^=1;assert(!f.AllowsPacket(corrupt,now));
   f.NotifyPacketPublished(corrupt,now);assert(f.passive_packets_sent()==size_t(emitted));
   f.NotifyPacketPublished(*packet,now);++emitted;
   f.NotifyPacketPublished(*packet,now);assert(f.passive_packets_sent()==size_t(emitted)); // duplicate acknowledgement ignored
  }
  if(f.phase()==SystemPhase::LIE_DOWN_OUTPUT_STOPPING){assert(!packet);assert(s-passive_at<.025);break;}
 }
 assert(emitted==10&&f.passive_sequence_complete()&&f.last_commanded_leg_mode()==0&&!f.output_enabled());
 if(old)assert(!f.PolicyResult(*old,p.stand,1,time_at(6.2)));
 f.EnableOutput(false,time_at(6.2));f.ConfirmOutputStopped();assert(f.system_state()==SystemState::SYSTEM_HOLD);
 for(auto event:{SystemEvent::REQUEST_HOLD,SystemEvent::REQUEST_STAND,SystemEvent::REQUEST_RL,SystemEvent::REQUEST_LIE_DOWN,SystemEvent::ENABLE_OUTPUT}) {
  assert(!f.Dispatch(event,time_at(6.2)).success);
  assert(f.system_state()==SystemState::SYSTEM_HOLD&&!f.output_enabled()&&!f.NeedsPolicy());
 }
 f.ArmHomeRequestAccepted(true,time_at(6.2));assert(f.ControlledAbort(time_at(6.2)).success&&!f.BeginPolicy(time_at(6.2)));
 auto i=system_facts(7);i.own_output_healthy=false;i.measured_q.fill(-.18F);f.Observe(i,time_at(7));
 assert(f.StartRemoteSequence(time_at(7)).success);f.Observe(i,time_at(7));assert(f.RemoteSequenceNext(time_at(7))==R3SequenceAction::ENABLE_OUTPUT);
 assert(f.EnableOutput(true,time_at(7)).success);auto active=f.Tick(time_at(7));assert(active);
 for(int j=0;j<12;++j)assert(active->motor_cmd[j].mode==1&&active->motor_cmd[j].q==-.18F&&active->motor_cmd[j].kp==40&&active->motor_cmd[j].kd==1);
 f.NotifyPacketPublished(*active,time_at(7));assert(f.last_commanded_leg_mode()==1&&f.passive_packets_sent()==0&&!f.passive_sequence_complete());
 // HOME timeout, unachieved pose, dropout and critical fault never claim a passive success.
 for(int failure=0;failure<4;++failure){R3Supervisor x(p);enter_active(x);x.ControlledAbort(time_at(6));if(failure!=0)x.ArmHomeRequestAccepted(true,time_at(6));
  for(int n=1;n<150;++n){double s=6+n*.002;auto fact=system_facts(s,failure!=0);if(failure==2&&n<12)fact.measured_q=*p.lie_down;
   if(failure==3)fact.arm_control_ready=false;auto packet=run_tick(x,s,fact);
   if(packet)assert(packet->motor_cmd[0].mode==1);assert(x.passive_packets_sent()==0&&!x.passive_sequence_complete());
   if(x.fault_latched())break;
  }
 }
 // Publication is required: no acknowledgements -> bounded timeout, unchanged B damping.
 R3Supervisor stalled(p);start_stop(stalled);double entered=0;unitree_go::msg::LowCmd passive;
 for(int n=1;n<50;++n){double s=6+n*.002;auto fact=system_facts(s);fact.measured_q=*p.lie_down;stalled.Observe(fact,time_at(s));
  if(stalled.NeedsPolicy()){auto w=stalled.BeginPolicy(time_at(s));if(w)assert(stalled.PolicyResult(*w,p.stand,2,time_at(s)));}
  auto packet=stalled.Tick(time_at(s));if(stalled.phase()==SystemPhase::LIE_DOWN_PASSIVE){assert(packet);passive=*packet;entered=s;break;}}
 assert(entered>0&&stalled.passive_packets_sent()==0);
 auto t=entered+.102;auto fact=system_facts(t);fact.measured_q=*p.lie_down;stalled.Observe(fact,time_at(t));auto emergency=stalled.Tick(time_at(t));
 assert(stalled.last_fault()=="passive_transition_timeout"&&!stalled.passive_sequence_complete()&&stalled.passive_packets_sent()==0);
 assert(emergency);for(int j=0;j<12;++j)assert(emergency->motor_cmd[j].mode==1&&emergency->motor_cmd[j].kp==0&&emergency->motor_cmd[j].kd==3&&emergency->motor_cmd[j].dq==0&&emergency->motor_cmd[j].tau==0);
 assert(!stalled.AllowsPacket(passive,time_at(t)));stalled.NotifyPacketPublished(passive,time_at(t));assert(stalled.passive_packets_sent()==0);
 // A late publish return is recorded truthfully but cannot claim successful shutdown.
 R3Supervisor delayed(p);start_stop(delayed);
 for(int n=1;n<50;++n){double s=6+n*.002;auto fact=system_facts(s);fact.measured_q=*p.lie_down;delayed.Observe(fact,time_at(s));
  if(delayed.NeedsPolicy()){auto w=delayed.BeginPolicy(time_at(s));if(w)assert(delayed.PolicyResult(*w,p.stand,2,time_at(s)));}
  auto packet=delayed.Tick(time_at(s));if(delayed.phase()==SystemPhase::LIE_DOWN_PASSIVE){
   assert(packet&&delayed.AllowsPacket(*packet,time_at(s)));
   delayed.NotifyPacketPublished(*packet,time_at(s+.102));
   assert(delayed.passive_packets_sent()==1&&delayed.last_commanded_leg_mode()==0);
   assert(delayed.last_fault()=="passive_transition_timeout"&&!delayed.passive_sequence_complete());break;
  }
 }
 // B preempts an already publishing passive sequence, preserving active damping mode.
 R3Supervisor interrupted(p);start_stop(interrupted);
 for(int n=1;n<40;++n){double s=6+n*.002;auto fact=system_facts(s);fact.measured_q=*p.lie_down;
  auto packet=run_tick(interrupted,s,fact);
  if(interrupted.phase()==SystemPhase::LIE_DOWN_PASSIVE){
   assert(packet&&interrupted.passive_packets_sent()==1);
   std::array requests{SystemEvent::REQUEST_A,SystemEvent::REQUEST_X,SystemEvent::REQUEST_B};interrupted.DispatchEvents(requests,time_at(s));
   auto damp=interrupted.Tick(time_at(s));assert(damp&&!interrupted.passive_sequence_complete());
   for(int j=0;j<12;++j)assert(damp->motor_cmd[j].mode==1&&damp->motor_cmd[j].kp==0&&damp->motor_cmd[j].kd==3);
   assert(!interrupted.AllowsPacket(*packet,time_at(s)));break;
  }
 }
 std::cout<<"PASS B2 canonical 12-motor PASSIVE/sentinels/CRC, exactly10 acknowledged packets then OFF, mode1 measured restart, no false HOME/lie/dropout/fault success, bounded unacknowledged timeout and unchanged B damping\n";
}
