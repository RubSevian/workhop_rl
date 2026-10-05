#include "r3_commissioning.hpp"
#include "output_lease.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <filesystem>
#include <unistd.h>
#include <cstring>
using namespace sim2real;
SafetyTime t(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
// Synthetic gains/poses below are fixtures, NEVER deployment calibration.
R3Profile profile(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);
 p.stand.fill(.2F);std::array<float,12> lie;lie.fill(-.3F);p.lie_down=lie;
 std::array<float,12> damping;damping.fill(1.5F);p.emergency_kd=damping;
 p.gate0_verified=p.mapping_verified=p.emergency_validated=p.remote_chords_verified=p.robot_supported=p.lie_down_validated=p.policy_timing_reviewed=true;
 p.emergency_evidence="OFFLINE_MOCK_ONLY";return p;}
R3Inputs input(double at,SportMode sport=SportMode::RELEASED){R3Inputs in;
 in.ready={true,true,true,true,true,true,true,true,false,true};in.measured_q.fill(.4F);in.arm_static_hold=true;in.arm_home_ready=true;
 in.sport=sport;in.sport_stamp=in.lowstate_stamp=in.remote_stamp=in.arm_stamp=in.target_stamp=t(at);return in;}
void arm(R3Supervisor& s,double at=0){s.Observe(input(at,SportMode::ACTIVE),t(at));assert(s.state()==R3State::STOCK);assert(s.Takeover(t(at)).success);
 assert(!s.output_enabled());s.Observe(input(at),t(at));assert(s.state()==R3State::SPORT_RELEASE_VERIFIED);assert(s.EnableOutput(true,t(at)).success);}
void holding(R3Supervisor& s){arm(s);auto first=s.Tick(t(0));assert(first);assert(s.AllowsPacket(*first,t(0)));
 for(int i=0;i<12;++i)assert(first->motor_cmd[i].q==.4F);assert(s.state()==R3State::HOLD_CURRENT);
 assert(s.RequestStand(t(0)).success);auto start=s.Tick(t(0));assert(start->motor_cmd[0].q==.4F);
 s.Observe(input(2),t(2));auto quarter=s.Tick(t(2));assert(std::abs(quarter->motor_cmd[0].q-.35F)<1e-6);
 s.Observe(input(4),t(4));auto middle=s.Tick(t(4));assert(std::abs(middle->motor_cmd[0].q-.3F)<1e-6);
 auto standing=input(8);standing.measured_q=profile().stand;s.Observe(standing,t(8));auto end=s.Tick(t(8));assert(end->motor_cmd[0].q==.2F);assert(s.state()==R3State::HOLDING);}
bool policy(R3Supervisor& s,const std::array<float,12>& q,double ms,SafetyTime now){
 const auto ticket=s.BeginPolicy(now);return ticket&&s.PolicyResult(*ticket,q,ms,now);
}
void state_tests(){
 R3Supervisor s(profile());assert(s.state()==R3State::DISARMED);assert(!s.Tick(t(0)));assert(!s.EnableOutput(true,t(0)).success);
 holding(s);assert(s.RequestRl(t(8)).success);assert(s.state()==R3State::RL_ZERO);assert(!s.navigation_active());assert((s.command()==std::array<double,3>{}));
 assert(s.ConsumePolicyReset());assert(!s.ConsumePolicyReset());std::array<float,12> action;action.fill(.21F);
 policy(s,action,2,t(8));auto rl=s.Tick(t(8));assert(rl&&rl->motor_cmd[0].kp==25&&s.AllowsPacket(*rl,t(8)));
 assert(!s.ManualCommand({.1,.1,0},1,t(8)).success);assert(!s.ManualCommand({.21,0,0},1,t(8)).success);
 assert(!s.ManualCommand({0,.11,0},1,t(8)).success);assert(!s.ManualCommand({0,0,.11},1,t(8)).success);
 assert(!s.ManualCommand({.1,0,0},2,t(8)).success);assert(!s.ManualCommand({std::numeric_limits<double>::infinity(),0,0},1,t(8)).success);
 assert(s.ManualCommand({.1,0,0},1,t(8)).success);assert(s.state()==R3State::RL_ACTIVE);
 s.Observe(input(8.2),t(8.2));s.RenewDeadman(t(8.2));policy(s,action,2,t(8.2));assert(s.Tick(t(8.2)));assert(s.state()==R3State::RL_ACTIVE);
 s.Observe(input(8.451),t(8.451));assert(s.Tick(t(8.451)));assert(s.state()==R3State::CONTROLLED_ABORT);assert((s.command()==std::array<double,3>{}));
 s.Observe(input(9.451),t(9.451));s.Tick(t(9.451));assert(s.state()==R3State::HOLDING);
 assert(s.ControlledAbort(t(9.451)).success);assert(s.ConsumeArmHoldRequest());assert(!s.navigation_active());
 // X keeps the captured pose even with a validated lie-down profile.
 s.Observe(input(10.451),t(10.451));auto held=s.Tick(t(10.451));assert(s.state()==R3State::HOLDING);
 assert(held&&held->motor_cmd[0].q==.4F&&held->motor_cmd[0].kp==40&&held->motor_cmd[0].kd==1);
 auto later=input(18.451);later.measured_q.fill(.45F);s.Observe(later,t(18.451));held=s.Tick(t(18.451));
 assert(s.state()==R3State::HOLDING&&held&&held->motor_cmd[0].q==.4F);
 assert(s.RequestLieDown(t(18.451)).success);assert(s.state()==R3State::LIE_DOWN_TRANSITION);
 assert(!s.RequestReturnToStock(t(18.451)).success);
 s.Observe(input(26.450),t(26.450));assert(s.Tick(t(26.450)));assert(s.output_enabled());
 auto lying=input(26.451);lying.measured_q=*profile().lie_down;s.Observe(lying,t(26.451));assert(!s.Tick(t(26.451)));
 assert(s.state()==R3State::OUTPUT_STOPPING);assert(!s.RequestReturnToStock(t(26.451)).success);
 s.ConfirmOutputStopped();assert(s.RequestReturnToStock(t(26.451)).success);assert(s.state()==R3State::RETURN_TO_STOCK);
 lying.sport=SportMode::ACTIVE;s.Observe(lying,t(26.451));s.ConfirmStockObserved(t(26.451));assert(s.state()==R3State::STOCK);
}
void gates(){
 R3Supervisor no_tracking(profile());arm(no_tracking);no_tracking.Tick(t(0));no_tracking.RequestStand(t(0));
 no_tracking.Observe(input(9.01),t(9.01));auto tracked_failure=no_tracking.Tick(t(9.01));
 assert(!no_tracking.fault_latched()&&no_tracking.state()==R3State::HOLDING&&tracked_failure);

 auto no_lie=profile();no_lie.lie_down.reset();no_lie.lie_down_validated=false;
 R3Supervisor hold_exit(no_lie);arm(hold_exit);hold_exit.Tick(t(0));assert(hold_exit.ControlledAbort(t(0)).success);
 assert((hold_exit.command()==std::array<double,3>{}));assert(!hold_exit.RequestReturnToStock(t(0)).success);
 // Fresh measured motion after capture does not impose a 0.01 rad entry gate.
 R3Supervisor changed(profile());arm(changed);auto shifted=input(.002);shifted.measured_q.fill(.43F);changed.Observe(shifted,t(.002));
 auto first_hold=changed.Tick(t(.002));
 assert(!changed.fault_latched()&&first_hold&&changed.state()==R3State::HOLD_CURRENT);
 for(int i=0;i<12;++i)assert(first_hold->motor_cmd[i].q==.4F&&first_hold->motor_cmd[i].kp==40&&first_hold->motor_cmd[i].kd==1);
 assert(changed.RequestStand(t(.002)).success);

 for(auto sport:{SportMode::ACTIVE,SportMode::UNKNOWN,SportMode::ERROR}) {
  R3Supervisor s(profile());auto in=input(0,sport);s.Observe(in,t(0));s.Takeover(t(0));s.Observe(in,t(0));assert(!s.EnableOutput(true,t(0)).success);
 }
 using Member=bool SafetyReadiness::*;
 for(Member m:{&SafetyReadiness::model_loaded,&SafetyReadiness::config_valid,&SafetyReadiness::lowstate_fresh,&SafetyReadiness::motor_state_valid,
  &SafetyReadiness::remote_fresh,&SafetyReadiness::arm_feedback_ready,&SafetyReadiness::arm_target_ready,&SafetyReadiness::transport_ready,&SafetyReadiness::no_fault}) {
  R3Supervisor s(profile());auto in=input(0);s.Observe(in,t(0));s.Takeover(t(0));s.Observe(in,t(0));in.ready.*m=false;s.Observe(in,t(0));assert(!s.EnableOutput(true,t(0)).success);
 }
 for(int missing=0;missing<7;++missing){auto p=profile();
  switch(missing){case 0:p.gate0_verified=false;break;case 1:p.mapping_verified=false;break;case 2:p.robot_supported=false;break;
   case 3:p.remote_chords_verified=false;break;case 4:p.emergency_validated=false;break;case 5:p.emergency_kd.reset();break;case 6:p.emergency_evidence.clear();break;}
  R3Supervisor s(p);s.Observe(input(0),t(0));s.Takeover(t(0));s.Observe(input(0),t(0));assert(!s.EnableOutput(true,t(0)).success);
 }
 R3Supervisor stale(profile());arm(stale);assert(!stale.Tick(t(.501)));assert(stale.fault_latched());assert(!stale.output_enabled());
 R3Supervisor emergency(profile());arm(emergency);emergency.Tick(t(0));assert(emergency.Emergency(t(0)).success);auto damp=emergency.Tick(t(0));
 assert(damp&&damp->motor_cmd[0].kp==0&&damp->motor_cmd[0].kd==1.5F&&damp->motor_cmd[0].tau==0&&damp->motor_cmd[0].dq==0);
 // Exercise the selected Go2 candidate in a mock-only eligible profile.
 auto candidate=profile();candidate.emergency_kd->fill(3);
 R3Supervisor candidate_damp(candidate);arm(candidate_damp);candidate_damp.Tick(t(0));
 assert(candidate_damp.Emergency(t(0)).success);auto candidate_packet=candidate_damp.Tick(t(0));
 assert(candidate_packet&&candidate_damp.AllowsPacket(*candidate_packet,t(0)));
 for(int i=0;i<12;++i){const auto& m=candidate_packet->motor_cmd[i];
  assert(m.kp==0&&m.kd==3&&m.dq==0&&m.tau==0);
 }
 candidate.emergency_validated=false;R3Supervisor unvalidated_candidate(candidate);
 unvalidated_candidate.Observe(input(0),t(0));assert(unvalidated_candidate.Takeover(t(0)).success);
 assert(!unvalidated_candidate.EnableOutput(true,t(0)).success&&!unvalidated_candidate.output_enabled());
 assert(emergency.AllowsPacket(*damp,t(0)));assert(!emergency.RequestStand(t(0)).success);assert(!emergency.RequestRl(t(0)).success);
 auto bad=*damp;bad.motor_cmd[0].kp=1;assert(!emergency.AllowsPacket(bad,t(0)));
 emergency.EnableOutput(false,t(0));emergency.ConfirmOutputStopped();assert(!emergency.EnableOutput(true,t(0)).success);
 R3Supervisor armfail(profile());arm(armfail);auto in=input(0);in.ready.arm_target_ready=false;armfail.Observe(in,t(0));assert(armfail.fault_latched());assert(armfail.state()==R3State::EMERGENCY_DAMP);
 R3Supervisor nan(profile());arm(nan);in=input(0);in.measured_q[0]=std::numeric_limits<float>::quiet_NaN();nan.Observe(in,t(0));assert(nan.fault_latched());
 R3Supervisor deadline(profile());holding(deadline);deadline.RequestRl(t(8));deadline.ConsumePolicyReset();std::array<float,12> q{};
 for(int i=0;i<3;++i)policy(deadline,q,20,t(8));assert(deadline.deadline_misses()==3&&deadline.fault_latched());
 R3Supervisor reset(profile());holding(reset);reset.RequestRl(t(8));policy(reset,q,2,t(8));assert(reset.fault_latched());
}
std::array<uint8_t,40> remote(uint16_t mask){std::array<uint8_t,40> raw{};raw[2]=mask&255;raw[3]=mask>>8;return raw;}
void remote_tests(){
 R3RemoteCommands r({"L1","L2","X"},{"L1","L2","B"});
 R3RemoteCommands lone_x({"L1","L2","X"},{"L1","L2","B"});
 for(double at:{0.,.2,.4,.6,.8}){lone_x.Receive(remote(0x400),t(at));assert(lone_x.Poll(t(at))==R3RemoteEvent::NONE);}
 assert(r.Poll(t(0))==R3RemoteEvent::NONE);
 // All chords simultaneously: emergency outranks abort and takeover.
 auto all=remote(0x722);for(double at:{0.,.2,.4,.6}){r.Receive(all,t(at));assert(r.Poll(t(at))==R3RemoteEvent::NONE);}
 r.Receive(all,t(.75));assert(r.Poll(t(.75))==R3RemoteEvent::EMERGENCY);assert(r.Poll(t(.75))==R3RemoteEvent::NONE);
 auto release=remote(0);r.Receive(release,t(.8));r.Poll(t(.8));
 auto abort=remote(0x422);for(double at:{1.,1.2,1.4,1.6}){r.Receive(abort,t(at));assert(r.Poll(t(at))==R3RemoteEvent::NONE);}
 r.Receive(abort,t(1.75));assert(r.Poll(t(1.75))==R3RemoteEvent::CONTROLLED_ABORT);
 r.Receive(release,t(2));r.Poll(t(2));auto takeover=remote(0x122);
 for(double at:{2.1,2.3,2.5,2.7}){r.Receive(takeover,t(at));assert(r.Poll(t(at))==R3RemoteEvent::NONE);}
 r.Receive(takeover,t(2.85));assert(r.Poll(t(2.85))==R3RemoteEvent::TAKEOVER);
 R3RemoteCommands stale({"L1","L2","X"},{"L1","L2","B"});stale.Receive(all,t(0));assert(stale.Poll(t(1))==R3RemoteEvent::NONE);
 bool threw=false;try{R3RemoteCommands same({"L1","L2","B"},{"L1","L2","B"});}catch(...){threw=true;}assert(threw);
 threw=false;try{R3RemoteCommands takeover_same({"L1","L2","A"},{"L1","L2","B"});}catch(...){threw=true;}assert(threw);
}
void lease(){auto path=std::filesystem::temp_directory_path()/("r3-output-"+std::to_string(getpid()));
 {OutputLease publishing(path.string());assert(publishing.acquired());OutputLease enable_stock(path.string());assert(!enable_stock.acquired());}
 {OutputLease enable_stock(path.string());assert(enable_stock.acquired());OutputLease publishing(path.string());assert(!publishing.acquired());}
 std::filesystem::remove(path);
}
void home_readiness_gate_tests(){
 R3Supervisor blocked(profile());auto in=input(0,SportMode::ACTIVE);in.arm_home_ready=false;
 blocked.Observe(in,t(0));assert(!blocked.StartRemoteSequence(t(0)).success);
 assert(blocked.RemoteSequenceNext(t(0))==R3SequenceAction::NONE&&!blocked.output_enabled());
 R3Supervisor ready(profile());in.arm_home_ready=true;ready.Observe(in,t(0));
 assert(ready.StartRemoteSequence(t(0)).success);ready.Observe(in,t(0));
 assert(ready.RemoteSequenceNext(t(0))==R3SequenceAction::RELEASE_SPORT);
 // Readiness disappears after a chord, before RPC: do not release.
 R3Supervisor vanished(profile());vanished.Observe(in,t(0));assert(vanished.StartRemoteSequence(t(0)).success);
 in.arm_home_ready=false;vanished.Observe(in,t(0));assert(vanished.RemoteSequenceNext(t(0))==R3SequenceAction::NONE);
}
void automatic_sequence_tests(){
 // X now holds the measured pose; stand/RL do not depend on automatic lie-down.
 auto no_lie=profile();no_lie.lie_down.reset();no_lie.lie_down_validated=false;
 R3Supervisor without_lie(no_lie);without_lie.Observe(input(0,SportMode::ACTIVE),t(0));
 assert(without_lie.StartRemoteSequence(t(0)).success);
 R3Supervisor stand_without_lie(no_lie);arm(stand_without_lie);stand_without_lie.Tick(t(0));
 assert(stand_without_lie.RequestStand(t(0)).success);
 auto reached=input(8);reached.measured_q=no_lie.stand;
 stand_without_lie.Observe(reached,t(8));stand_without_lie.Tick(t(8));
 assert(stand_without_lie.state()==R3State::HOLDING&&!stand_without_lie.RequestLieDown(t(8)).success);
 auto p=profile();R3Supervisor s(p);s.Observe(input(0,SportMode::ACTIVE),t(0));
 assert(s.StartRemoteSequence(t(0)).success);assert(!s.output_enabled());
 s.Observe(input(0,SportMode::ACTIVE),t(0));assert(s.RemoteSequenceNext(t(0))==R3SequenceAction::RELEASE_SPORT);
 assert(s.RemoteSequenceNext(t(0))==R3SequenceAction::NONE); // never repeat release
 s.Observe(input(.1),t(.1));assert(s.RemoteSequenceNext(t(.1))==R3SequenceAction::ENABLE_OUTPUT);
 assert(s.EnableOutput(true,t(.1)).success);auto first=s.Tick(t(.1));assert(first&&first->motor_cmd[0].q==.4F);
 assert(s.RemoteSequenceNext(t(.1))==R3SequenceAction::NONE);
 s.Observe(input(1.11),t(1.11));assert(s.RemoteSequenceNext(t(1.11))==R3SequenceAction::STAND);
 assert(s.RequestStand(t(1.11)).success);s.Tick(t(1.11));
 auto stand=input(9.12);stand.measured_q=p.stand;s.Observe(stand,t(9.12));s.Tick(t(9.12));
 assert(s.state()==R3State::HOLDING);assert(s.RemoteSequenceNext(t(9.12))==R3SequenceAction::NONE);
 auto hold=s.Tick(t(9.12));assert(hold&&hold->motor_cmd[0].kp==p.kp[0]); // PD hold, not emergency
 stand=input(10.13);stand.measured_q=p.stand;s.Observe(stand,t(10.13));
 assert(s.RemoteSequenceNext(t(10.13))==R3SequenceAction::RL);assert(s.RequestRl(t(10.13)).success);
 assert(s.ConsumePolicyReset());assert((s.command()==std::array<double,3>{}));
 assert(s.RemoteSequenceNext(t(10.13))==R3SequenceAction::NONE&&!s.remote_sequence_active());
 assert(s.ControlledAbort(t(10.13)).success);
 assert(s.StartRemoteSequence(t(10.13)).success&&s.state()==R3State::HOLD_CURRENT);
 assert(s.RemoteSequenceNext(t(10.13))==R3SequenceAction::NONE);
 s.Observe(input(10.16),t(10.16));assert(s.RemoteSequenceNext(t(10.16))==R3SequenceAction::STAND);
 auto unreviewed=p;unreviewed.policy_timing_reviewed=false;R3Supervisor blocked(unreviewed);
 blocked.Observe(input(0,SportMode::ACTIVE),t(0));assert(!blocked.StartRemoteSequence(t(0)).success);
 assert(!blocked.remote_sequence_active()&&!blocked.output_enabled());
 R3Supervisor unknown(p);unknown.Observe(input(0,SportMode::UNKNOWN),t(0));assert(!unknown.StartRemoteSequence(t(0)).success);
 R3Supervisor cancel(p);cancel.Observe(input(0,SportMode::ACTIVE),t(0));assert(cancel.StartRemoteSequence(t(0)).success);
 cancel.Observe(input(0,SportMode::ACTIVE),t(0));cancel.RemoteSequenceNext(t(0));
 cancel.ControlledAbort(t(0));cancel.Observe(input(.1),t(.1));assert(cancel.RemoteSequenceNext(t(.1))==R3SequenceAction::NONE);
 assert(!cancel.output_enabled());
 R3Supervisor stale(p);stale.Observe(input(0,SportMode::ACTIVE),t(0));stale.StartRemoteSequence(t(0));
 stale.RemoteSequenceNext(t(.6));assert(stale.fault_latched()&&!stale.remote_sequence_active()&&!stale.output_enabled());
 R3Supervisor emergency(p);emergency.Observe(input(0,SportMode::ACTIVE),t(0));emergency.StartRemoteSequence(t(0));
 emergency.Emergency(t(0));assert(!emergency.remote_sequence_active());
}
void discovery_wait_tests(){
 LowCmdDiscoveryWait wait(20,.5);
 R3Supervisor s(profile());s.Observe(input(0,SportMode::ACTIVE),t(0));assert(s.StartRemoteSequence(t(0)).success);
 s.Observe(input(0,SportMode::ACTIVE),t(0));assert(s.RemoteSequenceNext(t(0))==R3SequenceAction::RELEASE_SPORT);
 // Sport endpoint can remain in discovery for about ten seconds after release.
 // The automatic sequence waits without repeating release or enabling output.
 for(int i=1;i<=50;++i){const double at=i*.2;s.Observe(input(at),t(at));
  assert(s.RemoteSequenceNext(t(at))==R3SequenceAction::ENABLE_OUTPUT);
  assert(wait.Update(t(at),true,1)==LowCmdWaitResult::WAITING);
  assert(!s.output_enabled()&&!s.Tick(t(at))&&!s.fault_latched());}
 assert(wait.Update(t(10.1),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(10.59),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(10.6),true,0)==LowCmdWaitResult::CLEAR);
 s.Observe(input(10.6),t(10.6));assert(s.EnableOutput(true,t(10.6)).success);
 auto first=s.Tick(t(10.6));assert(first&&first->motor_cmd[0].q==.4F);wait.Reset();assert(!wait.active());
 // A publisher appearing during the quiet interval restarts that interval.
 assert(wait.Update(t(11),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(11.4),true,1)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(11.5),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(11.99),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(12),true,0)==LowCmdWaitResult::CLEAR);
 // Loss of fresh Sport confirmation also resets quiet time, even with no endpoint.
 wait.Reset();assert(wait.Update(t(13),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(13.4),false,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(13.5),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(13.99),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(14),true,0)==LowCmdWaitResult::CLEAR);
 // Publisher disappearance at the deadline cannot enable output.
 wait.Reset();assert(wait.Update(t(15),true,1)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(34.99),true,1)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(35),true,0)==LowCmdWaitResult::TIMEOUT);
 assert(wait.Update(t(36),false,0)==LowCmdWaitResult::TIMEOUT);
 wait.Reset();assert(wait.AgeSeconds(t(36))==0);
 assert(wait.Update(t(36),true,0)==LowCmdWaitResult::WAITING);
 assert(wait.Update(t(36.5),true,0)==LowCmdWaitResult::CLEAR);
 for(auto values:{std::pair{0.,.5},std::pair{20.,0.},std::pair{.5,.5},std::pair{20.,-1.},
                  std::pair{std::numeric_limits<double>::quiet_NaN(),.5},std::pair{20.,std::numeric_limits<double>::infinity()}}){
  bool threw=false;try{LowCmdDiscoveryWait invalid(values.first,values.second);}catch(const std::invalid_argument&){threw=true;}assert(threw);
 }
}
void policy_delay_tests(){
 auto enter=[](R3Supervisor& s){holding(s);assert(s.RequestRl(t(8)).success);assert(s.ConsumePolicyReset());};
 std::array<float,12> action;action.fill(.21F);
 // A single 30 ms computation is allowed; response age starts at acceptance.
 R3Supervisor short_delay(profile());enter(short_delay);auto ticket=short_delay.BeginPolicy(t(8));assert(ticket);
 assert(!short_delay.BeginPolicy(t(8.001))); // No queue of simultaneous jobs.
 short_delay.Observe(input(8.03),t(8.03));assert(short_delay.PolicyResult(*ticket,action,30,t(8.03)));
 assert(short_delay.deadline_misses()==1&&short_delay.PolicyAgeSeconds(t(8.03))==0);
 assert(std::abs(short_delay.PolicyObservationAgeSeconds(t(8.03))-.03)<1e-8);
 assert(short_delay.Tick(t(8.039))->motor_cmd[0].q==.21F);
 short_delay.Observe(input(8.042),t(8.042));assert(short_delay.Tick(t(8.042))&&!short_delay.fault_latched());
 short_delay.Observe(input(8.072),t(8.072));auto damp=short_delay.Tick(t(8.072));
 assert(short_delay.fault_latched()&&short_delay.last_fault()=="policy_result_stale");
 assert(damp&&damp->motor_cmd[0].kp==0&&damp->motor_cmd[0].kd==1.5F);
 for(double delay:{.1,.2,1.}) {
  // IO runs during the stall: timeout latches; late completion cannot revive RL.
  R3Supervisor watched(profile());enter(watched);auto work=watched.BeginPolicy(t(8));assert(work);
  watched.Observe(input(8.042),t(8.042));assert(watched.Tick(t(8.042)));
  assert(watched.fault_latched()&&watched.last_fault()=="policy_inference_timeout");
  watched.Observe(input(8+delay),t(8+delay));assert(!watched.PolicyResult(*work,action,delay*1000,t(8+delay)));
  assert(watched.state()==R3State::EMERGENCY_DAMP&&watched.rejected_policy_results()==1);
  auto packet=watched.Tick(t(8+delay));assert(packet&&packet->motor_cmd[0].kp==0);
  // Executor resumes with fresh inputs before IO gets a turn: old result is still rejected.
  R3Supervisor stalled(profile());enter(stalled);work=stalled.BeginPolicy(t(8));assert(work);
  stalled.Observe(input(8+delay),t(8+delay));assert(!stalled.PolicyResult(*work,action,delay*1000,t(8+delay)));
  assert(stalled.fault_latched()&&stalled.last_fault()=="policy_inference_timeout");
  packet=stalled.Tick(t(8+delay));assert(packet&&packet->motor_cmd[0].kp==0);
 }
 // Time before inference matters too: a quick calculation on an old snapshot is rejected.
 R3Supervisor old_source(profile());enter(old_source);auto in=input(8);in.lowstate_stamp=t(7.975);
 old_source.Observe(in,t(8));ticket=old_source.BeginPolicy(t(8));assert(ticket);
 old_source.Observe(input(8.02),t(8.02));assert(!old_source.PolicyResult(*ticket,action,2,t(8.02)));
 assert(old_source.last_fault()=="policy_observation_stale");
 R3Supervisor already_old(profile());enter(already_old);in=input(8);in.lowstate_stamp=t(7.95);
 already_old.Observe(in,t(8));assert(!already_old.BeginPolicy(t(8))&&already_old.fault_latched());
 // Completion after abort, disable, emergency, or a fault never updates the action.
 for(int mode=0;mode<4;++mode){R3Supervisor s(profile());enter(s);ticket=s.BeginPolicy(t(8));assert(ticket);
  if(mode==0)assert(s.ControlledAbort(t(8.01)).success);
  if(mode==1)assert(s.EnableOutput(false,t(8.01)).success);
  if(mode==2)assert(s.Emergency(t(8.01)).success);
  if(mode==3)s.Fault("injected_fault",t(8.01));
  const auto state=s.state();assert(!s.PolicyResult(*ticket,action,30,t(8.03))&&s.state()==state);
 }
 // A result/exception from a prior RL session cannot affect the new session's job.
 R3Supervisor session(profile());enter(session);auto old=session.BeginPolicy(t(8));assert(old);
 assert(session.RequestHold(t(8.01)).success);assert(session.RequestStand(t(8.02)).success);
 in=input(16.02);in.measured_q=profile().stand;session.Observe(in,t(16.02));session.Tick(t(16.02));
 assert(session.RequestRl(t(16.02)).success);assert(session.ConsumePolicyReset());auto current=session.BeginPolicy(t(16.02));assert(current);
 assert(!session.PolicyResult(*old,action,8000,t(16.025))&&session.policy_inflight()&&!session.fault_latched());
 assert(!session.PolicyFailed(*old,"old_exception",t(16.025))&&session.policy_inflight()&&!session.fault_latched());
 assert(session.PolicyResult(*current,action,5,t(16.025)));
 assert(!session.PolicyResult(*current,action,5,t(16.026))); // Duplicate cannot refresh TTL.
 assert(std::abs(session.PolicyAgeSeconds(t(16.026))-.001)<1e-8);
 // Command changes invalidate pending work without destroying an accepted sample.
 R3Supervisor command(profile());enter(command);assert(policy(command,action,2,t(8)));command.Tick(t(8));
 command.Observe(input(8.01),t(8.01));old=command.BeginPolicy(t(8.01));assert(old);
 assert(command.ManualCommand({.1,0,0},1,t(8.012)).success);
 assert(!command.PolicyResult(*old,action,5,t(8.015))&&!command.fault_latched());
 command.Observe(input(8.02),t(8.02));current=command.BeginPolicy(t(8.02));assert(current);
 assert(command.PolicyResult(*current,action,5,t(8.025))&&command.Tick(t(8.026)));
 // Stale/invalid critical feeds and a current inference exception latch fault.
 for(auto member:{&SafetyReadiness::lowstate_fresh,&SafetyReadiness::remote_fresh,&SafetyReadiness::arm_feedback_ready}){
  R3Supervisor s(profile());enter(s);ticket=s.BeginPolicy(t(8));assert(ticket);in=input(8.01);in.ready.*member=false;
  s.Observe(in,t(8.01));assert(s.fault_latched()&&!s.PolicyResult(*ticket,action,15,t(8.015)));
 }
 for(int feed=0;feed<4;++feed){R3Supervisor s(profile());enter(s);ticket=s.BeginPolicy(t(8));assert(ticket);
  in=input(8.01);
  if(feed==0)in.lowstate_stamp=t(7.509);
  if(feed==1)in.remote_stamp=t(7.759);
  if(feed==2)in.arm_stamp=t(7.759);
  if(feed==3)in.target_stamp=t(7.759);
  s.Observe(in,t(8.01));assert(s.fault_latched()&&!s.PolicyResult(*ticket,action,15,t(8.015)));
 }
 R3Supervisor exception(profile());enter(exception);ticket=exception.BeginPolicy(t(8));assert(ticket);
 assert(exception.PolicyFailed(*ticket,"injected_exception",t(8.01))&&exception.fault_latched());
 // Reproduce the physical timing: arm data 15 ms old, first job 8.5 ms,
 // next job at 20 ms, IO at 27 ms, second result at 28 ms. No false timeout.
 R3Supervisor separate(profile());enter(separate);in=input(8);
 in.arm_stamp=t(7.985);in.target_stamp=t(7.994);separate.Observe(in,t(8));
 ticket=separate.BeginPolicy(t(8));assert(ticket);
 separate.Observe(input(8.0085),t(8.0085));assert(separate.PolicyResult(*ticket,action,8.5,t(8.0085)));
 assert(separate.PolicyAgeSeconds(t(8.0085))==0);
 in=input(8.02);in.arm_stamp=t(8.005);in.target_stamp=t(8.014);separate.Observe(in,t(8.02));
 ticket=separate.BeginPolicy(t(8.02));assert(ticket);
 separate.Observe(input(8.027),t(8.027));assert(separate.Tick(t(8.027))&&!separate.fault_latched());
 assert(separate.PolicyResult(*ticket,action,6.5,t(8.028)));
 assert(separate.Tick(t(8.028))->motor_cmd[0].kp==25);
 // Updating live feeds must not conceal stale arm data captured by a job.
 R3Supervisor old_arm(profile());enter(old_arm);in=input(8);in.arm_stamp=t(7.765);
 old_arm.Observe(in,t(8));ticket=old_arm.BeginPolicy(t(8));assert(ticket);
 old_arm.Observe(input(8.02),t(8.02));assert(!old_arm.PolicyResult(*ticket,action,8,t(8.02)));
 assert(old_arm.last_fault()=="policy_arm_observation_stale");
 std::cout<<"PASS independent policy response/job deadlines, leg/arm source freshness, 30/100/200/1000ms delays, executor stall, session/command cancellation and physical timing regression\n";
}
void remote_test_motion_tests(){
 auto p=profile();RemoteStatus r;r.remote_valid=true;r.ly=1;r.rx=-1;r.lx=.5;
 assert((RemoteStickCommand(r,p)==std::array<double,3>{.2,.1,-.05}));
 r.ly=-2;r.rx=2;r.lx=-2;assert((RemoteStickCommand(r,p)==std::array<double,3>{-.2,-.1,.1}));
 r.ly=r.rx=r.lx=.005F;assert((RemoteStickCommand(r,p)==std::array<double,3>{}));
 r.remote_valid=false;r.ly=1;assert((RemoteStickCommand(r,p)==std::array<double,3>{}));
 auto raw=remote(0);float lx=.5F,rx=-1.F,ly=1.F;
 std::memcpy(raw.data()+4,&lx,4);std::memcpy(raw.data()+8,&rx,4);std::memcpy(raw.data()+20,&ly,4);
 RemoteSafety decode;assert(decode.Receive(raw,t(8)));decode.Poll(t(8));
 assert((RemoteStickCommand(decode.status(),p)==std::array<double,3>{.2,.1,-.05}));
 decode.Poll(t(8.251));assert((RemoteStickCommand(decode.status(),p)==std::array<double,3>{}));
 ly=std::numeric_limits<float>::quiet_NaN();std::memcpy(raw.data()+20,&ly,4);assert(!decode.Receive(raw,t(8.3)));
 R3Supervisor idle(p);assert(!idle.RemoteTestCommand({.1,0,0},t(0)).success&&!idle.output_enabled());
 R3Supervisor s(p);holding(s);assert(s.RequestRl(t(8)).success);assert(s.ConsumePolicyReset());
 assert(!s.RemoteTestCommand({.1,0,0},t(8)).success); // First inference must use zero command.
 std::array<float,12> q;q.fill(.21F);assert(policy(s,q,2,t(8)));s.Tick(t(8));
 assert(!s.ManualCommand({.1,.05,0},1,t(8)).success); // Existing single-axis mode unchanged.
 assert(s.NavigationCommand({.1,.05,-.1},t(8)).success&&s.navigation_active());
 assert(s.RemoteTestCommand({.1,.05,-.1},t(8)).success&&s.state()==R3State::RL_ACTIVE);
 assert(!s.RemoteTestCommand({.201,0,0},t(8)).success);
 assert(!s.RemoteTestCommand({0,.101,0},t(8)).success);
 assert(!s.RemoteTestCommand({0,0,.101},t(8)).success);
 auto old=s.BeginPolicy(t(8));assert(old);
 assert(s.RemoteTestCommand({},t(8.01)).success&&s.state()==R3State::RL_ZERO);
 assert(!s.PolicyResult(*old,q,20,t(8.02))&&(s.command()==std::array<double,3>{}));
 s.Observe(input(8.02),t(8.02));assert(s.RemoteTestCommand({-.1,0,0},t(8.02)).success);
 assert(policy(s,q,2,t(8.02))&&s.Tick(t(8.02)));
 s.Observe(input(8.271),t(8.271));s.Tick(t(8.271));
 assert(s.state()==R3State::CONTROLLED_ABORT&&(s.command()==std::array<double,3>{}));
 assert(!s.RemoteTestCommand({.1,0,0},t(8.271)).success); // No joystick recovery after stop/abort.
 std::cout<<"PASS workshop remote test: ly/-rx/-lx, combined axes, deadband/bounds, neutral RL_ZERO, first zero policy, deadman, cancellation and no auto-arm\n";
}
void real_sequence_and_interruptions(){
 auto p=profile();p.stand_s=6;p.hold_s=4;p.capture_hold_s=.02;
 std::array<float,12> kd;kd.fill(3);p.emergency_kd=kd;
 R3Supervisor cycle(p);cycle.Observe(input(0,SportMode::ACTIVE),t(0));
 assert(cycle.StartRemoteSequence(t(0)).success);
 cycle.Observe(input(0,SportMode::ACTIVE),t(0));
 assert(cycle.RemoteSequenceNext(t(0))==R3SequenceAction::RELEASE_SPORT);
 cycle.Observe(input(.1),t(.1));assert(cycle.RemoteSequenceNext(t(.1))==R3SequenceAction::ENABLE_OUTPUT);
 assert(cycle.EnableOutput(true,t(.1)).success&&cycle.Tick(t(.1)));
 assert(cycle.RemoteSequenceNext(t(.1))==R3SequenceAction::NONE);
 cycle.Observe(input(.121),t(.121));assert(cycle.RemoteSequenceNext(t(.121))==R3SequenceAction::STAND);
 assert(cycle.RequestStand(t(.121)).success);
 cycle.Observe(input(1.621),t(1.621));auto packet=cycle.Tick(t(1.621));
 for(int i=0;i<12;++i) {
  const auto& m=packet->motor_cmd[i];
  assert(std::abs(m.q-.35F)<1e-6&&m.kp==40&&m.kd==1&&m.dq==0&&m.tau==0);
 }
 // Stand completion is time-driven even when measured q has not reached target.
 cycle.Observe(input(6.122),t(6.122));packet=cycle.Tick(t(6.122));
 assert(cycle.state()==R3State::HOLDING&&!cycle.fault_latched());
 for(int i=0;i<12;++i)assert(packet->motor_cmd[i].q==p.stand[i]&&packet->motor_cmd[i].kp==40&&packet->motor_cmd[i].kd==1);
 assert(cycle.RemoteSequenceNext(t(6.122))==R3SequenceAction::NONE);
 cycle.Observe(input(10.121),t(10.121));assert(cycle.RemoteSequenceNext(t(10.121))==R3SequenceAction::NONE);
 cycle.Observe(input(10.123),t(10.123));assert(cycle.RemoteSequenceNext(t(10.123))==R3SequenceAction::RL);
 assert(cycle.RequestRl(t(10.123)).success&&cycle.ConsumePolicyReset());
 auto job=cycle.BeginPolicy(t(10.123));assert(job);
 // Stop arrives during the first policy calculation; all twelve measured q hold.
 auto stopped=input(10.124);for(int i=0;i<12;++i)stopped.measured_q[i]=.1F+i*.01F;
 cycle.Observe(stopped,t(10.124));assert(cycle.ControlledAbort(t(10.124)).success);
 std::array<float,12> stale;stale.fill(.6F);
 assert(!cycle.PolicyResult(*job,stale,2,t(10.125)));
 packet=cycle.Tick(t(10.125));assert(packet&&cycle.AllowsPacket(*packet,t(10.125)));
 for(int i=0;i<12;++i)assert(packet->motor_cmd[i].q==stopped.measured_q[i]&&packet->motor_cmd[i].kp==40&&packet->motor_cmd[i].kd==1);
 // Emergency from that hold is exactly passive 0/3, and cannot re-enter RL.
 assert(cycle.Emergency(t(10.125)).success);packet=cycle.Tick(t(10.125));
 assert(packet&&cycle.AllowsPacket(*packet,t(10.125)));
 for(int i=0;i<12;++i)assert(packet->motor_cmd[i].kp==0&&packet->motor_cmd[i].kd==3&&packet->motor_cmd[i].dq==0&&packet->motor_cmd[i].tau==0);
 assert(!cycle.StartRemoteSequence(t(10.125)).success&&!cycle.RequestRl(t(10.125)).success);
 // X also interrupts the stand without completing its target trajectory.
 R3Supervisor lift(p);arm(lift);lift.Tick(t(0));assert(lift.RequestStand(t(0)).success);
 lift.Observe(input(1.5),t(1.5));assert(lift.Tick(t(1.5)));
 auto lift_stop=input(1.501);lift_stop.measured_q=stopped.measured_q;
 lift.Observe(lift_stop,t(1.501));assert(lift.ControlledAbort(t(1.501)).success);
 packet=lift.Tick(t(1.501));for(int i=0;i<12;++i)assert(packet->motor_cmd[i].q==stopped.measured_q[i]);
 std::cout<<"PASS real 6s linear/4s hold cycle, X during first inference/stand captures all12, late result rejected, B passive0/3 and no recovery\n";
}
int main(){real_sequence_and_interruptions();state_tests();gates();remote_tests();lease();automatic_sequence_tests();home_readiness_gate_tests();discovery_wait_tests();policy_delay_tests();remote_test_motion_tests();std::cout<<"PASS R3 synthetic/mocks: measured first HOLD_CURRENT, continuity, RL reset, manual bounds/deadman, abort/lying/stop/stock ordering, critical faults and emergency priority/packet, exclusive output lease, bounded post-release DDS wait; no hardware\n";}
