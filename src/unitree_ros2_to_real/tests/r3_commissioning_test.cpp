#include "r3_commissioning.hpp"
#include "output_lease.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <filesystem>
#include <unistd.h>
using namespace sim2real;
SafetyTime t(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
// Synthetic gains/poses below are fixtures, NEVER deployment calibration.
R3Profile profile(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);
 p.stand.fill(.2F);std::array<float,12> lie;lie.fill(-.3F);p.lie_down=lie;
 std::array<float,12> damping;damping.fill(1.5F);p.emergency_kd=damping;
 p.gate0_verified=p.mapping_verified=p.emergency_validated=p.remote_chords_verified=p.robot_supported=p.lie_down_validated=p.policy_timing_reviewed=true;
 p.emergency_evidence="OFFLINE_MOCK_ONLY";return p;}
R3Inputs input(double at,SportMode sport=SportMode::RELEASED){R3Inputs in;
 in.ready={true,true,true,true,true,true,true,true,false,true};in.measured_q.fill(.4F);in.arm_static_hold=true;
 in.sport=sport;in.sport_stamp=in.lowstate_stamp=in.remote_stamp=in.arm_stamp=in.target_stamp=t(at);return in;}
void arm(R3Supervisor& s,double at=0){s.Observe(input(at,SportMode::ACTIVE),t(at));assert(s.state()==R3State::STOCK);assert(s.Takeover(t(at)).success);
 assert(!s.output_enabled());s.Observe(input(at),t(at));assert(s.state()==R3State::SPORT_RELEASE_VERIFIED);assert(s.EnableOutput(true,t(at)).success);}
void holding(R3Supervisor& s){arm(s);auto first=s.Tick(t(0));assert(first);assert(s.AllowsPacket(*first,t(0)));
 for(int i=0;i<12;++i)assert(first->motor_cmd[i].q==.4F);assert(s.state()==R3State::HOLD_CURRENT);
 assert(s.RequestStand(t(0)).success);auto start=s.Tick(t(0));assert(start->motor_cmd[0].q==.4F);
 s.Observe(input(4),t(4));auto middle=s.Tick(t(4));assert(std::abs(middle->motor_cmd[0].q-.3F)<1e-6);
 auto standing=input(8);standing.measured_q=profile().stand;s.Observe(standing,t(8));auto end=s.Tick(t(8));assert(end->motor_cmd[0].q==.2F);assert(s.state()==R3State::HOLDING);}
void state_tests(){
 R3Supervisor s(profile());assert(s.state()==R3State::DISARMED);assert(!s.Tick(t(0)));assert(!s.EnableOutput(true,t(0)).success);
 holding(s);assert(s.RequestRl(t(8)).success);assert(s.state()==R3State::RL_ZERO);assert(!s.navigation_active());assert((s.command()==std::array<double,3>{}));
 assert(s.ConsumePolicyReset());assert(!s.ConsumePolicyReset());std::array<float,12> action;action.fill(.21F);
 s.PolicyResult(action,2,t(8));auto rl=s.Tick(t(8));assert(rl&&rl->motor_cmd[0].kp==25&&s.AllowsPacket(*rl,t(8)));
 assert(!s.ManualCommand({.1,.1,0},1,t(8)).success);assert(!s.ManualCommand({.21,0,0},1,t(8)).success);
 assert(!s.ManualCommand({0,.11,0},1,t(8)).success);assert(!s.ManualCommand({0,0,.11},1,t(8)).success);
 assert(!s.ManualCommand({.1,0,0},2,t(8)).success);assert(!s.ManualCommand({std::numeric_limits<double>::infinity(),0,0},1,t(8)).success);
 assert(s.ManualCommand({.1,0,0},1,t(8)).success);assert(s.state()==R3State::RL_ACTIVE);
 s.Observe(input(8.2),t(8.2));s.RenewDeadman(t(8.2));s.PolicyResult(action,2,t(8.2));assert(s.Tick(t(8.2)));assert(s.state()==R3State::RL_ACTIVE);
 s.Observe(input(8.451),t(8.451));assert(s.Tick(t(8.451)));assert(s.state()==R3State::CONTROLLED_ABORT);assert((s.command()==std::array<double,3>{}));
 s.Observe(input(9.451),t(9.451));s.Tick(t(9.451));assert(s.state()==R3State::HOLDING);
 assert(s.ControlledAbort(t(9.451)).success);assert(s.ConsumeArmHoldRequest());assert(!s.navigation_active());
 s.Observe(input(10.451),t(10.451));s.Tick(t(10.451));assert(s.state()==R3State::STAND_TRANSITION);
 auto abort_standing=input(18.451);abort_standing.measured_q=profile().stand;s.Observe(abort_standing,t(18.451));s.Tick(t(18.451));assert(s.state()==R3State::LIE_DOWN_TRANSITION);
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
 assert(no_tracking.fault_latched()&&no_tracking.last_fault()=="stand_feedback_not_reached");

 auto no_lie=profile();no_lie.lie_down.reset();no_lie.lie_down_validated=false;
 R3Supervisor hold_exit(no_lie);arm(hold_exit);hold_exit.Tick(t(0));assert(hold_exit.ControlledAbort(t(0)).success);
 assert((hold_exit.command()==std::array<double,3>{}));assert(!hold_exit.RequestReturnToStock(t(0)).success);
 R3Supervisor changed(profile());arm(changed);auto shifted=input(0);shifted.measured_q[0]+=.1F;changed.Observe(shifted,t(0));
 auto only_damp=changed.Tick(t(0));assert(changed.fault_latched()&&only_damp&&only_damp->motor_cmd[0].kp==0);

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
 assert(emergency.AllowsPacket(*damp,t(0)));assert(!emergency.RequestStand(t(0)).success);assert(!emergency.RequestRl(t(0)).success);
 auto bad=*damp;bad.motor_cmd[0].kp=1;assert(!emergency.AllowsPacket(bad,t(0)));
 emergency.EnableOutput(false,t(0));emergency.ConfirmOutputStopped();assert(!emergency.EnableOutput(true,t(0)).success);
 R3Supervisor armfail(profile());arm(armfail);auto in=input(0);in.ready.arm_target_ready=false;armfail.Observe(in,t(0));assert(armfail.fault_latched());assert(armfail.state()==R3State::EMERGENCY_DAMP);
 R3Supervisor nan(profile());arm(nan);in=input(0);in.measured_q[0]=std::numeric_limits<float>::quiet_NaN();nan.Observe(in,t(0));assert(nan.fault_latched());
 R3Supervisor deadline(profile());holding(deadline);deadline.RequestRl(t(8));deadline.ConsumePolicyReset();std::array<float,12> q{};
 for(int i=0;i<3;++i)deadline.PolicyResult(q,20,t(8));assert(deadline.deadline_misses()==3&&deadline.fault_latched());
 R3Supervisor reset(profile());holding(reset);reset.RequestRl(t(8));reset.PolicyResult(q,2,t(8));assert(reset.fault_latched());
}
std::array<uint8_t,40> remote(uint16_t mask){std::array<uint8_t,40> raw{};raw[2]=mask&255;raw[3]=mask>>8;return raw;}
void remote_tests(){
 R3RemoteCommands r({"L1","L2","X"},{"L1","L2","B"});
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
void automatic_sequence_tests(){
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
int main(){state_tests();gates();remote_tests();lease();automatic_sequence_tests();std::cout<<"PASS R3 synthetic/mocks: measured first HOLD_CURRENT, continuity, RL reset, manual bounds/deadman, abort/lying/stop/stock ordering, critical faults and emergency priority/packet, exclusive output lease; no hardware\n";}
