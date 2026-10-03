#include "safety_io.hpp"
#include "gamepad.hpp"
#include <cassert>
#include <cstring>
#include <cmath>
#include <iostream>
#include <limits>
using namespace sim2real;
SafetyTime t(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
std::array<uint8_t,40> packet(uint16_t mask) {
 unitree::common::REMOTE_DATA_RX rx{};rx.RF_RX.btn.value=mask;
 std::array<uint8_t,40> raw{};std::memcpy(raw.data(),&rx.RF_RX,40);return raw;
}
void remote_test() {
 using namespace unitree::common;
 assert(sizeof(xKeySwitchUnion)==2&&sizeof(REMOTE_DATA_RX)==40&&sizeof(xRockerBtnDataStruct)==40);
 assert(offsetof(xRockerBtnDataStruct,btn)==2&&offsetof(xRockerBtnDataStruct,ly)==20);
 xKeySwitchUnion key{};key.components.L1=1;key.components.L2=1;key.components.A=1;assert(key.value==0x0122);
 // Every SDK named bit, not a manually replicated bitfield.
 for(int i=0;i<16;++i) {
  xKeySwitchUnion k{};k.value=uint16_t(1)<<i;
  const bool fields[]{bool(k.components.R1),bool(k.components.L1),bool(k.components.start),bool(k.components.select),
   bool(k.components.R2),bool(k.components.L2),bool(k.components.F1),bool(k.components.F2),bool(k.components.A),bool(k.components.B),
   bool(k.components.X),bool(k.components.Y),bool(k.components.up),bool(k.components.right),bool(k.components.down),bool(k.components.left)};
  for(int j=0;j<16;++j)assert(fields[j]==(i==j));
 }
 RemoteSafety r;assert(!r.Poll(t(100)));assert(!r.status().remote_valid);
 auto hold=packet(key.value),release=packet(0);
 r.Receive(hold,t(0));assert(r.status().button_mask==0x122);assert(r.status().decoded_buttons=="L1+L2+A");
 for(double s:{.2,.4,.6,.74}) {r.Receive(hold,t(s));assert(!r.Poll(t(s)));}
 r.Receive(hold,t(.75));assert(r.Poll(t(.75)));assert(r.status().takeover_request_latched);
 for(double s:{.8,1.,1.2}){r.Receive(hold,t(s));assert(!r.Poll(t(s)));}
 r.Receive(release,t(1.3));assert(!r.status().takeover_request_latched);
 r.Receive(hold,t(1.4));for(double s:{1.6,1.8,2.,2.15})r.Receive(hold,t(s));assert(r.Poll(t(2.15)));
 RemoteSafety stale;stale.Receive(hold,t(0));assert(!stale.Poll(t(.251)));
 stale.Receive(hold,t(.5));assert(!stale.Poll(t(.5)));
 // A missing Poll cannot hide a receive gap.
 RemoteSafety gap;gap.Receive(hold,t(0));gap.Receive(hold,t(1));assert(!gap.Poll(t(1)));
 RemoteSafety invalid;invalid.Receive(hold,t(0));assert(!invalid.Receive(std::span(hold).first(39),t(.1)));
 assert(!invalid.Poll(t(1)));
 auto nan=hold;float v=std::numeric_limits<float>::quiet_NaN();std::memcpy(nan.data()+4,&v,4);
 assert(!invalid.Receive(nan,t(2)));assert(!invalid.Poll(t(3)));
 for(auto partial:{uint16_t(0x22),uint16_t(0x102),uint16_t(0x120)}) {
  RemoteSafety no;auto raw=packet(partial);for(double s:{0.,.2,.4,.6,.8}){no.Receive(raw,t(s));assert(!no.Poll(t(s)));}
 }
}
SafetyReadiness ready() {return {true,true,true,true,true,true,true,true,true,true};}
void advance(SafetyFsm& f,const SafetyReadiness& r){f.TakeoverRequest();f.Update(r,t(0));f.Update(r,t(0));f.Update(r,t(0));f.Update(r,t(0));}
void fsm_test() {
 assert(InterpretSportServiceStatus(0)==SportMode::ACTIVE);
 assert(InterpretSportServiceStatus(1)==SportMode::RELEASED);
 assert(InterpretSportServiceStatus(2)==SportMode::UNKNOWN);
 for(auto mode:{SportMode::ACTIVE,SportMode::RELEASED,SportMode::UNKNOWN,SportMode::ERROR})
  assert(ParseSportMode(SportModeName(mode))==mode);
 assert(ParseSportMode("false")==SportMode::UNKNOWN);
 auto r=ready();SafetyFsm f;assert(f.state()==SafetyState::DISARMED);assert(!f.AllowsOutput(r,t(0)));
 f.TakeoverRequest();assert(f.state()==SafetyState::TAKEOVER_REQUESTED);assert(!f.AllowsOutput(r,t(0)));
 f.Update(r,t(0));assert(f.state()==SafetyState::PRECHECK);f.Update(r,t(0));assert(f.state()==SafetyState::SPORT_RELEASE_REQUIRED);
 for(auto s:{SportMode::ACTIVE,SportMode::UNKNOWN,SportMode::ERROR}){f.ObserveSportMode(s,t(0));assert(!f.AllowsOutput(r,t(0)));}
 f.ObserveSportMode(SportMode::RELEASED,t(0));f.Update(r,t(0));assert(f.state()==SafetyState::SPORT_RELEASE_VERIFIED);
 auto off=r;off.explicit_output_enable=false;f.Update(off,t(0));assert(!f.AllowsOutput(off,t(0)));
 f.Update(r,t(0));assert(f.state()==SafetyState::HOLD_READY);assert(f.AllowsOutput(r,t(0)));
 assert(f.RequestRl(r,t(0)));assert(f.state()==SafetyState::RL_READY);
 using Member=bool SafetyReadiness::*;
 for(Member m:{&SafetyReadiness::model_loaded,&SafetyReadiness::config_valid,&SafetyReadiness::lowstate_fresh,
  &SafetyReadiness::motor_state_valid,&SafetyReadiness::remote_fresh,&SafetyReadiness::arm_feedback_ready,
  &SafetyReadiness::arm_target_ready,&SafetyReadiness::transport_ready,&SafetyReadiness::explicit_output_enable,&SafetyReadiness::no_fault}) {
  auto missing=r;missing.*m=false;assert(!f.AllowsOutput(missing,t(0)));assert(!f.Blockers(missing,t(0)).empty());
 }
 assert(!f.AllowsOutput(r,t(.501)));assert(!f.AllowsOutput(r,t(-.1)));
 f.Update(off,t(0));assert(f.state()==SafetyState::SPORT_RELEASE_VERIFIED);
 f.Fault();f.Disarm();f.TakeoverRequest();f.Update(r,t(0));assert(f.state()==SafetyState::FAULT_LATCHED);assert(!f.AllowsOutput(r,t(0)));
 SafetyFsm fault;auto bad=r;bad.no_fault=false;fault.Update(bad,t(0));assert(fault.state()==SafetyState::FAULT_LATCHED);
}
void lowstate_test() {
 LowStateReader read;unitree_go::msg::LowState msg{};msg.imu_state.quaternion={1,2,3,4};msg.imu_state.gyroscope={5,6,7};
 for(int i=0;i<20;++i){msg.motor_state[i].q=.1F*i;msg.motor_state[i].dq=-.2F*i;}
 msg.wireless_remote=packet(0x122);assert(read.Receive(msg,t(0)));assert(read.Fresh(t(.5)));assert(!read.Fresh(t(.501)));
 const auto& s=read.snapshot();assert((s.quaternion_xyzw==std::array<float,4>{2,3,4,1}));
 for(int i=0;i<12;++i){assert(s.policy_q[io_motor_to_policy[i]]==msg.motor_state[i].q);assert(s.policy_dq[io_motor_to_policy[i]]==msg.motor_state[i].dq);}
 assert(read.Diagnostic(t(.1)).find("motor[0] FR_hip")!=std::string::npos);
 for(float invalid:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
  auto bad=msg;bad.motor_state[0].q=invalid;assert(!read.Receive(bad,t(0)));assert(!read.Fresh(t(0)));
  bad=msg;bad.motor_state[19].dq=invalid;assert(!read.Receive(bad,t(0)));
  bad=msg;bad.imu_state.quaternion[1]=invalid;assert(!read.Receive(bad,t(0)));
  bad=msg;bad.imu_state.gyroscope[2]=invalid;assert(!read.Receive(bad,t(0)));
 }
 msg.imu_state.quaternion={0,0,0,0};assert(!read.Receive(msg,t(0)));
}
void lowcmd_test() {
 std::array<float,12> q{},kp{},kd{};
 for(int i=0;i<12;++i){q[i]=(i+1)*.125F;kp[i]=20+i;kd[i]=1+i*.125F;}
 auto cmd=MakeLowCmd(q,kp,kd);assert(cmd.crc==0xc0fb03a5U);auto bytes=SerializeLowCmd(cmd);
 assert(bytes[0]==0xfe&&bytes[1]==0xef&&bytes[2]==0xff&&bytes[22]==0&&bytes[23]==0&&bytes[803]==0);
 for(int i=0;i<20;++i){const auto& m=cmd.motor_cmd[i];assert(m.mode==1&&m.tau==0);assert((m.reserve==std::array<uint32_t,3>{}));
  if(i<12)assert(m.q==q[i]&&m.kp==kp[i]&&m.kd==kd[i]&&m.dq==0);
  else assert(m.q==2.146E9F&&m.dq==16000&&m.kp==0&&m.kd==0);
 }
 assert(SerializeLowCmd(MakeLowCmd(q,kp,kd))==bytes);
 LowCmdBytes zeros{};assert(Go2Crc(std::span(zeros).first(808))==0x4b5a4880U);
 const std::array<uint8_t,4> word{1,2,3,4};assert(Go2Crc(word)==0x1dabe74fU);
 auto corrupt=bytes;corrupt[804]^=1;assert(Go2Crc(std::span(corrupt).first(808))!=cmd.crc);
 // CRC field itself is excluded.
 corrupt=bytes;corrupt[811]^=1;assert(Go2Crc(std::span(corrupt).first(808))==cmd.crc);
 MockActuatorTransport mock;SafetyFsm f;auto r=ready();assert(!mock.Send(cmd,f,r,t(0)));assert(mock.sent.empty());
 f.ObserveSportMode(SportMode::RELEASED,t(0));advance(f,r);assert(f.AllowsOutput(r,t(0)));
 auto off=r;off.explicit_output_enable=false;assert(!mock.Send(cmd,f,off,t(0)));assert(mock.sent.empty());
 assert(mock.Send(cmd,f,r,t(0)));assert(mock.sent.size()==1);
 cmd.motor_cmd[19].tau=1;assert(!mock.Send(cmd,f,r,t(0)));
 cmd=MakeLowCmd(q,kp,kd);cmd.crc^=1;assert(!mock.Send(cmd,f,r,t(0)));
 cmd=MakeLowCmd(q,kp,kd);cmd.motor_cmd[0].tau=1;assert(!mock.Send(cmd,f,r,t(0)));
 auto next=MakeLowCmd(std::array<float,12>{},kp,kd);assert(next.motor_cmd[0].q==0&&next.motor_cmd[19].tau==0);
 int sink_calls=0;
 UnitreeLowCmdTransport offline_sink([&](const unitree_go::msg::LowCmd&){++sink_calls;return true;});
 assert(!offline_sink.Send(next,f,off,t(0)));assert(sink_calls==0);
 assert(offline_sink.Send(next,f,r,t(0)));assert(sink_calls==1);
 f.ObserveSportMode(SportMode::ACTIVE,t(0));assert(!offline_sink.Send(next,f,r,t(0)));assert(sink_calls==1);
 UnitreeLowCmdTransport disconnected({});assert(!disconnected.Ready());assert(!disconnected.Send(next,f,r,t(0)));
 bool throws=false;q[0]=4;try{MakeLowCmd(q,kp,kd);}catch(const std::exception&){throws=true;}assert(throws);
}
int main(){remote_test();fsm_test();lowstate_test();lowcmd_test();std::cout<<"PASS SDK ABI/all 16 buttons, 0x0122 hold/watchdogs; FSM all gates; LowState; LowCmd CRC 0xc0fb03a5, mock only\n";}
