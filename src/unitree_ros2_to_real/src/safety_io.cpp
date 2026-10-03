#include "safety_io.hpp"
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <bit>
#include "gamepad.hpp"

namespace sim2real {
static_assert(sizeof(unitree::common::xKeySwitchUnion)==2);
static_assert(sizeof(unitree::common::xRockerBtnDataStruct)==40);
static_assert(sizeof(unitree::common::REMOTE_DATA_RX)==40);
static_assert(offsetof(unitree::common::xRockerBtnDataStruct,btn)==2);
static_assert(offsetof(unitree::common::xRockerBtnDataStruct,lx)==4);
namespace {
bool FreshAge(SafetyTime now, SafetyTime stamp, double timeout) {
 const double age=std::chrono::duration<double>(now-stamp).count();
 return age>=0 && age<=timeout;
}
void Positive(double value) {
 if(!std::isfinite(value)||value<=0) throw std::invalid_argument("Timeout/hold must be finite and positive");
}
constexpr std::array<const char*,16> buttons{"R1","L1","start","select","R2","L2","F1","F2","A","B","X","Y","up","right","down","left"};
}
RemoteSafety::RemoteSafety(double hold_s,double stale_s):hold_s_(hold_s),stale_s_(stale_s) { Positive(hold_s);Positive(stale_s); }
bool RemoteSafety::Receive(std::span<const uint8_t> raw,SafetyTime now) {
 if(seen_ && (!valid_ || !FreshAge(now,stamp_,stale_s_))) holding_=false;
 seen_=true; stamp_=now; valid_=false;
 status_.button_mask=0; status_.decoded_buttons.clear();
 if(raw.size()==40) {
  unitree::common::REMOTE_DATA_RX rx{};
  std::memcpy(&rx.RF_RX,raw.data(),40);
  const auto& r=rx.RF_RX;
  valid_=std::isfinite(r.lx)&&std::isfinite(r.rx)&&std::isfinite(r.ry)&&std::isfinite(r.L2)&&std::isfinite(r.ly);
  if(valid_) {
   status_.button_mask=r.btn.value;
   for(unsigned i=0;i<16;++i) if(r.btn.value&(uint16_t(1)<<i)) {
    if(!status_.decoded_buttons.empty()) status_.decoded_buttons+='+';
    status_.decoded_buttons+=buttons[i];
   }
   chord_=r.btn.components.L1 && r.btn.components.L2 && r.btn.components.A;
   if(!chord_) { holding_=false; fired_=false; }
   else if(!holding_ && !fired_) { holding_=true; hold_start_=now; }
  }
 }
 if(!valid_) { chord_=false; holding_=false; }
 status_.remote_valid=valid_; status_.remote_age_ms=0;
 status_.takeover_hold_active=holding_;
 status_.takeover_request_latched=fired_;
 return valid_;
}
bool RemoteSafety::Poll(SafetyTime now) {
 status_.remote_age_ms=seen_?std::chrono::duration<double,std::milli>(now-stamp_).count():-1;
 status_.remote_valid=seen_&&valid_&&FreshAge(now,stamp_,stale_s_);
 if(!status_.remote_valid) holding_=false;
 status_.takeover_hold_active=holding_;
 if(status_.remote_valid && chord_ && holding_ && !fired_ &&
    std::chrono::duration<double>(now-hold_start_).count()>=hold_s_) {
  fired_=true; holding_=false;
  status_.takeover_hold_active=false; status_.takeover_request_latched=true;
  return true;
 }
 status_.takeover_request_latched=fired_;
 return false;
}
const char* StateName(SafetyState s) {
 switch(s) {
 case SafetyState::DISARMED:return "DISARMED";
 case SafetyState::TAKEOVER_REQUESTED:return "TAKEOVER_REQUESTED";
 case SafetyState::PRECHECK:return "PRECHECK";
 case SafetyState::SPORT_RELEASE_REQUIRED:return "SPORT_RELEASE_REQUIRED";
 case SafetyState::SPORT_RELEASE_VERIFIED:return "SPORT_RELEASE_VERIFIED";
 case SafetyState::HOLD_READY:return "HOLD_READY";
 case SafetyState::RL_READY:return "RL_READY";
 case SafetyState::FAULT_LATCHED:return "FAULT_LATCHED";
 } return "UNKNOWN";
}
std::vector<std::string> SafetyReadiness::Blockers() const {
 std::vector<std::string> out;
 #define CHECK(x) if(!x) out.emplace_back(#x)
 CHECK(model_loaded); CHECK(config_valid); CHECK(lowstate_fresh); CHECK(motor_state_valid);
 CHECK(remote_fresh); CHECK(arm_feedback_ready); CHECK(arm_target_ready); CHECK(transport_ready);
 CHECK(explicit_output_enable); CHECK(no_fault);
 #undef CHECK
 return out;
}
SafetyFsm::SafetyFsm(double timeout):sport_timeout_s_(timeout) { Positive(timeout); }
void SafetyFsm::SetSportTimeout(double timeout_s) { Positive(timeout_s); sport_timeout_s_=timeout_s; }
void SafetyFsm::TakeoverRequest() { if(!fault_ && state_==SafetyState::DISARMED) state_=SafetyState::TAKEOVER_REQUESTED; }
void SafetyFsm::ObserveSportMode(SportMode mode,SafetyTime received) { sport_=mode;sport_seen_=true;sport_stamp_=received; }
bool SafetyFsm::Released(SafetyTime now) const { return sport_seen_&&sport_==SportMode::RELEASED&&FreshAge(now,sport_stamp_,sport_timeout_s_); }
void SafetyFsm::Fault() { fault_=true;state_=SafetyState::FAULT_LATCHED; }
void SafetyFsm::Disarm() { if(!fault_) state_=SafetyState::DISARMED; }
std::vector<std::string> SafetyFsm::Blockers(const SafetyReadiness& r,SafetyTime now) const {
 auto out=r.Blockers();
 if(!Released(now)) out.emplace_back("sport_mode_release_verified");
 if(fault_) out.emplace_back("fault_latched");
 if(state_==SafetyState::DISARMED) out.emplace_back("takeover_not_requested");
 return out;
}
void SafetyFsm::Update(const SafetyReadiness& r,SafetyTime now) {
 if(!r.no_fault) Fault();
 if(fault_ || state_==SafetyState::DISARMED) return;
 if(state_==SafetyState::TAKEOVER_REQUESTED) { state_=SafetyState::PRECHECK;return; }
 // Output intent is deliberately separate from input prechecks.
 auto inputs=r.Blockers(); std::erase(inputs,std::string("explicit_output_enable"));
 if(!inputs.empty()) { state_=SafetyState::PRECHECK;return; }
 if(!Released(now)) { state_=SafetyState::SPORT_RELEASE_REQUIRED;return; }
 if(state_!=SafetyState::SPORT_RELEASE_VERIFIED && state_!=SafetyState::HOLD_READY && state_!=SafetyState::RL_READY) {
  state_=SafetyState::SPORT_RELEASE_VERIFIED;return;
 }
 if(!r.explicit_output_enable) { state_=SafetyState::SPORT_RELEASE_VERIFIED;return; }
 if(state_!=SafetyState::RL_READY) state_=SafetyState::HOLD_READY;
}
bool SafetyFsm::AllowsOutput(const SafetyReadiness& r,SafetyTime now) const {
 return !fault_&&(state_==SafetyState::HOLD_READY||state_==SafetyState::RL_READY)&&r.Blockers().empty()&&Released(now);
}
bool SafetyFsm::RequestRl(const SafetyReadiness& r,SafetyTime now) {
 if(state_!=SafetyState::HOLD_READY||!AllowsOutput(r,now)) return false;
 state_=SafetyState::RL_READY;return true;
}
LowStateReader::LowStateReader(double timeout):timeout_s_(timeout) { Positive(timeout); }
bool LowStateReader::Receive(const unitree_go::msg::LowState& msg,SafetyTime now) {
 LowStateSnapshot next{};next.received=now;
 auto reject=[&](const char* reason){next.rejection=reason;snapshot_=next;return false;};
 if(msg.motor_state.size()!=20||msg.wireless_remote.size()!=40) return reject("array_size");
 for(size_t i=0;i<msg.motor_state.size();++i) {
  if(!std::isfinite(msg.motor_state[i].q)||!std::isfinite(msg.motor_state[i].dq)) return reject("nonfinite_motor");
  if(i<12) {
   next.motor_q[i]=msg.motor_state[i].q;next.motor_dq[i]=msg.motor_state[i].dq;
   next.policy_q[io_motor_to_policy[i]]=next.motor_q[i];next.policy_dq[io_motor_to_policy[i]]=next.motor_dq[i];
  }
 }
 double norm=0;
 for(float q:msg.imu_state.quaternion) { if(!std::isfinite(q)) return reject("nonfinite_quaternion");norm+=double(q)*q; }
 if(norm<1e-16) return reject("zero_quaternion");
 for(float v:msg.imu_state.gyroscope) if(!std::isfinite(v)) return reject("nonfinite_gyro");
 const auto& q=msg.imu_state.quaternion;
 next.quaternion_xyzw={q[1],q[2],q[3],q[0]};next.gyro=msg.imu_state.gyroscope;
 next.valid=true;snapshot_=next;return true;
}
bool LowStateReader::Fresh(SafetyTime now) const { return snapshot_.valid&&FreshAge(now,snapshot_.received,timeout_s_); }
double LowStateReader::AgeMs(SafetyTime now) const { return snapshot_.valid?std::chrono::duration<double,std::milli>(now-snapshot_.received).count():-1; }
std::string LowStateReader::Diagnostic(SafetyTime now) const {
 std::ostringstream o;o<<"lowstate_age_ms="<<AgeMs(now)<<" fresh="<<Fresh(now)<<" rejection="<<snapshot_.rejection;
 if(!snapshot_.valid) return o.str();
 o<<" quaternion_xyzw=";for(float v:snapshot_.quaternion_xyzw)o<<v<<',';
 o<<" gyro=";for(float v:snapshot_.gyro)o<<v<<',';
 for(size_t i=0;i<12;++i) o<<"\n motor["<<i<<"] "<<policy_joint_names[io_motor_to_policy[i]]<<" q="<<snapshot_.motor_q[i]<<" dq="<<snapshot_.motor_dq[i];
 return o.str();
}
LowCmdBytes SerializeLowCmd(const unitree_go::msg::LowCmd& c) {
 LowCmdBytes b{};
 auto u32=[&](size_t at,uint32_t v){for(int j=0;j<4;++j)b[at+j]=uint8_t(v>>(8*j));};
 auto f32=[&](size_t at,float v){u32(at,std::bit_cast<uint32_t>(v));};
 b[0]=c.head[0];b[1]=c.head[1];b[2]=c.level_flag;b[3]=c.frame_reserve;
 for(int j=0;j<2;++j){u32(4+4*j,c.sn[j]);u32(12+4*j,c.version[j]);}
 b[20]=uint8_t(c.bandwidth);b[21]=uint8_t(c.bandwidth>>8);
 for(size_t i=0;i<20;++i) {
  const size_t at=24+36*i;const auto& m=c.motor_cmd[i];b[at]=m.mode;
  f32(at+4,m.q);f32(at+8,m.dq);f32(at+12,m.tau);f32(at+16,m.kp);f32(at+20,m.kd);
  for(int j=0;j<3;++j)u32(at+24+4*j,m.reserve[j]);
 }
 b[744]=c.bms_cmd.off;for(int j=0;j<3;++j)b[745+j]=c.bms_cmd.reserve[j];
 std::copy(c.wireless_remote.begin(),c.wireless_remote.end(),b.begin()+748);
 std::copy(c.led.begin(),c.led.end(),b.begin()+788);
 std::copy(c.fan.begin(),c.fan.end(),b.begin()+800);b[802]=c.gpio;
 u32(804,c.reserve);u32(808,c.crc);return b;
}
uint32_t Go2Crc(std::span<const uint8_t> bytes) {
 if(bytes.size()%4) throw std::invalid_argument("CRC requires complete little-endian words");
 uint32_t crc=0xffffffffU;
 for(size_t i=0;i<bytes.size();i+=4) {
  uint32_t word=uint32_t(bytes[i])|(uint32_t(bytes[i+1])<<8)|(uint32_t(bytes[i+2])<<16)|(uint32_t(bytes[i+3])<<24);
  for(int bit=31;bit>=0;--bit) {
   const bool high=(crc&0x80000000U)!=0;crc<<=1;
   if(high)crc^=0x04c11db7U;
   if(word&(uint32_t(1)<<bit))crc^=0x04c11db7U;
  }
 }return crc;
}
unitree_go::msg::LowCmd MakeLowCmd(const std::array<float,12>& q,const std::array<float,12>& kp,const std::array<float,12>& kd) {
 unitree_go::msg::LowCmd cmd{};cmd.head={0xfe,0xef};cmd.level_flag=0xff;
 for(size_t i=0;i<20;++i) {
  auto& m=cmd.motor_cmd[i];m.mode=1;m.q=2.146E9F;m.dq=16000.F;m.tau=0;m.kp=0;m.kd=0;m.reserve={0,0,0};
  if(i<12) {
   if(!std::isfinite(q[i])||std::abs(q[i])>3.5F||!std::isfinite(kp[i])||kp[i]<0||!std::isfinite(kd[i])||kd[i]<0)
    throw std::invalid_argument("Nonfinite/out-of-contract LowCmd target or gains");
   m.q=q[i];m.dq=0;m.kp=kp[i];m.kd=kd[i];
  }
 }
 const auto bytes=SerializeLowCmd(cmd);cmd.crc=Go2Crc(std::span(bytes).first(808));return cmd;
}
bool ActuatorTransport::Send(const unitree_go::msg::LowCmd& cmd,const SafetyFsm& fsm,const SafetyReadiness& r,SafetyTime now) {
 if(!Ready()||!fsm.AllowsOutput(r,now))return false;
 // Accept only a complete freshly constructed position+gains packet.
 std::array<float,12> q{},kp{},kd{};
 for(size_t i=0;i<12;++i){q[i]=cmd.motor_cmd[i].q;kp[i]=cmd.motor_cmd[i].kp;kd[i]=cmd.motor_cmd[i].kd;}
 try { if(SerializeLowCmd(MakeLowCmd(q,kp,kd))!=SerializeLowCmd(cmd))return false; }
 catch(const std::exception&){return false;}
 return Transmit(cmd);
}
} // namespace sim2real
