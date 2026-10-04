#include "r3_commissioning.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <utility>
#include <stdexcept>
namespace sim2real {
const char* R3StateName(R3State s) {
 switch(s) {
 #define STATE(x) case R3State::x:return #x
 STATE(STOCK);STATE(DISARMED);STATE(TAKEOVER_REQUESTED);STATE(PRECHECK);STATE(SPORT_RELEASE_REQUIRED);
 STATE(SPORT_RELEASE_VERIFIED);STATE(LOW_LEVEL_ARMED);STATE(HOLD_CURRENT);STATE(STAND_TRANSITION);
 STATE(HOLDING);STATE(RL_ZERO);STATE(RL_ACTIVE);STATE(CONTROLLED_ABORT);STATE(LIE_DOWN_TRANSITION);
 STATE(OUTPUT_STOPPING);STATE(RETURN_TO_STOCK);STATE(FAULT_LATCHED);STATE(EMERGENCY_DAMP);
 #undef STATE
 }return "UNKNOWN";
}
namespace {
std::array<float,12> Array12(const YAML::Node& n) {
 const auto v=n.as<std::vector<float>>();if(v.size()!=12)throw std::invalid_argument("Expected twelve values");
 std::array<float,12> a;std::copy(v.begin(),v.end(),a.begin());
 for(float f:a)if(!std::isfinite(f))throw std::invalid_argument("Nonfinite profile");return a;
}
std::array<float,12> Hardware(const std::array<float,12>& p) {
 std::array<float,12> m;for(size_t i=0;i<12;++i)m[i]=p[io_motor_to_policy[i]];return m;
}
uint16_t Chord(const std::vector<std::string>& names) {
 constexpr std::array<const char*,16> all{"R1","L1","start","select","R2","L2","F1","F2","A","B","X","Y","up","right","down","left"};
 uint16_t mask=0;for(const auto& n:names) {
  auto it=std::find(all.begin(),all.end(),n);if(it==all.end())throw std::invalid_argument("Unknown remote button "+n);
  const uint16_t bit=uint16_t(1)<<std::distance(all.begin(),it);
  if(mask&bit)throw std::invalid_argument("Duplicate button");mask|=bit;
 }return mask;
}
}
R3Profile LoadR3Profile(const YAML::Node& yaml) {
 R3Profile p;const auto actor=yaml["go2_rars01"],d=yaml["real_deployment"],r=d["r3_commissioning"];
 p.stand=Hardware(Array12(actor["default_dof_pos"]));p.kp=Hardware(Array12(actor["fixed_kp"]));p.kd=Hardware(Array12(actor["fixed_kd"]));
 p.rl_kp=Hardware(Array12(actor["rl_kp"]));p.rl_kd=Hardware(Array12(actor["rl_kd"]));
 p.stand_s=d["stand_duration_sec"].as<double>();p.hold_s=d["hold_transition_sec"].as<double>();
 p.command_timeout_s=d["cmd_vel_timeout_sec"].as<double>();p.sport_timeout_s=d["sport_mode"]["observation_timeout_s"].as<double>();
 p.lowstate_timeout_s=d["lowstate"]["stale_timeout_s"].as<double>();p.remote_timeout_s=d["remote"]["stale_timeout_s"].as<double>();
 p.arm_timeout_s=d["rars01"]["feedback_timeout_s"].as<double>();
 if(d["rars01"]["require_home_ready_for_leg_takeover"])p.require_arm_home_ready=d["rars01"]["require_home_ready_for_leg_takeover"].as<bool>();
 p.vx_bound=r["manual"]["max_vx"].as<double>();p.vy_bound=r["manual"]["max_vy"].as<double>();p.wz_bound=r["manual"]["max_wz"].as<double>();
 p.max_command_duration_s=r["manual"]["max_duration_s"].as<double>();p.q_capture_tolerance=r["hold_current_tolerance_rad"].as<double>();
 p.deadline_burst_limit=r["deadline_burst_limit"].as<int>();
 p.policy_timing_reviewed=r["policy_timing_reviewed"].as<bool>();
 p.gate0_verified=r["gate0_verified"].as<bool>();p.mapping_verified=r["mapping_imu_verified"].as<bool>();
 p.remote_chords_verified=r["remote_chords_verified"].as<bool>();p.robot_supported=r["robot_supported"].as<bool>();
 p.emergency_validated=r["emergency"]["operator_validated"].as<bool>();p.emergency_evidence=r["emergency"]["evidence"].as<std::string>();
 if(r["emergency"]["motor_kd"] && !r["emergency"]["motor_kd"].IsNull())p.emergency_kd=Array12(r["emergency"]["motor_kd"]);
 p.lie_down_validated=r["lie_down"]["operator_validated"].as<bool>();p.lie_down_s=r["lie_down"]["duration_s"].as<double>();
 if(r["lie_down"]["motor_q"]&&!r["lie_down"]["motor_q"].IsNull())p.lie_down=Array12(r["lie_down"]["motor_q"]);
 return p;
}
R3Supervisor::R3Supervisor(R3Profile p):profile_(std::move(p)) {
 for(double v:{profile_.stand_s,profile_.hold_s,profile_.lie_down_s,profile_.sport_timeout_s,profile_.command_timeout_s,
  profile_.lowstate_timeout_s,profile_.remote_timeout_s,profile_.arm_timeout_s,profile_.vx_bound,profile_.vy_bound,
  profile_.wz_bound,profile_.max_command_duration_s,profile_.q_capture_tolerance})
  if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Positive finite commissioning settings required");
 if(profile_.vx_bound>.20||profile_.deadline_burst_limit<1)throw std::invalid_argument("R3 vx bound/valid deadline limit");
 for(float q:profile_.stand)if(!std::isfinite(q)||std::abs(q)>3.5F)throw std::invalid_argument("Stand q outside software contract");
 if(profile_.lie_down)for(float q:*profile_.lie_down)if(!std::isfinite(q)||std::abs(q)>3.5F)throw std::invalid_argument("Invalid lie-down pose");
 if(profile_.emergency_kd)for(float kd:*profile_.emergency_kd)if(!std::isfinite(kd)||kd<=0)throw std::invalid_argument("Invalid damping profile");
 MakeLowCmd(profile_.stand,profile_.kp,profile_.kd);MakeLowCmd(profile_.stand,profile_.rl_kp,profile_.rl_kd);
}
bool R3Supervisor::Fresh(SafetyTime now,SafetyTime stamp,double limit) const {
 const double age=std::chrono::duration<double>(now-stamp).count();return age>=0&&age<=limit;
}
bool R3Supervisor::Released(SafetyTime now) const {return seen_&&inputs_.sport==SportMode::RELEASED&&Fresh(now,inputs_.sport_stamp,profile_.sport_timeout_s);}
R3Reply R3Supervisor::Fail(const std::string& message) const {return {false,message};}
std::vector<std::string> R3Supervisor::Blockers(SafetyTime now,bool physical) const {
 auto b=inputs_.ready.Blockers();std::erase(b,std::string("explicit_output_enable"));
 if(!seen_)b.emplace_back("inputs_unavailable");
 if(!Released(now))b.emplace_back("sport_release_not_fresh_verified");
 if(!Fresh(now,inputs_.lowstate_stamp,profile_.lowstate_timeout_s))b.emplace_back("lowstate_age");
 if(!Fresh(now,inputs_.remote_stamp,profile_.remote_timeout_s))b.emplace_back("remote_age");
 if(!Fresh(now,inputs_.arm_stamp,profile_.arm_timeout_s))b.emplace_back("arm_feedback_age");
 if(!Fresh(now,inputs_.target_stamp,profile_.arm_timeout_s))b.emplace_back("arm_target_age");
 if(!inputs_.arm_static_hold)b.emplace_back("arm_static_hold");
 if(profile_.require_arm_home_ready&&!inputs_.arm_home_ready)b.emplace_back("arm_home_not_ready");
 if(fault_)b.emplace_back("fault_latched");
 if(physical) {
  if(!profile_.gate0_verified)b.emplace_back("gate0_not_passed");
  if(!profile_.mapping_verified)b.emplace_back("mapping_imu_not_verified");
  if(!profile_.robot_supported)b.emplace_back("robot_support_not_confirmed");
  if(!profile_.remote_chords_verified)b.emplace_back("chords_not_verified_read_only");
  if(!profile_.emergency_validated||!profile_.emergency_kd||profile_.emergency_evidence.empty())b.emplace_back("emergency_profile_not_validated");
 }
 for(float q:inputs_.measured_q)if(!std::isfinite(q)||std::abs(q)>3.5F){b.emplace_back("invalid_measured_q");break;}
 return b;
}
R3Reply R3Supervisor::Require(SafetyTime now) const {
 const auto b=Blockers(now);if(b.empty())return {true,"ready"};
 std::ostringstream s;for(const auto& v:b)s<<v<<',';return Fail(s.str());
}
void R3Supervisor::Zero(){command_={};navigation_active_=false;have_command_=false;}
void R3Supervisor::Capture(SafetyTime now){start_=inputs_.measured_q;target_=start_;transition_=now;have_policy_=false;}
bool R3Supervisor::IsActive() const {return output_enabled_;}
void R3Supervisor::Observe(const R3Inputs& in,SafetyTime now) {
 inputs_=in;seen_=true;
 if(!output_enabled_&&!fault_&&(state_==R3State::DISARMED||state_==R3State::STOCK))
  state_=in.sport==SportMode::ACTIVE&&Fresh(now,in.sport_stamp,profile_.sport_timeout_s)?R3State::STOCK:R3State::DISARMED;
 if(IsActive() && state_!=R3State::EMERGENCY_DAMP && !Blockers(now).empty())Fault("critical_input_or_ownership",now);
 if(state_==R3State::TAKEOVER_REQUESTED||state_==R3State::PRECHECK||state_==R3State::SPORT_RELEASE_REQUIRED) {
  if(!inputs_.ready.model_loaded||!inputs_.ready.config_valid||!inputs_.ready.lowstate_fresh||!inputs_.ready.remote_fresh||
     !inputs_.ready.arm_feedback_ready||!inputs_.ready.arm_target_ready||!inputs_.arm_static_hold||
     (profile_.require_arm_home_ready&&!inputs_.arm_home_ready))state_=R3State::PRECHECK;
  else state_=Released(now)?R3State::SPORT_RELEASE_VERIFIED:R3State::SPORT_RELEASE_REQUIRED;
 }
}
R3Reply R3Supervisor::Takeover(SafetyTime now) {
 if(fault_||output_enabled_||!seen_||!inputs_.ready.remote_fresh||!Fresh(now,inputs_.remote_stamp,profile_.remote_timeout_s))return Fail("takeover blocked: remote/state");
 if(state_!=R3State::STOCK&&state_!=R3State::DISARMED)return Fail("takeover already requested");
 state_=R3State::TAKEOVER_REQUESTED;return {true,"intent only; no output or Sport switch"};
}
std::vector<std::string> R3Supervisor::RemoteSequenceBlockers(SafetyTime now) const {
 auto b=Blockers(now);std::erase(b,std::string("sport_release_not_fresh_verified"));
 if(!seen_||(inputs_.sport!=SportMode::ACTIVE&&inputs_.sport!=SportMode::RELEASED)||
    !Fresh(now,inputs_.sport_stamp,profile_.sport_timeout_s))b.emplace_back("sport_state_not_fresh_known");
 if(!profile_.policy_timing_reviewed)b.emplace_back("policy_timing_not_reviewed");
 if(!profile_.lie_down_validated||!profile_.lie_down)b.emplace_back("lie_down_not_validated");
 return b;
}
void R3Supervisor::CancelRemoteSequence(){remote_sequence_=false;release_requested_=false;sequence_hold_started_=false;}
R3Reply R3Supervisor::StartRemoteSequence(SafetyTime now) {
 const auto b=RemoteSequenceBlockers(now);
 if(!b.empty()){std::ostringstream out;for(const auto& x:b)out<<x<<',';return Fail(out.str());}
 const auto reply=Takeover(now);if(!reply.success)return reply;
 remote_sequence_=true;release_requested_=false;sequence_hold_started_=false;
 return {true,"remote sequence: release verified -> current hold -> stand -> PD hold -> RL_ZERO"};
}
R3SequenceAction R3Supervisor::RemoteSequenceNext(SafetyTime now) {
 if(!remote_sequence_)return R3SequenceAction::NONE;
 if(fault_){CancelRemoteSequence();return R3SequenceAction::NONE;}
 auto blockers=RemoteSequenceBlockers(now);
 // While the SDK releases ownership, stale Sport blocks output but does not cancel the bounded RPC.
 if(release_requested_&&!output_enabled_)std::erase(blockers,std::string("sport_state_not_fresh_known"));
 if(!blockers.empty()){Fault("remote_sequence_precondition",now);return R3SequenceAction::NONE;}
 if(state_==R3State::SPORT_RELEASE_REQUIRED&&!release_requested_){release_requested_=true;return R3SequenceAction::RELEASE_SPORT;}
 if(state_==R3State::SPORT_RELEASE_VERIFIED&&!output_enabled_)return R3SequenceAction::ENABLE_OUTPUT;
 if(state_==R3State::HOLD_CURRENT||state_==R3State::HOLDING) {
  if(!sequence_hold_started_){sequence_hold_stamp_=now;sequence_hold_started_=true;}
  if(std::chrono::duration<double>(now-sequence_hold_stamp_).count()>=profile_.hold_s) {
   sequence_hold_started_=false;
   return state_==R3State::HOLD_CURRENT?R3SequenceAction::STAND:R3SequenceAction::RL;
  }
 } else sequence_hold_started_=false;
 if(state_==R3State::RL_ZERO||state_==R3State::RL_ACTIVE)CancelRemoteSequence();
 return R3SequenceAction::NONE;
}
R3Reply R3Supervisor::EnableOutput(bool enable,SafetyTime now) {
 if(!enable){CancelRemoteSequence();output_enabled_=false;Zero();state_=fault_?R3State::FAULT_LATCHED:R3State::OUTPUT_STOPPING;return {true,"stop publisher then confirm output stopped"};}
 if(state_!=R3State::SPORT_RELEASE_VERIFIED||!output_stopped_)return Fail("requires explicit takeover + verified release + previous output stop");
 auto r=Require(now);if(!r.success)return r;
 Capture(now);output_enabled_=true;output_stopped_=false;state_=R3State::LOW_LEVEL_ARMED;Zero();
 return {true,"first packet HOLD_CURRENT measured q; no stand/RL"};
}
R3Reply R3Supervisor::RequestHold(SafetyTime now) {
 if(!IsActive()||fault_)return Fail("hold requires active nonfault output");
 auto r=Require(now);if(!r.success)return r;
 Zero();Capture(now);state_=R3State::HOLD_CURRENT;return {true,"capture measured current pose"};
}
R3Reply R3Supervisor::RequestStand(SafetyTime now) {
 if(state_!=R3State::HOLD_CURRENT&&state_!=R3State::HOLDING)return Fail("stand requires HOLD_CURRENT/HOLDING");
 auto r=Require(now);if(!r.success)return r;
 if(!profile_.lie_down||!profile_.lie_down_validated)return Fail("verified reverse lie-down profile missing");
 Capture(now);Zero();state_=R3State::STAND_TRANSITION;return {true,"smooth measured->stand"};
}
R3Reply R3Supervisor::RequestRl(SafetyTime now) {
 if(state_!=R3State::HOLDING)return Fail("RL entry only from HOLDING");
 if(!profile_.policy_timing_reviewed)return Fail("timing review required before RL");
 auto r=Require(now);if(!r.success)return r;
 Zero();reset_policy_=true;have_policy_=false;policy_stamp_=now;state_=R3State::RL_ZERO;return {true,"zero + ResetPolicyState before any inference"};
}
R3Reply R3Supervisor::RequestLieDown(SafetyTime now) {
 if(state_!=R3State::HOLDING)return Fail("lie-down requires HOLDING");
 auto r=Require(now);if(!r.success)return r;
 if(!profile_.lie_down||!profile_.lie_down_validated)return Fail("no operator-verified lie-down pose");
 Capture(now);Zero();state_=R3State::LIE_DOWN_TRANSITION;return {true,"smooth measured->lie-down"};
}
R3Reply R3Supervisor::ControlledAbort(SafetyTime now) {
 CancelRemoteSequence();
 if(!IsActive()||fault_)return Fail("abort requires active custom mode; use emergency for fault");
 Zero();arm_hold_request_=true;Capture(now);abort_sequence_=profile_.lie_down.has_value()&&profile_.lie_down_validated;state_=R3State::CONTROLLED_ABORT;
 return {true,abort_sequence_?"arm hold -> leg current hold -> stand -> lie-down -> stop":"arm/leg HOLD_CURRENT; lie-down exit blocked pending validated pose"};
}
R3Reply R3Supervisor::Emergency(SafetyTime now) {
 CancelRemoteSequence();
 if(!IsActive())return Fail("no active custom output");
 Zero();arm_hold_request_=true;fault_=true;have_policy_=false;
 if(!Released(now)||!profile_.emergency_validated||!profile_.emergency_kd||profile_.emergency_evidence.empty()) {
  output_enabled_=false;state_=R3State::FAULT_LATCHED;return Fail("no eligible validated damping: stop output; independent operator emergency required");
 }
 state_=R3State::EMERGENCY_DAMP;return {true,"latched damping; no autonomous recovery"};
}
void R3Supervisor::Fault(const std::string& why,SafetyTime now) {
 CancelRemoteSequence();last_fault_=why;fault_=true;
 if(output_enabled_)Emergency(now);else {Zero();state_=R3State::FAULT_LATCHED;}
}
R3Reply R3Supervisor::ManualCommand(const std::array<double,3>& cmd,double duration,SafetyTime now) {
 if(state_!=R3State::RL_ZERO&&state_!=R3State::RL_ACTIVE)return Fail("manual only from RL_ZERO/RL_ACTIVE");
 auto r=Require(now);if(!r.success)return r;
 int axes=0;for(double v:cmd){if(!std::isfinite(v))return Fail("nonfinite command");axes+=v!=0;}
 if(axes>1||std::abs(cmd[0])>profile_.vx_bound||std::abs(cmd[1])>profile_.vy_bound||std::abs(cmd[2])>profile_.wz_bound||
    !std::isfinite(duration)||duration<=0||duration>profile_.max_command_duration_s)return Fail("single-axis bounds/duration violated");
 if(axes==0){Zero();Capture(now);state_=R3State::CONTROLLED_ABORT;abort_sequence_=false;return {true,"zero -> controlled hold"};}
 if(state_==R3State::RL_ZERO&&!have_policy_)return Fail("RL_ZERO has no valid policy sample yet");
 command_=cmd;navigation_active_=false;have_command_=true;command_stamp_=now;
 command_end_=now+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(duration));
 state_=R3State::RL_ACTIVE;return {true,"bounded one-axis deadman command"};
}
void R3Supervisor::RenewDeadman(SafetyTime now) {
 if(state_==R3State::RL_ACTIVE && have_command_ && now>=command_stamp_)command_stamp_=now;
}
void R3Supervisor::PolicyResult(const std::array<float,12>& q,double ms,SafetyTime now) {
 if(!NeedsPolicy())return;
 if(!std::isfinite(ms)||ms<0){Fault("invalid_policy_timing",now);return;}
 for(float v:q)if(!std::isfinite(v)||std::abs(v)>3.5F){Fault("invalid_policy_action",now);return;}
 if(ms>=20){++deadline_misses_;++deadline_burst_;}else deadline_burst_=0;
 if(deadline_burst_>=profile_.deadline_burst_limit){Fault("policy_deadline_burst",now);return;}
 if(reset_policy_){Fault("policy_reset_not_consumed",now);return;}
 policy_q_=q;policy_stamp_=now;have_policy_=true;
}
bool R3Supervisor::ConsumePolicyReset(){return std::exchange(reset_policy_,false);}
bool R3Supervisor::ConsumeArmHoldRequest(){return std::exchange(arm_hold_request_,false);}
std::optional<unitree_go::msg::LowCmd> R3Supervisor::Tick(SafetyTime now) {
 if(!output_enabled_)return std::nullopt;
 if(state_==R3State::EMERGENCY_DAMP) {
  if(!Released(now)){output_enabled_=false;state_=R3State::FAULT_LATCHED;return std::nullopt;}
  return MakeLowCmd(target_,std::array<float,12>{},*profile_.emergency_kd);
 }
 if(!Require(now).success){Fault("watchdog_or_precondition",now);return Tick(now);}
 if(state_==R3State::RL_ACTIVE&&(!have_command_||!Fresh(now,command_stamp_,profile_.command_timeout_s)||now>=command_end_)) {
  Zero();Capture(now);abort_sequence_=false;state_=R3State::CONTROLLED_ABORT;
 }
 const double elapsed=std::chrono::duration<double>(now-transition_).count();
 if(elapsed<0){Fault("clock_order",now);return std::nullopt;}
 auto interpolate=[&](const std::array<float,12>& goal,double seconds){
  const float u=std::clamp(float(elapsed/seconds),0.F,1.F);
  // Smoothstep starts/ends with zero velocity.
  const float a=u*u*(3-2*u);for(int i=0;i<12;++i)target_[i]=start_[i]*(1-a)+goal[i]*a;return u>=1;
 };
 switch(state_) {
 case R3State::LOW_LEVEL_ARMED:
  for(int i=0;i<12;++i)if(std::abs(inputs_.measured_q[i]-target_[i])>profile_.q_capture_tolerance) {
   Fault("first_hold_capture_changed",now);return Tick(now);
  }
  state_=R3State::HOLD_CURRENT;break;
 case R3State::STAND_TRANSITION:
  if(interpolate(profile_.stand,profile_.stand_s)) {
   bool reached=true;
   for(int i=0;i<12;++i)reached=reached&&std::abs(inputs_.measured_q[i]-target_[i])<=profile_.q_capture_tolerance;
   if(!reached) {
    if(elapsed>=profile_.stand_s+profile_.hold_s){Fault("stand_feedback_not_reached",now);return Tick(now);}
    break;
   }
   state_=R3State::HOLDING;
   if(abort_sequence_){start_=target_;transition_=now;state_=R3State::LIE_DOWN_TRANSITION;}
  }break;
 case R3State::CONTROLLED_ABORT:
  // Stop RL and hold captured current q before moving through stand/lying.
  if(elapsed>=profile_.hold_s) {
   if(abort_sequence_){start_=target_;transition_=now;state_=R3State::STAND_TRANSITION;}
   else state_=R3State::HOLDING;
  }break;
 case R3State::LIE_DOWN_TRANSITION:
  if(interpolate(*profile_.lie_down,profile_.lie_down_s)) {
   bool reached=true;
   for(int i=0;i<12;++i)reached=reached&&std::abs(inputs_.measured_q[i]-target_[i])<=profile_.q_capture_tolerance;
   if(reached){output_enabled_=false;Zero();state_=R3State::OUTPUT_STOPPING;return std::nullopt;}
   if(elapsed>=profile_.lie_down_s+profile_.hold_s){Fault("lie_down_feedback_not_reached",now);return Tick(now);}
  }
  break;
 case R3State::RL_ZERO:case R3State::RL_ACTIVE:
  if(!have_policy_&&Fresh(now,policy_stamp_,.04))return MakeLowCmd(target_,profile_.kp,profile_.kd);
  if(!have_policy_||!Fresh(now,policy_stamp_,.04)){Fault("policy_result_stale",now);return Tick(now);}
  target_=policy_q_;return MakeLowCmd(target_,profile_.rl_kp,profile_.rl_kd);
 default:break;
 }
 return MakeLowCmd(target_,profile_.kp,profile_.kd);
}
bool R3Supervisor::AllowsPacket(const unitree_go::msg::LowCmd& cmd,SafetyTime now) const {
 if(!output_enabled_||!Released(now))return false;
 if(state_!=R3State::EMERGENCY_DAMP&&!Require(now).success)return false;
 std::array<float,12> q{},kp{},kd{};
 for(int i=0;i<12;++i){const auto& m=cmd.motor_cmd[i];q[i]=m.q;kp[i]=m.kp;kd[i]=m.kd;}
 try {if(SerializeLowCmd(cmd)!=SerializeLowCmd(MakeLowCmd(q,kp,kd)))return false;}catch(...){return false;}
 const auto& expect_kp=NeedsPolicy()&&have_policy_?profile_.rl_kp:profile_.kp;
 const auto& expect_kd=NeedsPolicy()&&have_policy_?profile_.rl_kd:profile_.kd;
 for(int i=0;i<12;++i) {
  if(std::abs(q[i]-target_[i])>profile_.q_capture_tolerance)return false;
  if(state_==R3State::EMERGENCY_DAMP){if(!profile_.emergency_validated||!profile_.emergency_kd||kp[i]!=0||kd[i]!=(*profile_.emergency_kd)[i])return false;}
  else if(kp[i]!=expect_kp[i]||kd[i]!=expect_kd[i])return false;
 }return true;
}
void R3Supervisor::ConfirmOutputStopped() {
 if(output_enabled_)return;output_stopped_=true;
 if(!fault_)state_=R3State::DISARMED;
}
R3Reply R3Supervisor::RequestReturnToStock(SafetyTime now) {
 if(output_enabled_||!output_stopped_)return Fail("stop and remove LowCmd publisher first");
 if(!seen_||!inputs_.ready.lowstate_fresh||!inputs_.ready.motor_state_valid||!Fresh(now,inputs_.lowstate_stamp,profile_.lowstate_timeout_s)||!profile_.lie_down||!profile_.lie_down_validated)return Fail("fresh lying pose confirmation required");
 for(int i=0;i<12;++i)if(!std::isfinite(inputs_.measured_q[i])||std::abs(inputs_.measured_q[i]-(*profile_.lie_down)[i])>profile_.q_capture_tolerance)return Fail("robot not confirmed lying");
 state_=R3State::RETURN_TO_STOCK;return {true,"separate SDK enable still requires operator gate"};
}
void R3Supervisor::ConfirmStockObserved(SafetyTime now) {
 if(state_==R3State::RETURN_TO_STOCK&&output_stopped_&&!output_enabled_&&inputs_.sport==SportMode::ACTIVE&&Fresh(now,inputs_.sport_stamp,profile_.sport_timeout_s)) {
  state_=R3State::STOCK; // fault_ remains latched; no re-arm in this session.
 }
}
R3RemoteCommands::R3RemoteCommands(std::vector<std::string> abort,std::vector<std::string> emergency,double hold,double stale)
 :remote_(hold,stale),hold_s_(hold),stale_s_(stale) {
 abort_.mask=Chord(abort);emergency_.mask=Chord(emergency);
 if(abort_.mask==0||emergency_.mask==0||abort_.mask==emergency_.mask||abort_.mask==0x122||emergency_.mask==0x122)
  throw std::invalid_argument("Distinct nonempty operator chords required");
}
void R3RemoteCommands::Receive(std::span<const uint8_t> raw,SafetyTime now) {
 const double gap=std::chrono::duration<double>(now-received_).count();
 if(seen_&&(gap<0||gap>stale_s_)){abort_.holding=false;emergency_.holding=false;}
 remote_.Receive(raw,now);received_=now;seen_=true;
 const auto& r=remote_.status();
 for(Edge* e:{&abort_,&emergency_}) {
  if(!r.remote_valid){e->holding=false;continue;}
  if((r.button_mask&e->mask)!=e->mask){e->holding=false;e->fired=false;}
  else if(!e->holding&&!e->fired){e->holding=true;e->began=now;}
 }
}
bool R3RemoteCommands::EdgeEvent(Edge& e,SafetyTime now) {
 if(!remote_.status().remote_valid){e.holding=false;return false;}
 if(e.holding&&!e.fired&&std::chrono::duration<double>(now-e.began).count()>=hold_s_){e.fired=true;e.holding=false;return true;}
 return false;
}
R3RemoteEvent R3RemoteCommands::Poll(SafetyTime now) {
 const bool takeover=remote_.Poll(now);
 const bool abort=EdgeEvent(abort_,now),emergency=EdgeEvent(emergency_,now);
 // Lower-priority simultaneous edges are consumed, never replayed later.
 return emergency?R3RemoteEvent::EMERGENCY:abort?R3RemoteEvent::CONTROLLED_ABORT:takeover?R3RemoteEvent::TAKEOVER:R3RemoteEvent::NONE;
}
} // namespace sim2real
