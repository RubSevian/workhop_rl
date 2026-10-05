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
 p.capture_hold_s=d["capture_hold_sec"]?d["capture_hold_sec"].as<double>():.02;
 p.stand_s=d["stand_duration_sec"].as<double>();p.hold_s=d["hold_transition_sec"].as<double>();
 p.command_timeout_s=d["cmd_vel_timeout_sec"].as<double>();p.sport_timeout_s=d["sport_mode"]["observation_timeout_s"].as<double>();
 p.lowstate_timeout_s=d["lowstate"]["stale_timeout_s"].as<double>();p.remote_timeout_s=d["remote"]["stale_timeout_s"].as<double>();
 p.arm_timeout_s=d["rars01"]["feedback_timeout_s"].as<double>();
 if(d["rars01"]["require_home_ready_for_leg_takeover"])p.require_arm_home_ready=d["rars01"]["require_home_ready_for_leg_takeover"].as<bool>();
 p.vx_bound=r["manual"]["max_vx"].as<double>();p.vy_bound=r["manual"]["max_vy"].as<double>();p.wz_bound=r["manual"]["max_wz"].as<double>();
 p.max_command_duration_s=r["manual"]["max_duration_s"].as<double>();p.q_capture_tolerance=r["hold_current_tolerance_rad"].as<double>();
 p.deadline_burst_limit=r["deadline_burst_limit"].as<int>();
 if(r["lowcmd_discovery_timeout_s"])p.lowcmd_discovery_timeout_s=r["lowcmd_discovery_timeout_s"].as<double>();
 if(r["lowcmd_clear_duration_s"])p.lowcmd_clear_duration_s=r["lowcmd_clear_duration_s"].as<double>();
 p.policy_timing_reviewed=r["policy_timing_reviewed"].as<bool>();
 p.gate0_verified=r["gate0_verified"].as<bool>();p.mapping_verified=r["mapping_imu_verified"].as<bool>();
 p.remote_chords_verified=r["remote_chords_verified"].as<bool>();p.robot_supported=r["robot_supported"].as<bool>();
 p.emergency_validated=r["emergency"]["operator_validated"].as<bool>();p.emergency_evidence=r["emergency"]["evidence"].as<std::string>();
 if(r["emergency"]["motor_kd"] && !r["emergency"]["motor_kd"].IsNull())p.emergency_kd=Array12(r["emergency"]["motor_kd"]);
 if(r["controlled_stop"]){const auto x=r["controlled_stop"];
  if(x["arm_home_timeout_s"])p.arm_home_timeout_s=x["arm_home_timeout_s"].as<double>();
  if(x["arm_home_settle_s"])p.arm_home_settle_s=x["arm_home_settle_s"].as<double>();
 }
 if(r["lie_down"]["timeout_s"])p.lie_down_timeout_s=r["lie_down"]["timeout_s"].as<double>();
 if(r["lie_down"]["reached_tolerance_rad"])p.lie_down_tolerance=r["lie_down"]["reached_tolerance_rad"].as<double>();
 if(r["lie_down"]["settle_s"])p.lie_down_settle_s=r["lie_down"]["settle_s"].as<double>();
 p.lie_down_validated=r["lie_down"]["operator_validated"].as<bool>();p.lie_down_s=r["lie_down"]["duration_s"].as<double>();
 if(r["lie_down"]["motor_q"]&&!r["lie_down"]["motor_q"].IsNull())p.lie_down=Array12(r["lie_down"]["motor_q"]);
 return p;
}
LowCmdDiscoveryWait::LowCmdDiscoveryWait(double timeout_s,double clear_s):timeout_s_(timeout_s),clear_s_(clear_s) {
 if(!std::isfinite(timeout_s)||!std::isfinite(clear_s)||clear_s<=0||timeout_s<=clear_s)
  throw std::invalid_argument("Discovery timeout must exceed positive finite clear duration");
}
void LowCmdDiscoveryWait::Reset(){active_=false;clear_started_=false;}
double LowCmdDiscoveryWait::AgeSeconds(SafetyTime now) const {
 return active_?std::chrono::duration<double>(now-started_).count():0;
}
LowCmdWaitResult LowCmdDiscoveryWait::Update(SafetyTime now,bool sport_released,size_t publishers) {
 if(!active_){active_=true;started_=now;clear_started_=false;}
 if(AgeSeconds(now)>=timeout_s_)return LowCmdWaitResult::TIMEOUT;
 if(!sport_released||publishers!=0){clear_started_=false;return LowCmdWaitResult::WAITING;}
 if(!clear_started_){clear_started_=true;clear_stamp_=now;}
 return std::chrono::duration<double>(now-clear_stamp_).count()>=clear_s_?LowCmdWaitResult::CLEAR:LowCmdWaitResult::WAITING;
}
R3Supervisor::R3Supervisor(R3Profile p,OperationProfile operation):profile_(std::move(p)),operation_(operation),capabilities_(ResolveOperationCapabilities(operation)) {
 Dispatch(SystemEvent::INITIALIZE,SafetyTime{});
 LowCmdDiscoveryWait validate_discovery(profile_.lowcmd_discovery_timeout_s,profile_.lowcmd_clear_duration_s);
 for(double v:{profile_.capture_hold_s,profile_.stand_s,profile_.hold_s,profile_.lie_down_s,profile_.sport_timeout_s,profile_.command_timeout_s,
  profile_.lowstate_timeout_s,profile_.remote_timeout_s,profile_.arm_timeout_s,profile_.vx_bound,profile_.vy_bound,
  profile_.wz_bound,profile_.max_command_duration_s,profile_.q_capture_tolerance})
  if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Positive finite commissioning settings required");
 for(double v:{profile_.arm_home_timeout_s,profile_.lie_down_timeout_s,profile_.lie_down_tolerance})
  if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Positive finite controlled-stop criteria required");
 for(double v:{profile_.arm_home_settle_s,profile_.lie_down_settle_s})
  if(!std::isfinite(v)||v<0)throw std::invalid_argument("Finite nonnegative optional settle required");
 if(profile_.lie_down_validated&&profile_.lie_down_timeout_s<profile_.lie_down_s)throw std::invalid_argument("Lie timeout before trajectory completes");
 if(profile_.vx_bound>.20||profile_.deadline_burst_limit<1)throw std::invalid_argument("R3 vx bound/valid deadline limit");
 for(float q:profile_.stand)if(!std::isfinite(q)||std::abs(q)>3.5F)throw std::invalid_argument("Stand q outside software contract");
 if(profile_.lie_down)for(float q:*profile_.lie_down)if(!std::isfinite(q)||std::abs(q)>3.5F)throw std::invalid_argument("Invalid lie-down pose");
 if(profile_.emergency_kd)for(float kd:*profile_.emergency_kd)if(!std::isfinite(kd)||kd<=0)throw std::invalid_argument("Invalid damping profile");
 MakeLowCmd(profile_.stand,profile_.kp,profile_.kd);MakeLowCmd(profile_.stand,profile_.rl_kp,profile_.rl_kd);
}
R3State R3Supervisor::state() const {
 switch(phase_) {
 case SystemPhase::ARM_RETURN_HOME:case SystemPhase::ARM_HOME_BLOCKED:case SystemPhase::ARM_HOME_SETTLE:return R3State::RL_ZERO;
 case SystemPhase::PD_CAPTURE:return R3State::HOLD_CURRENT;
 case SystemPhase::LIE_DOWN:case SystemPhase::LIE_DOWN_VERIFY:case SystemPhase::LIE_DOWN_BLOCKED:return R3State::LIE_DOWN_TRANSITION;
 case SystemPhase::LIE_DOWN_HOLD:return R3State::HOLDING;
 default:return static_cast<R3State>(phase_);
 }
}
void R3Supervisor::SetPhase(R3State phase){Dispatch(SystemEvent::ADVANCE_PHASE,SafetyTime{},static_cast<SystemPhase>(phase));}
R3Reply R3Supervisor::Dispatch(SystemEvent e,SafetyTime now,SystemPhase p) {
 if(e==SystemEvent::REQUEST_B)return Emergency(now);
 if(fault_&&e!=SystemEvent::ADVANCE_PHASE&&e!=SystemEvent::INITIALIZE&&e!=SystemEvent::DISABLE_OUTPUT&&e!=SystemEvent::OUTPUT_STOPPED)return Fail("central emergency latched");
 switch(e){
 case SystemEvent::INITIALIZE:if(system_state_!=SystemState::INIT)return Fail("already initialized");system_state_=SystemState::STANDBY;phase_=SystemPhase::DISARMED;return {true,"initialized"};
 case SystemEvent::REQUEST_A:return StartRemoteSequence(now);
 case SystemEvent::REQUEST_X:return ControlledAbort(now);
 case SystemEvent::REQUEST_HOLD:return RequestHold(now);
 case SystemEvent::REQUEST_STAND:return RequestStand(now);
 case SystemEvent::REQUEST_RL:return RequestRl(now);
 case SystemEvent::REQUEST_LIE_DOWN:return RequestLieDown(now);
 case SystemEvent::ENABLE_OUTPUT:return EnableOutput(true,now);
 case SystemEvent::DISABLE_OUTPUT:return EnableOutput(false,now);
 case SystemEvent::RETURN_TO_STOCK:return RequestReturnToStock(now);
 case SystemEvent::OUTPUT_STOPPED:ConfirmOutputStopped();return {true,"output stop confirmed"};
 case SystemEvent::CRITICAL_FAULT:Fault("critical_control_fault",now);return {true,"fault latched"};
 case SystemEvent::ADVANCE_PHASE:
  phase_=p;
  switch(p){
   case SystemPhase::STOCK:case SystemPhase::DISARMED:case SystemPhase::OUTPUT_STOPPING:case SystemPhase::RETURN_TO_STOCK:system_state_=SystemState::STANDBY;break;
   case SystemPhase::RL_ZERO:case SystemPhase::RL_ACTIVE:system_state_=SystemState::ACTIVE;break;
   case SystemPhase::FAULT_LATCHED:case SystemPhase::EMERGENCY_DAMP:system_state_=SystemState::EMERGENCY_FAULT;break;
   case SystemPhase::CONTROLLED_ABORT:case SystemPhase::LIE_DOWN_TRANSITION:
   case SystemPhase::ARM_RETURN_HOME:case SystemPhase::ARM_HOME_BLOCKED:case SystemPhase::ARM_HOME_SETTLE:
   case SystemPhase::PD_CAPTURE:case SystemPhase::LIE_DOWN:case SystemPhase::LIE_DOWN_VERIFY:case SystemPhase::LIE_DOWN_BLOCKED:system_state_=SystemState::CONTROLLED_STOP;break;
   case SystemPhase::LIE_DOWN_HOLD:system_state_=SystemState::SYSTEM_HOLD;break;
   case SystemPhase::HOLD_CURRENT:system_state_=capabilities_.takeover_limit==TakeoverLimit::HOLD_CURRENT&&output_enabled_?SystemState::SYSTEM_HOLD:SystemState::TAKEOVER;break;
   default:system_state_=SystemState::TAKEOVER;break;
  }
  // A latched fault can never be downgraded by a late observation/event.
  if(fault_)system_state_=SystemState::EMERGENCY_FAULT;
  return {true,"phase advanced"};
 default:return Fail("unknown system event");
 }
}
R3Reply R3Supervisor::DispatchEvents(std::span<const SystemEvent> events,SafetyTime now){
 for(auto wanted:{SystemEvent::REQUEST_B,SystemEvent::CRITICAL_FAULT,SystemEvent::REQUEST_X,SystemEvent::REQUEST_A})
  if(std::find(events.begin(),events.end(),wanted)!=events.end())return Dispatch(wanted,now);
 return {true,"no request"};
}
SystemPortRequests R3Supervisor::ConsumePortRequests(){return std::exchange(ports_,{});}
void R3Supervisor::ArmHomeRequestAccepted(bool accepted,SafetyTime){
 if(system_state_!=SystemState::CONTROLLED_STOP||!NeedsPolicy())return;
 arm_home_accepted_=accepted;if(!accepted)stop_blocker_="arm_home_request_not_accepted";
}
void R3Supervisor::ArmEmergencyResult(bool,const std::string& detail){arm_emergency_detail_=detail;}
bool R3Supervisor::Fresh(SafetyTime now,SafetyTime stamp,double limit) const {
 const double age=std::chrono::duration<double>(now-stamp).count();return age>=0&&age<=limit;
}
bool R3Supervisor::Released(SafetyTime now) const {return seen_&&inputs_.sport==SportMode::RELEASED&&Fresh(now,inputs_.sport_stamp,profile_.sport_timeout_s);}
R3Reply R3Supervisor::Fail(const std::string& message) const {return {false,message};}
std::vector<std::string> R3Supervisor::Blockers(SafetyTime now,bool physical) const {
 auto b=inputs_.ready.Blockers();std::erase(b,std::string("explicit_output_enable"));
 if(!seen_)b.emplace_back("inputs_unavailable");
 if(!inputs_.arm_control_ready)b.emplace_back("arm_control_not_ready");
 if(system_state_==SystemState::STANDBY||system_state_==SystemState::TAKEOVER) {
  if(capabilities_.require_navigation_ready&&!inputs_.navigation_ready)b.emplace_back("navigation_not_ready");
  if(capabilities_.require_perception_ready&&!inputs_.perception_ready)b.emplace_back("perception_not_ready");
  if(capabilities_.require_emergency_arm_validation&&!inputs_.arm_emergency_validated)b.emplace_back("arm_emergency_not_validated");
 }
 if(!Released(now))b.emplace_back("sport_release_not_fresh_verified");
 if(!Fresh(now,inputs_.lowstate_stamp,profile_.lowstate_timeout_s))b.emplace_back("lowstate_age");
 if(!Fresh(now,inputs_.remote_stamp,profile_.remote_timeout_s))b.emplace_back("remote_age");
 if(!Fresh(now,inputs_.arm_stamp,profile_.arm_timeout_s))b.emplace_back("arm_feedback_age");
 if(!Fresh(now,inputs_.target_stamp,profile_.arm_timeout_s))b.emplace_back("arm_target_age");
 const bool runtime=system_state_==SystemState::ACTIVE||(system_state_==SystemState::CONTROLLED_STOP&&NeedsPolicy());
 if(!runtime&&!inputs_.arm_static_hold)b.emplace_back("arm_static_hold");
 if(!runtime&&profile_.require_arm_home_ready&&!inputs_.arm_home_ready)b.emplace_back("arm_home_not_ready");
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
void R3Supervisor::InvalidatePolicyWork(){++policy_generation_;pending_policy_.reset();}
void R3Supervisor::Capture(SafetyTime now){InvalidatePolicyWork();start_=inputs_.measured_q;target_=start_;transition_=now;have_policy_=false;}
bool R3Supervisor::IsActive() const {return output_enabled_;}
void R3Supervisor::Observe(const R3Inputs& in,SafetyTime now) {
 inputs_=in;seen_=true;
 if(!output_enabled_&&!fault_&&(state()==R3State::DISARMED||state()==R3State::STOCK))
  SetPhase(in.sport==SportMode::ACTIVE&&Fresh(now,in.sport_stamp,profile_.sport_timeout_s)?R3State::STOCK:R3State::DISARMED);
 if(IsActive() && state()!=R3State::EMERGENCY_DAMP && !Blockers(now).empty())Fault("critical_input_or_ownership",now);
 if(state()==R3State::TAKEOVER_REQUESTED||state()==R3State::PRECHECK||state()==R3State::SPORT_RELEASE_REQUIRED) {
  if(!inputs_.ready.model_loaded||!inputs_.ready.config_valid||!inputs_.ready.lowstate_fresh||!inputs_.ready.remote_fresh||
     !inputs_.ready.arm_feedback_ready||!inputs_.ready.arm_target_ready||!inputs_.arm_static_hold||
     (profile_.require_arm_home_ready&&!inputs_.arm_home_ready))SetPhase(R3State::PRECHECK);
  else SetPhase(Released(now)?R3State::SPORT_RELEASE_VERIFIED:R3State::SPORT_RELEASE_REQUIRED);
 }
}
R3Reply R3Supervisor::Takeover(SafetyTime now) {
 if(fault_||output_enabled_||!seen_||!inputs_.ready.remote_fresh||!Fresh(now,inputs_.remote_stamp,profile_.remote_timeout_s))return Fail("takeover blocked: remote/state");
 if(state()!=R3State::STOCK&&state()!=R3State::DISARMED)return Fail("takeover already requested");
 SetPhase(R3State::TAKEOVER_REQUESTED);return {true,"intent only; no output or Sport switch"};
}
std::vector<std::string> R3Supervisor::RemoteSequenceBlockers(SafetyTime now) const {
 auto b=Blockers(now);std::erase(b,std::string("sport_release_not_fresh_verified"));
 if(!capabilities_.physical_leg_output||!capabilities_.allow_sport_release)b.emplace_back("profile_disallows_leg_takeover");
 SystemReadiness r;r.arm_control_ready=inputs_.arm_control_ready;r.arm_home_ready=inputs_.arm_home_ready;
 r.navigation_ready=inputs_.navigation_ready;r.perception_ready=inputs_.perception_ready;r.arm_emergency_validated=inputs_.arm_emergency_validated;
 for(const auto& blocker:r.TakeoverBlockers(capabilities_))if(std::find(b.begin(),b.end(),blocker)==b.end())b.push_back(blocker);
 if(!seen_||(inputs_.sport!=SportMode::ACTIVE&&inputs_.sport!=SportMode::RELEASED)||
    !Fresh(now,inputs_.sport_stamp,profile_.sport_timeout_s))b.emplace_back("sport_state_not_fresh_known");
 if(capabilities_.allow_rl&&!profile_.policy_timing_reviewed)b.emplace_back("policy_timing_not_reviewed");
 return b;
}
void R3Supervisor::CancelRemoteSequence(){remote_sequence_=false;release_requested_=false;sequence_hold_started_=false;}
R3Reply R3Supervisor::StartRemoteSequence(SafetyTime now) {
 const auto b=RemoteSequenceBlockers(now);
 if(!b.empty()){std::ostringstream out;for(const auto& x:b)out<<x<<',';return Fail(out.str());}
 if(system_state_==SystemState::SYSTEM_HOLD) {
  if(!inputs_.own_output_healthy||!output_enabled_||!Released(now))return Fail("restart requires own healthy lease/publisher and fresh released Sport");
  const auto hold=RequestHold(now);if(!hold.success)return hold;
  stop_blocker_.clear();arm_home_accepted_=false;
  remote_sequence_=true;release_requested_=true;sequence_hold_started_=false;
  return {true,"SYSTEM_HOLD restart: reuse own output; fresh measured -> stand -> hold -> reset -> RL_ZERO"};
 }
 // Preserve manually staged/deadman holds; X from ACTIVE uses SYSTEM_HOLD path above.
 if(output_enabled_&&!fault_&&(state()==R3State::HOLDING||state()==R3State::CONTROLLED_ABORT)) {
  const auto hold=RequestHold(now);if(!hold.success)return hold;
  remote_sequence_=true;release_requested_=true;sequence_hold_started_=false;
  return {true,"fresh A: current hold -> stand -> PD hold -> RL_ZERO"};
 }
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
 if(state()==R3State::SPORT_RELEASE_REQUIRED&&!release_requested_){release_requested_=true;return R3SequenceAction::RELEASE_SPORT;}
 if(state()==R3State::SPORT_RELEASE_VERIFIED&&!output_enabled_)return R3SequenceAction::ENABLE_OUTPUT;
 if(capabilities_.takeover_limit==TakeoverLimit::HOLD_CURRENT&&output_enabled_){CancelRemoteSequence();return R3SequenceAction::NONE;}
 if(state()==R3State::HOLD_CURRENT||state()==R3State::HOLDING) {
  if(!sequence_hold_started_){sequence_hold_stamp_=now;sequence_hold_started_=true;}
  if(std::chrono::duration<double>(now-sequence_hold_stamp_).count()>=(state()==R3State::HOLD_CURRENT?profile_.capture_hold_s:profile_.hold_s)) {
   sequence_hold_started_=false;
   return state()==R3State::HOLD_CURRENT?R3SequenceAction::STAND:R3SequenceAction::RL;
  }
 } else sequence_hold_started_=false;
 if(state()==R3State::RL_ZERO||state()==R3State::RL_ACTIVE)CancelRemoteSequence();
 return R3SequenceAction::NONE;
}
R3Reply R3Supervisor::EnableOutput(bool enable,SafetyTime now) {
 if(enable&&!capabilities_.physical_leg_output)return Fail("profile_disallows_leg_output");
 if(!enable){InvalidatePolicyWork();CancelRemoteSequence();output_enabled_=false;Zero();SetPhase(fault_?R3State::FAULT_LATCHED:R3State::OUTPUT_STOPPING);return {true,"stop publisher then confirm output stopped"};}
 if(state()!=R3State::SPORT_RELEASE_VERIFIED||!output_stopped_)return Fail("requires explicit takeover + verified release + previous output stop");
 auto r=Require(now);if(!r.success)return r;
 Capture(now);output_enabled_=true;output_stopped_=false;SetPhase(R3State::LOW_LEVEL_ARMED);Zero();
 return {true,"first packet HOLD_CURRENT measured q; no stand/RL"};
}
R3Reply R3Supervisor::RequestHold(SafetyTime now) {
 if(system_state_==SystemState::CONTROLLED_STOP)return Fail("controlled_stop_in_progress");
 if(!IsActive()||fault_)return Fail("hold requires active nonfault output");
 auto r=Require(now);if(!r.success)return r;
 Zero();Capture(now);SetPhase(R3State::HOLD_CURRENT);return {true,"capture measured current pose"};
}
R3Reply R3Supervisor::RequestStand(SafetyTime now) {
 if(system_state_==SystemState::CONTROLLED_STOP)return Fail("controlled_stop_in_progress");
 if(capabilities_.takeover_limit!=TakeoverLimit::RL)return Fail("profile_limits_takeover_to_hold");
 if(state()!=R3State::HOLD_CURRENT&&state()!=R3State::HOLDING)return Fail("stand requires HOLD_CURRENT/HOLDING");
 auto r=Require(now);if(!r.success)return r;
 Capture(now);Zero();SetPhase(R3State::STAND_TRANSITION);return {true,"linear measured->stand"};
}
R3Reply R3Supervisor::RequestRl(SafetyTime now) {
 if(system_state_==SystemState::CONTROLLED_STOP)return Fail("controlled_stop_in_progress");
 if(!capabilities_.allow_rl)return Fail("profile_disallows_rl");
 if(state()!=R3State::HOLDING)return Fail("RL entry only from HOLDING");
 if(!profile_.policy_timing_reviewed)return Fail("timing review required before RL");
 auto r=Require(now);if(!r.success)return r;
 InvalidatePolicyWork();Zero();reset_policy_=true;have_policy_=false;policy_stamp_=now;SetPhase(R3State::RL_ZERO);return {true,"zero + ResetPolicyState before any inference"};
}
R3Reply R3Supervisor::RequestLieDown(SafetyTime now) {
 if(system_state_==SystemState::CONTROLLED_STOP)return Fail("controlled_stop_in_progress");
 if(capabilities_.takeover_limit!=TakeoverLimit::RL)return Fail("profile_limits_takeover_to_hold");
 if(state()!=R3State::HOLDING)return Fail("lie-down requires HOLDING");
 auto r=Require(now);if(!r.success)return r;
 if(!profile_.lie_down||!profile_.lie_down_validated)return Fail("no operator-verified lie-down pose");
 Capture(now);Zero();SetPhase(R3State::LIE_DOWN_TRANSITION);return {true,"smooth measured->lie-down"};
}
R3Reply R3Supervisor::ControlledAbort(SafetyTime now) {
 CancelRemoteSequence();
 if(!IsActive()||fault_)return Fail("abort requires active custom mode; use emergency for fault");
 if(capabilities_.takeover_limit==TakeoverLimit::HOLD_CURRENT)return RequestHold(now);
 if(system_state_==SystemState::CONTROLLED_STOP)return {true,"controlled stop already requested"};
 if(system_state_==SystemState::ACTIVE) {
  ++orchestration_generation_;InvalidatePolicyWork();Zero();stop_started_=now;arm_home_accepted_=false;home_settling_=lie_reached_=false;
  stop_blocker_.clear();ports_.cancel_navigation=ports_.cancel_manipulation=ports_.arm_return_home=true;
  Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::ARM_RETURN_HOME);
  return {true,"zero RL -> cancel missions -> ARM HOME -> measured PD handoff -> lie-down -> SYSTEM_HOLD"};
 }
 // Preserve interruption before RL: hold the measured pose, no inferred RL session.
 Zero();arm_hold_request_=true;Capture(now);abort_sequence_=false;SetPhase(R3State::CONTROLLED_ABORT);
 return {true,"takeover interrupted; fixed measured hold"};
}
R3Reply R3Supervisor::Emergency(SafetyTime now) {
 if(system_state_==SystemState::EMERGENCY_FAULT)return {true,"central emergency already latched"};
 CancelRemoteSequence();++orchestration_generation_;
 if(!fault_){fault_policy_age_s_=PolicyAgeSeconds(now);fault_inference_age_s_=PolicyInferenceAgeSeconds(now);fault_observation_age_s_=PolicyObservationAgeSeconds(now);}
 InvalidatePolicyWork();Zero();arm_hold_request_=true;fault_=true;have_policy_=false;reset_policy_=false;
 if(last_fault_.empty())last_fault_="operator_emergency";
 ports_.arm_return_home=false;
 ports_.cancel_navigation=capabilities_.physical_leg_output;
 ports_.cancel_manipulation=capabilities_.physical_leg_output||capabilities_.allow_arm_motion;
 ports_.arm_emergency=capabilities_.allow_arm_emergency;
 if(!output_enabled_||!Released(now)||!profile_.emergency_validated||!profile_.emergency_kd||profile_.emergency_evidence.empty()) {
  output_enabled_=false;SetPhase(R3State::FAULT_LATCHED);
  return {true,"emergency latched; leg damping ineligible, output stopped; asynchronous arm port is separately gated"};
 }
 SetPhase(R3State::EMERGENCY_DAMP);return {true,"latched leg damping; cancel missions; asynchronous validated arm emergency; no recovery"};
}
void R3Supervisor::Fault(const std::string& why,SafetyTime now) {
 if(!fault_) {
  fault_policy_age_s_=PolicyAgeSeconds(now);
  fault_inference_age_s_=PolicyInferenceAgeSeconds(now);
  fault_observation_age_s_=PolicyObservationAgeSeconds(now);
 }
 InvalidatePolicyWork();CancelRemoteSequence();last_fault_=why;fault_=true;
 Emergency(now);
}
R3Reply R3Supervisor::ManualCommand(const std::array<double,3>& cmd,double duration,SafetyTime now) {
 if(system_state_==SystemState::CONTROLLED_STOP)return Fail("controlled_stop_velocity_gate_closed");
 if(cmd!=std::array<double,3>{}&&(!capabilities_.allow_nonzero_velocity||capabilities_.command_source!=CommandSource::REMOTE))return Fail("profile_disallows_manual_velocity");
 if(state()!=R3State::RL_ZERO&&state()!=R3State::RL_ACTIVE)return Fail("manual only from RL_ZERO/RL_ACTIVE");
 auto r=Require(now);if(!r.success)return r;
 int axes=0;for(double v:cmd){if(!std::isfinite(v))return Fail("nonfinite command");axes+=v!=0;}
 if(axes>1||std::abs(cmd[0])>profile_.vx_bound||std::abs(cmd[1])>profile_.vy_bound||std::abs(cmd[2])>profile_.wz_bound||
    !std::isfinite(duration)||duration<=0||duration>profile_.max_command_duration_s)return Fail("single-axis bounds/duration violated");
 if(axes==0){Zero();Capture(now);SetPhase(R3State::CONTROLLED_ABORT);abort_sequence_=false;return {true,"zero -> controlled hold"};}
 if(state()==R3State::RL_ZERO&&!have_policy_)return Fail("RL_ZERO has no valid policy sample yet");
 if(command_!=cmd)InvalidatePolicyWork();
 command_=cmd;navigation_active_=false;have_command_=true;command_stamp_=now;
 command_end_=now+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(duration));
 SetPhase(R3State::RL_ACTIVE);return {true,"bounded one-axis deadman command"};
}
void R3Supervisor::RenewDeadman(SafetyTime now) {
 if(state()==R3State::RL_ACTIVE && have_command_ && now>=command_stamp_)command_stamp_=now;
}
std::array<double,3> RemoteStickCommand(const RemoteStatus& r,const R3Profile& p) {
 if(!r.remote_valid)return {};
 // Existing workshop sim controller: vx=ly, vy=-rx, wz=-lx.
 auto axis=[](double v,double limit){return std::isfinite(v)&&std::abs(v)>.01?std::clamp(v,-1.,1.)*limit:0.;};
 return {axis(r.ly,p.vx_bound),axis(-r.rx,p.vy_bound),axis(-r.lx,p.wz_bound)};
}
R3Reply R3Supervisor::RemoteTestCommand(const std::array<double,3>& cmd,SafetyTime now) {return VelocityCommand(cmd,now,false);}
R3Reply R3Supervisor::NavigationCommand(const std::array<double,3>& cmd,SafetyTime now) {return VelocityCommand(cmd,now,true);}
R3Reply R3Supervisor::VelocityCommand(const std::array<double,3>& cmd,SafetyTime now,bool navigation) {
 if(system_state_==SystemState::CONTROLLED_STOP)return Fail("controlled_stop_velocity_gate_closed");
 if(cmd!=std::array<double,3>{}&&(!capabilities_.allow_nonzero_velocity||
    capabilities_.command_source!=(navigation?CommandSource::NAVIGATION:CommandSource::REMOTE)))return Fail("profile_disallows_command_source");
 if(!NeedsPolicy())return Fail("remote test requires RL_ZERO/RL_ACTIVE");
 const auto ready=Require(now);if(!ready.success)return ready;
 for(double v:cmd)if(!std::isfinite(v))return Fail("nonfinite remote test command");
 if(std::abs(cmd[0])>profile_.vx_bound||std::abs(cmd[1])>profile_.vy_bound||std::abs(cmd[2])>profile_.wz_bound)
  return Fail("remote test bounds violated");
 const bool moving=cmd!=std::array<double,3>{};
 if(moving&&!have_policy_)return Fail("remote test awaits valid zero policy sample");
 if(command_!=cmd)InvalidatePolicyWork();
 command_=cmd;navigation_active_=navigation&&moving;have_command_=moving;command_stamp_=now;
 command_end_=now+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(profile_.max_command_duration_s));
 SetPhase(moving?R3State::RL_ACTIVE:R3State::RL_ZERO);
 return {true,moving?(navigation?"bounded navigation command":"bounded workshop sticks"):"zero velocity: RL_ZERO"};
}
std::optional<PolicyTicket> R3Supervisor::BeginPolicy(SafetyTime now) {
 if(!NeedsPolicy()||fault_||!output_enabled_||pending_policy_)return std::nullopt;
 if(reset_policy_){Fault("policy_reset_not_consumed",now);return std::nullopt;}
 if(!Require(now).success){Fault("policy_input_not_ready",now);return std::nullopt;}
 // Leg observation and arm feeds have separate freshness requirements.
 // Arm publication latency must not consume the policy response deadline.
 const auto observation=inputs_.lowstate_stamp;
 if(!Fresh(now,observation,.04)){Fault("policy_observation_stale",now);return std::nullopt;}
 pending_policy_=PolicyTicket{++next_policy_id_,policy_generation_,now,observation,inputs_.arm_stamp,inputs_.target_stamp};
 return pending_policy_;
}
bool R3Supervisor::CurrentPolicy(const PolicyTicket& ticket) const {
 return pending_policy_&&ticket.id==pending_policy_->id&&ticket.generation==policy_generation_&&
  ticket.generation==pending_policy_->generation&&ticket.started==pending_policy_->started&&
  ticket.observation_stamp==pending_policy_->observation_stamp&&ticket.arm_stamp==pending_policy_->arm_stamp&&
  ticket.target_stamp==pending_policy_->target_stamp&&NeedsPolicy()&&!fault_&&output_enabled_;
}
double R3Supervisor::PolicyAgeSeconds(SafetyTime now) const {
 if(fault_)return fault_policy_age_s_;
 return have_policy_?std::chrono::duration<double>(now-policy_stamp_).count():-1;
}
double R3Supervisor::PolicyInferenceAgeSeconds(SafetyTime now) const {
 if(fault_)return fault_inference_age_s_;
 return pending_policy_?std::chrono::duration<double>(now-pending_policy_->started).count():-1;
}
double R3Supervisor::PolicyObservationAgeSeconds(SafetyTime now) const {
 if(fault_)return fault_observation_age_s_;
 return have_policy_?std::chrono::duration<double>(now-policy_observation_stamp_).count():-1;
}
bool R3Supervisor::PolicyFailed(const PolicyTicket& ticket,const std::string& reason,SafetyTime now) {
 if(!CurrentPolicy(ticket)){++rejected_policy_results_;return false;}
 Fault(reason,now);return true;
}
bool R3Supervisor::PolicyResult(const PolicyTicket& ticket,const std::array<float,12>& q,double ms,SafetyTime now) {
 if(!CurrentPolicy(ticket)){++rejected_policy_results_;return false;}
 if(!std::isfinite(ms)||ms<0||now<ticket.started){Fault("invalid_policy_timing",now);return false;}
 // Check the actual job deadline before accepting it; a late completion must
 // never renew the response watchdog even if IO was delayed by the executor.
 if(!Fresh(now,ticket.started,.04)){Fault("policy_inference_timeout",now);return false;}
 for(float v:q)if(!std::isfinite(v)||std::abs(v)>3.5F){Fault("invalid_policy_action",now);return false;}
 if(ms>=20){++deadline_misses_;++deadline_burst_;}else deadline_burst_=0;
 if(deadline_burst_>=profile_.deadline_burst_limit){Fault("policy_deadline_burst",now);return false;}
 if(!Fresh(now,ticket.observation_stamp,.04)){Fault("policy_observation_stale",now);return false;}
 if(!Fresh(now,ticket.arm_stamp,profile_.arm_timeout_s)||!Fresh(now,ticket.target_stamp,profile_.arm_timeout_s)) {
  Fault("policy_arm_observation_stale",now);return false;
 }
 if(!Require(now).success){Fault("policy_input_not_ready",now);return false;}
 pending_policy_.reset();
 policy_q_=q;policy_stamp_=now;policy_observation_stamp_=ticket.observation_stamp;have_policy_=true;return true;
}
bool R3Supervisor::ConsumePolicyReset(){return std::exchange(reset_policy_,false);}
bool R3Supervisor::ConsumeArmHoldRequest(){return std::exchange(arm_hold_request_,false);}
std::optional<unitree_go::msg::LowCmd> R3Supervisor::Tick(SafetyTime now) {
 if(!output_enabled_)return std::nullopt;
 if(state()==R3State::EMERGENCY_DAMP) {
  if(!Released(now)){output_enabled_=false;SetPhase(R3State::FAULT_LATCHED);return std::nullopt;}
  return MakeLowCmd(target_,std::array<float,12>{},*profile_.emergency_kd);
 }
 if(!Require(now).success){Fault("watchdog_or_precondition",now);return Tick(now);}
 if(system_state_==SystemState::ACTIVE&&state()==R3State::RL_ACTIVE&&(!have_command_||!Fresh(now,command_stamp_,profile_.command_timeout_s)||now>=command_end_)) {
  Zero();Capture(now);abort_sequence_=false;SetPhase(R3State::CONTROLLED_ABORT);
 }
 if(system_state_==SystemState::CONTROLLED_STOP&&NeedsPolicy()) {
  const bool home=arm_home_accepted_&&inputs_.arm_home_ready&&inputs_.arm_stamp>=stop_started_&&inputs_.target_stamp>=stop_started_;
  if(home&&LieDownTargetApproved()&&profile_.lie_down_validated) {
   if(!home_settling_){stop_blocker_.clear();home_settling_=true;home_settle_started_=now;Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::ARM_HOME_SETTLE);}
   if(std::chrono::duration<double>(now-home_settle_started_).count()>=profile_.arm_home_settle_s) {
    if(!Fresh(now,inputs_.lowstate_stamp,.04)){Fault("pd_handoff_lowstate_stale",now);return Tick(now);}
    Capture(now);Zero();reset_policy_=false;stop_blocker_.clear();
    Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::PD_CAPTURE);
    return MakeLowCmd(target_,profile_.kp,profile_.kd);
   }
  }else {
   home_settling_=false;
   if(home&&(!LieDownTargetApproved()||!profile_.lie_down_validated)) {
    stop_blocker_=LieDownTargetApproved()?"lie_down_dynamics_not_commissioned":"lie_down_target_not_approved";Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::ARM_HOME_BLOCKED);
   }else if(std::chrono::duration<double>(now-stop_started_).count()>=profile_.arm_home_timeout_s) {
    stop_blocker_="arm_home_timeout";Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::ARM_HOME_BLOCKED);
   }else Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::ARM_RETURN_HOME);
  }
 }
 if(system_state_==SystemState::CONTROLLED_STOP&&!NeedsPolicy()&&phase_!=SystemPhase::CONTROLLED_ABORT&&phase_!=SystemPhase::LIE_DOWN_TRANSITION) {
  const double elapsed=std::chrono::duration<double>(now-transition_).count();
  if(elapsed<0){Fault("clock_order",now);return Tick(now);}
  if(phase_==SystemPhase::PD_CAPTURE)Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::LIE_DOWN);
  if(phase_==SystemPhase::LIE_DOWN||phase_==SystemPhase::LIE_DOWN_VERIFY) {
   const float u=std::clamp(float(elapsed/profile_.lie_down_s),0.F,1.F);
   for(int i=0;i<12;++i)target_[i]=start_[i]*(1-u)+(*profile_.lie_down)[i]*u;
   if(u>=1) {
    Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::LIE_DOWN_VERIFY);
    bool reached=true;for(int i=0;i<12;++i)reached=reached&&std::abs(inputs_.measured_q[i]-target_[i])<=profile_.lie_down_tolerance;
    if(reached){
     if(!lie_reached_){lie_reached_=true;lie_reached_stamp_=now;}
     if(std::chrono::duration<double>(now-lie_reached_stamp_).count()>=profile_.lie_down_settle_s)
      Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::LIE_DOWN_HOLD);
    }else lie_reached_=false;
   }
   if(system_state_!=SystemState::SYSTEM_HOLD&&elapsed>=profile_.lie_down_timeout_s){
    // Explicit fallback: retain last planned fixed-PD target; no target/output jump.
    stop_blocker_="lie_down_timeout_holding_last_planned_target";Dispatch(SystemEvent::ADVANCE_PHASE,now,SystemPhase::LIE_DOWN_BLOCKED);
   }
  }
  return MakeLowCmd(target_,profile_.kp,profile_.kd);
 }
 if(system_state_==SystemState::SYSTEM_HOLD)return MakeLowCmd(target_,profile_.kp,profile_.kd);
 const double elapsed=std::chrono::duration<double>(now-transition_).count();
 if(elapsed<0){Fault("clock_order",now);return std::nullopt;}
 auto interpolate=[&](const std::array<float,12>& goal,double seconds){
  const float u=std::clamp(float(elapsed/seconds),0.F,1.F);
  // Workshop/MuJoCo jointLinearInterpolation: q0*(1-u)+q1*u.
  const float a=u;for(int i=0;i<12;++i)target_[i]=start_[i]*(1-a)+goal[i]*a;return u>=1;
 };
 switch(state()) {
 case R3State::LOW_LEVEL_ARMED:
  SetPhase(R3State::HOLD_CURRENT);break;
 case R3State::STAND_TRANSITION:
  if(interpolate(profile_.stand,profile_.stand_s)) {
   // Reference finishes stand-up by time; measured error is diagnostic only.
   SetPhase(R3State::HOLDING);
   if(abort_sequence_){start_=target_;transition_=now;SetPhase(R3State::LIE_DOWN_TRANSITION);}
  }break;
 case R3State::CONTROLLED_ABORT:
  // Stop RL and hold captured current q before moving through stand/lying.
  if(elapsed>=profile_.hold_s) {
   if(abort_sequence_){start_=target_;transition_=now;SetPhase(R3State::STAND_TRANSITION);}
   else SetPhase(R3State::HOLDING);
  }break;
 case R3State::LIE_DOWN_TRANSITION:
  if(interpolate(*profile_.lie_down,profile_.lie_down_s)) {
   bool reached=true;
   for(int i=0;i<12;++i)reached=reached&&std::abs(inputs_.measured_q[i]-target_[i])<=profile_.q_capture_tolerance;
   if(reached){output_enabled_=false;Zero();SetPhase(R3State::OUTPUT_STOPPING);return std::nullopt;}
   if(elapsed>=profile_.lie_down_s+profile_.hold_s){Fault("lie_down_feedback_not_reached",now);return Tick(now);}
  }
  break;
 case R3State::RL_ZERO:case R3State::RL_ACTIVE:
  if(pending_policy_&&!Fresh(now,pending_policy_->started,.04)){Fault("policy_inference_timeout",now);return Tick(now);}
  if(!have_policy_&&Fresh(now,policy_stamp_,.04))return MakeLowCmd(target_,profile_.kp,profile_.kd);
  if(!have_policy_||!Fresh(now,policy_stamp_,.04)){Fault("policy_result_stale",now);return Tick(now);}
  target_=policy_q_;return MakeLowCmd(target_,profile_.rl_kp,profile_.rl_kd);
 default:break;
 }
 return MakeLowCmd(target_,profile_.kp,profile_.kd);
}
bool R3Supervisor::AllowsPacket(const unitree_go::msg::LowCmd& cmd,SafetyTime now) const {
 if(!output_enabled_||!Released(now))return false;
 if(state()!=R3State::EMERGENCY_DAMP&&!Require(now).success)return false;
 std::array<float,12> q{},kp{},kd{};
 for(int i=0;i<12;++i){const auto& m=cmd.motor_cmd[i];q[i]=m.q;kp[i]=m.kp;kd[i]=m.kd;}
 try {if(SerializeLowCmd(cmd)!=SerializeLowCmd(MakeLowCmd(q,kp,kd)))return false;}catch(...){return false;}
 const auto& expect_kp=NeedsPolicy()&&have_policy_?profile_.rl_kp:profile_.kp;
 const auto& expect_kd=NeedsPolicy()&&have_policy_?profile_.rl_kd:profile_.kd;
 for(int i=0;i<12;++i) {
  if(std::abs(q[i]-target_[i])>profile_.q_capture_tolerance)return false;
  if(state()==R3State::EMERGENCY_DAMP){if(!profile_.emergency_validated||!profile_.emergency_kd||kp[i]!=0||kd[i]!=(*profile_.emergency_kd)[i])return false;}
  else if(kp[i]!=expect_kp[i]||kd[i]!=expect_kd[i])return false;
 }return true;
}
void R3Supervisor::ConfirmOutputStopped() {
 if(output_enabled_)return;output_stopped_=true;
 if(!fault_)SetPhase(R3State::DISARMED);
}
R3Reply R3Supervisor::RequestReturnToStock(SafetyTime now) {
 if(output_enabled_||!output_stopped_)return Fail("stop and remove LowCmd publisher first");
 if(!seen_||!inputs_.ready.lowstate_fresh||!inputs_.ready.motor_state_valid||!Fresh(now,inputs_.lowstate_stamp,profile_.lowstate_timeout_s)||!profile_.lie_down||!profile_.lie_down_validated)return Fail("fresh lying pose confirmation required");
 for(int i=0;i<12;++i)if(!std::isfinite(inputs_.measured_q[i])||std::abs(inputs_.measured_q[i]-(*profile_.lie_down)[i])>profile_.q_capture_tolerance)return Fail("robot not confirmed lying");
 SetPhase(R3State::RETURN_TO_STOCK);return {true,"separate SDK enable still requires operator gate"};
}
void R3Supervisor::ConfirmStockObserved(SafetyTime now) {
 if(state()==R3State::RETURN_TO_STOCK&&output_stopped_&&!output_enabled_&&inputs_.sport==SportMode::ACTIVE&&Fresh(now,inputs_.sport_stamp,profile_.sport_timeout_s)) {
  SetPhase(R3State::STOCK); // fault_ remains latched; no re-arm in this session.
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
