#pragma once
#include "safety_io.hpp"
#include <yaml-cpp/yaml.h>
namespace sim2real {
enum class R3State { STOCK,DISARMED,TAKEOVER_REQUESTED,PRECHECK,SPORT_RELEASE_REQUIRED,
 SPORT_RELEASE_VERIFIED,LOW_LEVEL_ARMED,HOLD_CURRENT,STAND_TRANSITION,HOLDING,
 RL_ZERO,RL_ACTIVE,CONTROLLED_ABORT,LIE_DOWN_TRANSITION,OUTPUT_STOPPING,RETURN_TO_STOCK,
 FAULT_LATCHED,EMERGENCY_DAMP };
const char* R3StateName(R3State state);
struct R3Profile {
 std::array<float,12> stand{},kp{},kd{},rl_kp{},rl_kd{};
 std::optional<std::array<float,12>> lie_down;
 std::optional<std::array<float,12>> emergency_kd;
 bool policy_timing_reviewed=false;
 bool gate0_verified=false, mapping_verified=false, emergency_validated=false;
 bool remote_chords_verified=false, robot_supported=false, lie_down_validated=false;
 std::string emergency_evidence;
 double stand_s=8,hold_s=1,lie_down_s=8,sport_timeout_s=.5,command_timeout_s=.25;
 double lowstate_timeout_s=.5,remote_timeout_s=.25,arm_timeout_s=.25;
 double vx_bound=.20,vy_bound=.10,wz_bound=.10,max_command_duration_s=1;
 double q_capture_tolerance=.01;
 int deadline_burst_limit=3;
};
R3Profile LoadR3Profile(const YAML::Node& yaml);
struct R3Inputs {
 SafetyReadiness ready;
 std::array<float,12> measured_q{};
 SportMode sport=SportMode::UNKNOWN;
 SafetyTime sport_stamp{},lowstate_stamp{},remote_stamp{},arm_stamp{},target_stamp{};
 bool arm_static_hold=false;
};
struct R3Reply { bool success=false;std::string message; };
enum class R3SequenceAction { NONE, RELEASE_SPORT, ENABLE_OUTPUT, STAND, RL };
class R3Supervisor {
 public:
 explicit R3Supervisor(R3Profile profile);
 void Observe(const R3Inputs& in,SafetyTime now);
 R3Reply Takeover(SafetyTime now);
 R3Reply StartRemoteSequence(SafetyTime now);
 R3SequenceAction RemoteSequenceNext(SafetyTime now);
 void CancelRemoteSequence();
 bool remote_sequence_active() const {return remote_sequence_;}
 std::vector<std::string> RemoteSequenceBlockers(SafetyTime now) const;
 R3Reply EnableOutput(bool enable,SafetyTime now);
 R3Reply RequestHold(SafetyTime now);
 R3Reply RequestStand(SafetyTime now);
 R3Reply RequestRl(SafetyTime now);
 R3Reply RequestLieDown(SafetyTime now);
 R3Reply ControlledAbort(SafetyTime now);
 R3Reply Emergency(SafetyTime now);
 R3Reply ManualCommand(const std::array<double,3>& command,double duration_s,SafetyTime now);
 R3Reply RequestReturnToStock(SafetyTime now);
 void ConfirmStockObserved(SafetyTime now);
 void ConfirmOutputStopped();
 void Fault(const std::string& reason,SafetyTime now);
 void RenewDeadman(SafetyTime now);
 void PolicyResult(const std::array<float,12>& motor_q,double elapsed_ms,SafetyTime now);
 std::optional<unitree_go::msg::LowCmd> Tick(SafetyTime now);
 bool AllowsPacket(const unitree_go::msg::LowCmd& cmd,SafetyTime now) const;
 R3State state() const {return state_;}
 bool output_enabled() const {return output_enabled_;}
 bool fault_latched() const {return fault_;}
 bool navigation_active() const {return navigation_active_;}
 bool NeedsPolicy() const {return state_==R3State::RL_ZERO||state_==R3State::RL_ACTIVE;}
 bool ConsumePolicyReset();
 bool ConsumeArmHoldRequest();
 const std::array<double,3>& command() const {return command_;}
 const R3Profile& profile() const {return profile_;}
 std::vector<std::string> Blockers(SafetyTime now,bool physical=true) const;
 const std::string& last_fault() const {return last_fault_;}
 size_t deadline_misses() const {return deadline_misses_;}
 private:
 bool Released(SafetyTime now) const;
 bool Fresh(SafetyTime now,SafetyTime stamp,double timeout) const;
 R3Reply Require(SafetyTime now) const;
 R3Reply Fail(const std::string& message) const;
 void Capture(SafetyTime now);
 void Zero();
 bool IsActive() const;
 R3Profile profile_;R3Inputs inputs_;
 R3State state_=R3State::DISARMED;
 bool remote_sequence_=false,release_requested_=false,sequence_hold_started_=false;
 SafetyTime sequence_hold_stamp_{};
 bool seen_=false,output_enabled_=false,output_stopped_=true,fault_=false,abort_sequence_=false;
 bool navigation_active_=false,reset_policy_=false,arm_hold_request_=false,have_policy_=false;
 std::array<float,12> start_{},target_{},policy_q_{};
 std::array<double,3> command_{};
 SafetyTime transition_{},command_stamp_{},command_end_{},policy_stamp_{};
 bool have_command_=false;
 size_t deadline_misses_=0;int deadline_burst_=0;
 std::string last_fault_;
};
enum class R3RemoteEvent { NONE,TAKEOVER,CONTROLLED_ABORT,EMERGENCY };
class R3RemoteCommands {
 public:
 R3RemoteCommands(std::vector<std::string> abort_chord,std::vector<std::string> emergency_chord,
                  double hold_s=.75,double stale_s=.25);
 void Receive(std::span<const uint8_t> raw,SafetyTime now);
 R3RemoteEvent Poll(SafetyTime now);
 const RemoteStatus& status() const {return remote_.status();}
 private:
 struct Edge { uint16_t mask=0;bool holding=false,fired=false;SafetyTime began{}; };
 bool EdgeEvent(Edge& edge,SafetyTime now);
 RemoteSafety remote_;double hold_s_,stale_s_;bool seen_=false;SafetyTime received_{};
 Edge abort_,emergency_; // TAKEOVER uses SDK named-field decoder in RemoteSafety.
};
} // namespace sim2real
