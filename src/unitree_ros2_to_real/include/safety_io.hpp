#pragma once
#include <array>
#include "sport_mode_status.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

namespace sim2real {
using SafetyClock = std::chrono::steady_clock;
using SafetyTime = SafetyClock::time_point;
inline constexpr std::array<int,12> io_motor_to_policy{3,4,5,0,1,2,9,10,11,6,7,8};
inline constexpr std::array<const char*,12> policy_joint_names{
 "FL_hip","FL_thigh","FL_calf","FR_hip","FR_thigh","FR_calf",
 "RL_hip","RL_thigh","RL_calf","RR_hip","RR_thigh","RR_calf"};
struct RemoteStatus {
 float lx=0,rx=0,ly=0;
 bool remote_valid=false, takeover_hold_active=false, takeover_request_latched=false;
 double remote_age_ms=-1;
 uint16_t button_mask=0;
 std::string decoded_buttons;
};
class RemoteSafety {
 public:
 RemoteSafety(double hold_s=0.75, double stale_s=0.25);
 bool Receive(std::span<const uint8_t> raw, SafetyTime now);
 // Returns one edge only; a stale stream resets the hold but not the fired latch.
 bool Poll(SafetyTime now);
 const RemoteStatus& status() const { return status_; }
 private:
 double hold_s_, stale_s_;
 RemoteStatus status_;
 bool seen_=false, valid_=false, chord_=false, holding_=false, fired_=false;
 SafetyTime stamp_{}, hold_start_{};
};
enum class SafetyState { DISARMED, TAKEOVER_REQUESTED, PRECHECK,
 SPORT_RELEASE_REQUIRED, SPORT_RELEASE_VERIFIED, HOLD_READY, RL_READY, FAULT_LATCHED };
const char* StateName(SafetyState state);
struct SafetyReadiness {
 bool model_loaded=false, config_valid=false, lowstate_fresh=false, motor_state_valid=false;
 bool remote_fresh=false, arm_feedback_ready=false, arm_target_ready=false;
 bool transport_ready=false, explicit_output_enable=false, no_fault=true;
 std::vector<std::string> Blockers() const;
};
class SafetyFsm {
 public:
 explicit SafetyFsm(double sport_timeout_s=0.5);
 void SetSportTimeout(double timeout_s);
 void TakeoverRequest();
 void ObserveSportMode(SportMode mode, SafetyTime received);
 void Fault();
 void Disarm();
 void Update(const SafetyReadiness& ready, SafetyTime now);
 bool RequestRl(const SafetyReadiness& ready, SafetyTime now);
 bool AllowsOutput(const SafetyReadiness& ready, SafetyTime now) const;
 std::vector<std::string> Blockers(const SafetyReadiness& ready, SafetyTime now) const;
 SafetyState state() const { return state_; }
 private:
 bool Released(SafetyTime now) const;
 double sport_timeout_s_;
 SafetyState state_=SafetyState::DISARMED;
 SportMode sport_=SportMode::UNKNOWN;
 bool sport_seen_=false, fault_=false;
 SafetyTime sport_stamp_{};
};
struct LowStateSnapshot {
 std::array<float,12> motor_q{}, motor_dq{}, policy_q{}, policy_dq{};
 std::array<float,4> quaternion_xyzw{};
 std::array<float,3> gyro{};
 bool valid=false;
 SafetyTime received{};
 std::string rejection;
};
class LowStateReader {
 public:
 explicit LowStateReader(double timeout_s=0.5);
 bool Receive(const unitree_go::msg::LowState& msg, SafetyTime now);
 bool Fresh(SafetyTime now) const;
 double AgeMs(SafetyTime now) const;
 const LowStateSnapshot& snapshot() const { return snapshot_; }
 std::string Diagnostic(SafetyTime now) const;
 private:
 double timeout_s_;
 LowStateSnapshot snapshot_;
};
// CRC input is the SDK's 812-byte little-endian native Go2 layout, NOT DDS CDR.
// Explicit offsets and zero padding avoid reading indeterminate C++ padding.
using LowCmdBytes = std::array<uint8_t,812>;
LowCmdBytes SerializeLowCmd(const unitree_go::msg::LowCmd& cmd);
uint32_t Go2Crc(std::span<const uint8_t> bytes);
unitree_go::msg::LowCmd MakeLowCmd(const std::array<float,12>& motor_q,
 const std::array<float,12>& motor_kp, const std::array<float,12>& motor_kd);
// Canonical passive packet: same layout/CRC and existing stop sentinels.
unitree_go::msg::LowCmd MakePassiveLowCmd();
class ActuatorTransport {
 public:
 virtual ~ActuatorTransport()=default;
 bool Send(const unitree_go::msg::LowCmd& cmd, const SafetyFsm& fsm,
           const SafetyReadiness& ready, SafetyTime now);
 virtual bool Ready() const=0;
 protected:
 virtual bool Transmit(const unitree_go::msg::LowCmd& cmd)=0;
};
class MockActuatorTransport final : public ActuatorTransport {
 public:
 bool Ready() const override { return true; }
 std::vector<unitree_go::msg::LowCmd> sent;
 protected:
 bool Transmit(const unitree_go::msg::LowCmd& cmd) override { sent.push_back(cmd); return true; }
};
// The future ROS publisher is injected by the deployment owner. No publisher
// or DDS channel is constructed by this class or by the read-only executable.
class UnitreeLowCmdTransport final : public ActuatorTransport {
 public:
 using Sink=std::function<bool(const unitree_go::msg::LowCmd&)>;
 explicit UnitreeLowCmdTransport(Sink sink) : sink_(std::move(sink)) {}
 bool Ready() const override { return static_cast<bool>(sink_); }
 protected:
 bool Transmit(const unitree_go::msg::LowCmd& cmd) override { return sink_(cmd); }
 private:
 Sink sink_;
};
} // namespace sim2real
