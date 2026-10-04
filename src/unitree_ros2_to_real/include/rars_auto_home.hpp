#pragma once
#include "rars_arm.hpp"
#include <chrono>
#include <optional>
#include <string>
#include <limits>
namespace sim2real {
using ArmClock = std::chrono::steady_clock;
using ArmTime = ArmClock::time_point;
struct AutoHomeConfig {
 bool enabled=false;
 double startup_delay_s=10, command_rate_hz=100;
 double feedback_timeout_s=.25, target_timeout_s=.25, home_tolerance_rad=.15;
 double connect_retry_s=1, enable_feedback_grace_s=1;
 rars_arm::RarsArm::MotorValues home_target{};
};
// Deliberately has no zero/calibration mutation API.
class AutoHomeTransport {
 public:
 virtual ~AutoHomeTransport()=default;
 virtual bool Connect()=0;
 virtual bool Enable()=0;
 virtual bool Send(const rars_arm::RarsArm::MotorValues& target)=0;
 virtual bool Read(rars_arm::JointState& state)=0;
 virtual rars_arm::CommunicationStatus Status() const=0;
 virtual std::string Error() const=0;
};
// Must durably record the enable attempt BEFORE invoking SDK enable.
class AutoHomeJournal {
 public:
 virtual ~AutoHomeJournal()=default;
 virtual std::string BlockReason() const=0;
 virtual bool RecordEnableAttempt()=0;
 virtual bool RecordFault(const std::string& reason)=0;
};
class FileAutoHomeJournal final:public AutoHomeJournal {
 public:
 FileAutoHomeJournal(std::string path,std::string boot_id);
 std::string BlockReason() const override {return blocked_;}
 bool RecordEnableAttempt() override;
 bool RecordFault(const std::string& reason) override;
 private:
 bool Save(const std::string& value);
 std::string path_,boot_id_,blocked_;
};
enum class AutoHomeState { READ_ONLY, WAIT_DEVICE, WAIT_COMMUNICATION, STARTUP_DELAY, HOLD_HOME, FAULT_LATCHED };
const char* AutoHomeStateName(AutoHomeState state);
struct AutoHomeStatus {
 AutoHomeStatus(){joints.position.fill(std::numeric_limits<float>::quiet_NaN());joints.velocity.fill(std::numeric_limits<float>::quiet_NaN());home_error.fill(std::numeric_limits<float>::quiet_NaN());}
 AutoHomeState state=AutoHomeState::READ_ONLY;
 rars_arm::JointState joints;
 rars_arm::CommunicationStatus communication;
 rars_arm::RarsArm::MotorValues home_error{};
 bool feedback_ready=false,motors_enabled=false,target_fresh=false,arm_home_ready=false,enable_attempted=false;
 double feedback_age_s=-1,target_age_s=-1;
 uint64_t successful_sends=0;double last_send_gap_ms=0,max_send_gap_ms=0;
 std::optional<ArmTime> first_accepted_stamp;
 std::optional<ArmTime> feedback_stamp,accepted_stamp;
 std::string last_error;
};
class AutoHomeController {
 public:
 AutoHomeController(AutoHomeConfig config,AutoHomeTransport& transport,AutoHomeJournal& journal);
 // command_timer_tick is true only for the external configured-rate timer.
 // Other callers retain the internal rate limiter. Never replay missed slots.
 void Tick(ArmTime now,bool connect_allowed=true,bool command_timer_tick=false);
 const AutoHomeStatus& status() const {return status_;}
 const AutoHomeConfig& config() const {return config_;}
 void Fault(const std::string& reason);
 private:
 void Observe(ArmTime now);
 bool UsableFeedback() const;
 AutoHomeConfig config_;AutoHomeTransport& transport_;AutoHomeJournal& journal_;
 AutoHomeStatus status_;
 std::optional<ArmTime> countdown_,enabled_at_,last_read_,next_send_;
 ArmTime next_connect_{};
 bool have_usable_feedback_=false;
 double age_at_read_s_=0;
};
} // namespace sim2real
