#pragma once
#include "safety_io.hpp"
#include "rars_arm.hpp"
#include <memory>
#include <deque>
#include <limits>
namespace sim2real {
struct RarsFrame {
 rars_arm::JointState joints; // Already converted by SDK, never convert twice.
 rars_arm::CommunicationStatus communication;
 SafetyTime received{};
};
struct AcceptedArmTarget {
 std::array<float,6> q{};
 SafetyTime accepted{};
 bool valid=false;
};
class RarsTransport {
 public:
 virtual ~RarsTransport()=default;
 virtual std::optional<RarsFrame> Read(SafetyTime now)=0;
 // The serial owner records a target ONLY after its control path accepted it.
 virtual std::optional<AcceptedArmTarget> LastAcceptedTarget() const=0;
 virtual const rars_arm::ArmConfiguration& Configuration() const=0;
};
class MockRarsTransport final : public RarsTransport {
 public:
 explicit MockRarsTransport(rars_arm::ArmConfiguration config):config_(std::move(config)){}
 std::deque<RarsFrame> frames;
 std::optional<AcceptedArmTarget> target;
 std::optional<RarsFrame> Read(SafetyTime) override {
  if(frames.empty())return std::nullopt;
  auto f=frames.front();frames.pop_front();return f;
 }
 std::optional<AcceptedArmTarget> LastAcceptedTarget() const override {return target;}
 const rars_arm::ArmConfiguration& Configuration() const override {return config_;}
 private:
 rars_arm::ArmConfiguration config_;
};
// An externally owned, already configured SDK instance is borrowed read-only.
// This adapter cannot open, enable, zero, or command the serial device.
class SerialOwnerLease;
class SdkRarsReadOnlyTransport final : public RarsTransport {
 public:
 SdkRarsReadOnlyTransport(rars_arm::RarsArm& owner, const SerialOwnerLease& lease);
 std::optional<RarsFrame> Read(SafetyTime now) override;
 std::optional<AcceptedArmTarget> LastAcceptedTarget() const override { return accepted_; }
 const rars_arm::ArmConfiguration& Configuration() const override { return owner_.configuration(); }
 // Called by the SAME serial owner, after successful acceptance, never by a
 // desired-target subscription. Rejected requests must not refresh this record.
 void RecordAcceptedTarget(const std::array<float,6>& q,SafetyTime accepted);
 private:
 rars_arm::RarsArm& owner_;
 const SerialOwnerLease& lease_;
 std::optional<AcceptedArmTarget> accepted_;
};
struct ArmSnapshot {
 ArmSnapshot() { q.fill(std::numeric_limits<float>::quiet_NaN()); dq.fill(std::numeric_limits<float>::quiet_NaN()); }
 std::array<float,6> q{},dq{};
 std::array<bool,6> valid{};
 SafetyTime received{};
 double feedback_age_ms=-1;
 bool feedback_ready=false,target_ready=false;
 bool per_joint_freshness_proven=false;
 bool zero_calibration_operator_verified=true;
 std::string zero_calibration_source="SDK_GUI_OPERATOR_SAVED";
 std::optional<AcceptedArmTarget> target;
 std::string rejection;
};
class RarsBridge {
 public:
 RarsBridge(RarsTransport& transport,double feedback_timeout_s=.25,double target_timeout_s=.25);
 const ArmSnapshot& Poll(SafetyTime now);
 const ArmSnapshot& snapshot() const {return snapshot_;}
 std::string CalibrationDiagnostic() const;
 private:
 RarsTransport& transport_;
 double feedback_timeout_s_,target_timeout_s_;
 std::optional<RarsFrame> frame_;
 ArmSnapshot snapshot_;
};
// Advisory flock per serial device. All SDK/GraspNet/helpers must share this
// lease; non-cooperating processes are outside this software guarantee.
// Acquiring the lease never opens the serial device.
class SerialOwnerLease {
 public:
 SerialOwnerLease(const std::string& lock_directory,const std::string& device);
 ~SerialOwnerLease();
 SerialOwnerLease(const SerialOwnerLease&)=delete;
 SerialOwnerLease& operator=(const SerialOwnerLease&)=delete;
 bool acquired() const {return fd_>=0;}
 private:
 int fd_=-1;
};
} // namespace sim2real
