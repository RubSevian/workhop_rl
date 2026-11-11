#pragma once
#include "latest_lowstate_channel.hpp"

namespace sim2real {
// Owned only by the control side, serialized by the node's control mutex.
// The receiver never touches the supervisor, decoder, or cached display status.
class LatestLowStateControl {
 public:
  explicit LatestLowStateControl(double remote_timeout) : display_(.75,remote_timeout),remote_timeout_(remote_timeout) {}
  void Apply(LatestLowStateChannel& channel,LowStateReader& reader,R3Inputs& inputs,
             R3Supervisor& supervisor,SafetyTime now) {
    LatestLowStateChannel::Sample sample;
    if(channel.Read(sample)){pending_=sample;have_pending_=true;}
    if(have_pending_ && pending_.state.received<=now) {
      current_=pending_;have_current_=true;have_pending_=false;
      reader.AssignSnapshot(current_.state);
      display_.Receive(current_.remote,current_.state.received);
      status_=display_.status();
      inputs.lowstate_stamp=inputs.remote_stamp=current_.state.received;
    }
    status_.remote_age_ms=have_current_?std::chrono::duration<double,std::milli>(now-current_.state.received).count():-1;
    status_.remote_valid=have_current_&&current_.remote_valid&&status_.remote_age_ms>=0&&status_.remote_age_ms<=remote_timeout_*1000;
    status_.takeover_hold_active=status_.remote_valid&&current_.hold_active;
    status_.takeover_request_latched=have_current_&&current_.request_latched;
    inputs.ready.remote_fresh=status_.remote_valid;
    inputs.ready.motor_state_valid=have_current_&&reader.snapshot().valid&&
        reader.snapshot().received.time_since_epoch().count()>channel.LatestRejectionTick();
    if(inputs.ready.motor_state_valid)inputs.measured_q=reader.snapshot().motor_q;
    LatestLowStateChannel::Rejection rejection;
    const bool rejected=channel.TakeRejection(rejection);
    if(rejected&&!fault_rejection_frozen_) {last_rejection_=rejection;have_rejection_=true;}
    if(supervisor.output_enabled() && (rejected || (have_current_&&!inputs.ready.motor_state_valid))) {
      supervisor.Fault("invalid_lowstate",now);
    }
    // A rejection barrier can precede publication of its diagnostic payload.
    // Freeze only after consuming that payload, never an older recovered error.
    if(supervisor.fault_latched()&&supervisor.last_fault()=="invalid_lowstate"&&rejected)
      fault_rejection_frozen_=true;
  }
  R3RemoteEvent TakeDecision(LatestLowStateChannel& channel,SafetyTime now) {
    return channel.TakeDecision(current_,now);
  }
  const RemoteStatus& status() const { return status_; }
  const LatestLowStateChannel::Rejection* rejection() const { return have_rejection_?&last_rejection_:nullptr; }
 private:
  RemoteSafety display_;
  double remote_timeout_;
  RemoteStatus status_{};
  LatestLowStateChannel::Sample current_{},pending_{};
  LatestLowStateChannel::Rejection last_rejection_{};
  bool have_current_=false,have_pending_=false,have_rejection_=false,fault_rejection_frozen_=false;
};
}
