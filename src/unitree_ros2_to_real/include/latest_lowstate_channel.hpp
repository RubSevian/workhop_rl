#pragma once
#include "latest_snapshot.hpp"
#include "pending_stop.hpp"
#include "first_pending_event.hpp"
#include "r3_commissioning.hpp"
#include <algorithm>
#include <limits>

namespace sim2real {
// Latest state plus preserving decisions; no queue of stale input frames.
// Receive belongs exclusively to the receiver thread. Read/TakeDecision/
// TakeRejection belong exclusively to the serialized control thread.
class LatestLowStateChannel {
 public:
  struct Sample {
    LowStateSnapshot state{};
    std::array<uint8_t,40> remote{};
    std::uint64_t takeover_id=0;
    SafetyTime takeover_time{};
    bool remote_valid=false, takeover_held=false, hold_active=false, request_latched=false;
  };
  struct Rejection { LowStateRejection reason=LowStateRejection::NONE; SafetyTime first_time{}; };
  LatestLowStateChannel(double timeout, std::vector<std::string> abort,
                       std::vector<std::string> emergency, double hold=.75,double stale=.25)
      :reader_(timeout),remote_(std::move(abort),std::move(emergency),hold,stale),stale_s_(stale) {}
  void Receive(const unitree_go::msg::LowState& message,SafetyTime received) {
    reader_.Receive(message,received);
    if(!reader_.snapshot().valid) {
      // Admission observes this barrier even if the snapshot/event publication
      // is still in flight. A good frame must be strictly newer to unblock it.
      latest_rejection_tick_.store(received.time_since_epoch().count(),std::memory_order_release);
      rejections_.Publish({reader_.snapshot().rejection,received});
    }
    remote_.Receive(message.wireless_remote,received);
    const auto decision=remote_.Poll(received);
    if(decision==R3RemoteEvent::EMERGENCY || decision==R3RemoteEvent::CONTROLLED_ABORT) {
      cancelled_id_.store(next_id_,std::memory_order_release);
      pending_id_=0;
      stops_.Publish(decision==R3RemoteEvent::EMERGENCY?PendingStop::EMERGENCY:PendingStop::ABORT);
    }
    const auto& status=remote_.status();
    const bool held=status.remote_valid && (status.button_mask & takeover_button_mask)==takeover_button_mask;
    if(!held) {
      cancelled_id_.store(next_id_,std::memory_order_release);
      pending_id_=0;
    }
    if(decision==R3RemoteEvent::TAKEOVER){pending_id_=++next_id_;request_time_=received;}
    Sample sample;
    sample.state=reader_.snapshot();
    std::copy(message.wireless_remote.begin(),message.wireless_remote.end(),sample.remote.begin());
    sample.takeover_id=pending_id_;sample.takeover_time=request_time_;
    sample.remote_valid=status.remote_valid;sample.takeover_held=held;
    sample.hold_active=status.takeover_hold_active;sample.request_latched=status.takeover_request_latched;
    latest_.Publish(sample);
  }
  std::int64_t LatestRejectionTick() const { return latest_rejection_tick_.load(std::memory_order_acquire); }
  bool Read(Sample& sample) { return latest_.Read(sample); }
  bool TakeRejection(Rejection& rejection) { return rejections_.Take(rejection); }
  // The sample must be the most recently read one. A stop consumes concurrent
  // takeover intent. A request's own age expires even if held frames continue.
  R3RemoteEvent TakeDecision(const Sample& sample,SafetyTime now) {
    const auto stop=stops_.Take();
    if(stop!=PendingStop::NONE) {
      consumed_id_=std::max(consumed_id_,sample.takeover_id);
      return stop==PendingStop::EMERGENCY?R3RemoteEvent::EMERGENCY:R3RemoteEvent::CONTROLLED_ABORT;
    }
    if(sample.takeover_id==0 || sample.takeover_id<=consumed_id_)return R3RemoteEvent::NONE;
    if(now<sample.state.received || now<sample.takeover_time)return R3RemoteEvent::NONE;
    consumed_id_=sample.takeover_id;
    if(sample.takeover_id<=cancelled_id_.load(std::memory_order_acquire))return R3RemoteEvent::NONE;
    const auto fresh=[&](SafetyTime stamp){const double age=std::chrono::duration<double>(now-stamp).count();return age>=0&&age<=stale_s_;};
    return sample.remote_valid&&sample.takeover_held&&fresh(sample.state.received)&&fresh(sample.takeover_time)
        ? R3RemoteEvent::TAKEOVER : R3RemoteEvent::NONE;
  }
 private:
  LowStateReader reader_;
  R3RemoteCommands remote_;
  double stale_s_;
  std::uint64_t pending_id_=0,next_id_=0,consumed_id_=0;
  std::atomic<std::int64_t> latest_rejection_tick_{std::numeric_limits<std::int64_t>::min()};
  static_assert(std::atomic<std::int64_t>::is_always_lock_free);
  std::atomic<std::uint64_t> cancelled_id_{0};
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  SafetyTime request_time_{};
  LatestSnapshot<Sample> latest_;
  PendingStopLatch stops_;
  FirstPendingEvent<Rejection> rejections_;
};
}
