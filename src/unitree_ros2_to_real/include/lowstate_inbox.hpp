#pragma once
#include "safety_io.hpp"
#include "r3_commissioning.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>

namespace sim2real {
// One producer: dedicated LowState callback group. Consumers must be serialized
// by the existing control mutex. Neither side waits for the other: release /
// acquire publication protects each slot until its consumer has finished copying.
// Preserve ALL accepted remote transitions and invalid samples, not just q/dq.
// Capacity is bounded; overflow latches until this process is restarted.
template<size_t Capacity = 64>
class LowStateInbox {
 public:
  static_assert(Capacity > 0);
  static_assert(std::atomic<size_t>::is_always_lock_free);
  struct Sample {
    LowStateReader state{.5};
    std::array<uint8_t,40> remote{};
    SafetyTime stamp{};
  };
  explicit LowStateInbox(double timeout) : reader_(timeout) {}
  bool Receive(const unitree_go::msg::LowState& msg, SafetyTime stamp) {
    const auto head = head_.load(std::memory_order_relaxed);
    if(head-tail_.load(std::memory_order_acquire) >= Capacity) {
      overflow_.store(true,std::memory_order_release);
      return false;
    }
    reader_.Receive(msg,stamp);
    auto& sample = slots_[head % Capacity];
    sample.state = reader_;
    std::copy(msg.wireless_remote.begin(),msg.wireless_remote.end(),sample.remote.begin());
    sample.stamp = stamp;
    head_.store(head+1,std::memory_order_release);
    return true;
  }
  bool Pop(Sample& sample, SafetyTime through) {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if(tail == head_.load(std::memory_order_acquire))return false;
    const auto& slot = slots_[tail % Capacity];
    // A concurrent producer may observe a frame AFTER the consumer's clock
    // sample. Leave that frame for the next pass instead of making it future.
    if(slot.stamp > through)return false;
    sample = slot;
    tail_.store(tail+1,std::memory_order_release);
    return true;
  }
  bool overflowed() const {return overflow_.load(std::memory_order_acquire);}
  static constexpr size_t capacity = Capacity;
 private:
  LowStateReader reader_;
  std::array<Sample,Capacity> slots_{};
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
  std::atomic<bool> overflow_{false};
};
// Consumer is invoked under the existing control mutex before input admission
// and before IO creates a packet. Keep cancellation / output ownership there.
template<size_t Capacity>
void ApplyLowStateInbox(LowStateInbox<Capacity>& inbox, LowStateReader& reader,
                       R3RemoteCommands& remote, R3Inputs& inputs,
                       R3Supervisor& supervisor, SafetyTime now) {
 typename LowStateInbox<Capacity>::Sample sample;
 for(size_t i=0;i<Capacity && inbox.Pop(sample,now);++i) {
  reader=sample.state;
  remote.ReceiveBuffered(sample.remote,sample.stamp);
  inputs.lowstate_stamp=sample.stamp;inputs.remote_stamp=sample.stamp;
  inputs.ready.motor_state_valid=reader.snapshot().valid;
  if(inputs.ready.motor_state_valid)inputs.measured_q=reader.snapshot().motor_q;
  else if(supervisor.output_enabled())supervisor.Fault("invalid_lowstate",now);
 }
 if(inbox.overflowed())supervisor.Fault("lowstate_inbox_overflow",now);
}
}
