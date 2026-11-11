#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>

namespace sim2real {
// Lifetime maxima since process start. One serialized receiver writer and one
// serialized IO writer; status may read concurrently. No allocation or locks.
// These are local receipt times, not end-to-end sensor measurement latency.
class LowStateTiming {
 public:
  using Time = std::chrono::steady_clock::time_point;
  static_assert(std::atomic<int64_t>::is_always_lock_free);
  void Receive(Time now) {
    if(previous_receive_!=Time{} && now>=previous_receive_)
      StoreMaximum(receive_gap_ns_,Ns(now-previous_receive_));
    previous_receive_=now;
  }
  void Use(Time received,Time now) {
    if(received!=Time{} && now>=received)
      StoreMaximum(io_age_ns_,Ns(now-received));
  }
  double ReceiveGapMaxMs() const {return receive_gap_ns_.load(std::memory_order_relaxed)/1e6;}
  double IoAgeMaxMs() const {return io_age_ns_.load(std::memory_order_relaxed)/1e6;}
 private:
  static int64_t Ns(Time::duration duration) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }
  static void StoreMaximum(std::atomic<int64_t>& maximum,int64_t value) {
    // Each maximum has exactly one writer; no CAS retry on the hot path.
    if(value>maximum.load(std::memory_order_relaxed))maximum.store(value,std::memory_order_relaxed);
  }
  Time previous_receive_{};
  std::atomic<int64_t> receive_gap_ns_{0},io_age_ns_{0};
};
}
