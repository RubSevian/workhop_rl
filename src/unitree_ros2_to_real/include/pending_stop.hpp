#pragma once
#include <atomic>

namespace sim2real {
enum class PendingStop { NONE, ABORT, EMERGENCY };
// Separate from latest-state storage: a neutral/new state cannot erase a stop.
// Concurrent Publish/Take linearize at the atomic RMW. A stop belongs either
// to this Take or the next one. Both flags together consume as EMERGENCY.
class PendingStopLatch {
 public:
  static_assert(std::atomic<unsigned>::is_always_lock_free);
  void Publish(PendingStop stop) noexcept {
    const unsigned mask = stop==PendingStop::EMERGENCY ? 2 : stop==PendingStop::ABORT ? 1 : 0;
    if(mask) bits_.fetch_or(mask,std::memory_order_release);
  }
  PendingStop Take() noexcept {
    const auto bits=bits_.exchange(0,std::memory_order_acq_rel);
    return bits&2 ? PendingStop::EMERGENCY : bits&1 ? PendingStop::ABORT : PendingStop::NONE;
  }
 private:
  std::atomic<unsigned> bits_{0};
};
}
