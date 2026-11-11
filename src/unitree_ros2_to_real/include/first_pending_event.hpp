#pragma once
#include <atomic>
#include <type_traits>
namespace sim2real {
// SPSC preserving latch. Coalesce repeated events until the first is taken.
// Release/acquire transfers payload ownership in both directions.
template<class T>
class FirstPendingEvent {
 public:
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::atomic<bool>::is_always_lock_free);
  bool Publish(const T& value) noexcept {
    if(pending_.load(std::memory_order_acquire))return false;
    value_=value;
    pending_.store(true,std::memory_order_release);
    return true;
  }
  bool Take(T& value) noexcept {
    if(!pending_.load(std::memory_order_acquire))return false;
    value=value_;
    pending_.store(false,std::memory_order_release);
    return true;
  }
 private:
  T value_{};
  std::atomic<bool> pending_{false};
};
}
