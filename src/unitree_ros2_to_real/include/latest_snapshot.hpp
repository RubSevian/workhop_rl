#pragma once
#include <array>
#include <atomic>
#include <type_traits>

namespace sim2real {
// Exactly one producer and one serialized consumer. Each owns a slot; the
// third slot is exchanged atomically. Neither party accesses the other's slot.
// A plain seqlock on non-atomic fields would be a C++ data race; this is not one.
// Updates may be coalesced. Events must use a separate preserving channel.
template<class T>
class LatestSnapshot {
 public:
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::atomic<unsigned>::is_always_lock_free);
  void Publish(const T& value) noexcept {
    slots_[writer_] = value;
    writer_ = middle_.exchange(writer_ | dirty, std::memory_order_acq_rel) & index_mask;
  }
  bool Read(T& value) noexcept {
    if (!(middle_.load(std::memory_order_acquire) & dirty)) return false;
    reader_ = middle_.exchange(reader_, std::memory_order_acq_rel) & index_mask;
    value = slots_[reader_];
    return true;
  }
 private:
  static constexpr unsigned dirty = 4, index_mask = 3;
  std::array<T,3> slots_{};
  alignas(64) std::atomic<unsigned> middle_{1};
  alignas(64) unsigned writer_ = 2;
  alignas(64) unsigned reader_ = 0;
};
}
