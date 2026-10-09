#pragma once
#include <chrono>
#include <cstdint>

// Wall-time liveness, independent of /clock (which can stop with the simulator).
class LowStateWatchdog {
 public:
  using Clock = std::chrono::steady_clock;
  void Observe(uint32_t tick, Clock::time_point now) {
    if (!have_state_ || tick != tick_) {
      state_time_ = now;
      tick_ = tick;
      have_state_ = true;
    }
  }
  void PolicyUpdated(Clock::time_point now) { policy_time_ = now; have_policy_ = true; }
  bool CanRefresh(Clock::time_point now) const {
    return have_state_ && have_policy_ &&
        now - state_time_ <= std::chrono::milliseconds(50) &&
        now - policy_time_ <= std::chrono::milliseconds(100);
  }
 private:
  bool have_state_ = false, have_policy_ = false;
  uint32_t tick_ = 0;
  Clock::time_point state_time_{}, policy_time_{};
};
