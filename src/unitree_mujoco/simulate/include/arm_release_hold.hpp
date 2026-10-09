#pragma once
#include <array>

namespace rars01_sim {
// After a valid manual command, latch the measured pose on release. Never
// generate an unvalidated path back to home. Startup/reset still uses home.
class ArmReleaseHold {
 public:
  void Reset() { armed_ = holding_ = false; }
  void CommandAccepted() { armed_ = true; holding_ = false; }
  bool Capture(const std::array<double, 8>& measured) {
    if (!armed_) return false;
    if (!holding_) { target_ = measured; holding_ = true; }
    return true;
  }
  const std::array<double, 8>& target() const { return target_; }
  bool holding() const { return holding_; }
 private:
  bool armed_ = false, holding_ = false;
  std::array<double, 8> target_{};
};
}  // namespace rars01_sim
