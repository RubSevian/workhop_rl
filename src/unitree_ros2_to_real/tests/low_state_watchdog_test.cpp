#include "low_state_watchdog.hpp"
#include <stdexcept>
int main() {
  using namespace std::chrono_literals;
  LowStateWatchdog guard;
  const auto t = LowStateWatchdog::Clock::now();
  const auto expect = [](bool value) { if (!value) throw std::runtime_error("watchdog regression"); };
  expect(!guard.CanRefresh(t));
  guard.Observe(1, t);
  expect(!guard.CanRefresh(t));
  guard.PolicyUpdated(t);
  expect(guard.CanRefresh(t + 20ms));
  guard.Observe(1, t + 40ms); // Replayed duplicate must not extend liveness.
  expect(!guard.CanRefresh(t + 51ms));
  guard.Observe(2, t + 110ms);
  expect(!guard.CanRefresh(t + 110ms)); // State alone cannot revive stale leg actions.
  guard.PolicyUpdated(t + 110ms);
  expect(guard.CanRefresh(t + 110ms));
}
