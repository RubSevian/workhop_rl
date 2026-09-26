#include "navigation_command_adapter.hpp"

#include <cassert>
#include <cmath>
#include <limits>

int main() {
  using Adapter = NavigationCommandAdapter;
  const auto t0 = Adapter::Clock::now();
  Adapter adapter(0.5);
  const std::array<float, 3> zero{{0.0F, 0.0F, 0.0F}};
  const std::array<float, 3> clamped{{1.0F, -0.5F, 0.5F}};

  // A valid but inactive command is held internally and remains gated to zero.
  assert(adapter.Accept(4.0F, -3.0F, 2.0F, t0));
  auto command = adapter.GetSafeCommand(t0);
  assert(command.value == zero);
  adapter.SetNavigationActive(true);
  command = adapter.GetSafeCommand(t0);
  assert(command.value == clamped);

  // Watchdog always wins over the last accepted command.
  command = adapter.GetSafeCommand(t0 + std::chrono::milliseconds(501));
  assert(command.stale);
  assert(command.value == zero);

  // NaN must neither refresh the watchdog nor replace the prior safe value.
  assert(adapter.Accept(0.25F, 0.1F, -0.2F, t0));
  assert(!adapter.Accept(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F,
                         t0 + std::chrono::milliseconds(400)));
  command = adapter.GetSafeCommand(t0 + std::chrono::milliseconds(501));
  assert(command.stale);

  adapter.SetNavigationActive(false);
  assert(adapter.Accept(0.25F, 0.1F, -0.2F, t0 + std::chrono::seconds(1)));
  command = adapter.GetSafeCommand(t0 + std::chrono::seconds(1));
  assert(!command.stale && !command.navigation_active);
  assert(command.value == zero);
  return 0;
}
