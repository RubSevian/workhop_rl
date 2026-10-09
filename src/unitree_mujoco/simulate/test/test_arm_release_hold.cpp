#include "arm_release_hold.hpp"
#include <stdexcept>
int main() {
  rars01_sim::ArmReleaseHold hold;
  std::array<double, 8> q{1, 2, 3, 4, 5, 6, .02, .02};
  const auto expect = [](bool value) { if (!value) throw std::runtime_error("release hold regression"); };
  expect(!hold.Capture(q));
  hold.CommandAccepted();
  expect(hold.Capture(q));
  q[0] = 2;
  expect(hold.Capture(q) && hold.target()[0] == 1); // No following drift.
  hold.CommandAccepted();
  expect(hold.Capture(q) && hold.target()[0] == 2); // New motion, new release pose.
  hold.Reset();
  expect(!hold.Capture(q));
}
