#pragma once

#include <array>
#include <chrono>
#include <mutex>

// Keeps navigation commands in physical units until Agent applies its normal
// command scaling.  This class deliberately has no ROS dependency so its
// safety behaviour can be tested without a running ROS graph.
class NavigationCommandAdapter {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct SafeCommand {
    std::array<float, 3> value{{0.0F, 0.0F, 0.0F}};  // vx [m/s], vy [m/s], wz [rad/s]
    bool navigation_active = false;
    bool has_valid_command = false;
    bool stale = true;
    double age_sec = 0.0;
  };

  explicit NavigationCommandAdapter(double timeout_sec = 0.5);

  void SetTimeout(double timeout_sec);
  void SetNavigationActive(bool active);

  // Returns false for a non-finite packet.  In that case neither the stored
  // command nor its receive timestamp changes.
  bool Accept(float vx, float vy, float wz, TimePoint receive_time);
  SafeCommand GetSafeCommand(TimePoint now) const;

private:
  static float Clamp(float value, float lower, float upper);

  mutable std::mutex mutex_;
  std::array<float, 3> raw_command_{{0.0F, 0.0F, 0.0F}};
  bool navigation_active_ = false;
  bool has_valid_command_ = false;
  TimePoint last_valid_receive_time_{};
  double timeout_sec_ = 0.5;
};
