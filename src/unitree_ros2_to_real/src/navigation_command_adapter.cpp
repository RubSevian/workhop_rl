#include "navigation_command_adapter.hpp"

#include <algorithm>
#include <cmath>

NavigationCommandAdapter::NavigationCommandAdapter(double timeout_sec) {
  SetTimeout(timeout_sec);
}

void NavigationCommandAdapter::SetTimeout(double timeout_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_sec_ = (std::isfinite(timeout_sec) && timeout_sec > 0.0) ? timeout_sec : 0.5;
}

void NavigationCommandAdapter::SetNavigationActive(bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  navigation_active_ = active;
}

bool NavigationCommandAdapter::Accept(float vx, float vy, float wz, TimePoint receive_time) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) return false;

  std::lock_guard<std::mutex> lock(mutex_);
  raw_command_ = {{Clamp(vx, -1.0F, 1.0F), Clamp(vy, -0.5F, 0.5F), Clamp(wz, -0.5F, 0.5F)}};
  last_valid_receive_time_ = receive_time;
  has_valid_command_ = true;
  return true;
}

NavigationCommandAdapter::SafeCommand NavigationCommandAdapter::GetSafeCommand(TimePoint now) const {
  std::lock_guard<std::mutex> lock(mutex_);
  SafeCommand result;
  result.navigation_active = navigation_active_;
  result.has_valid_command = has_valid_command_;
  if (has_valid_command_) {
    result.age_sec = std::max(0.0, std::chrono::duration<double>(now - last_valid_receive_time_).count());
  }
  result.stale = !has_valid_command_ || result.age_sec > timeout_sec_;
  if (result.navigation_active && !result.stale) result.value = raw_command_;
  return result;
}

float NavigationCommandAdapter::Clamp(float value, float lower, float upper) {
  return std::max(lower, std::min(upper, value));
}
