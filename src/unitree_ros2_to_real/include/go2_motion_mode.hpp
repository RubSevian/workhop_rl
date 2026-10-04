#pragma once

#include <string>
#include "sport_mode_status.hpp"

// This helper intentionally uses only the Unitree SDK2 robot_state service.
// It never publishes LowCmd. Release changes robot ownership; commissioning only.
namespace go2_motion_mode {

struct Result {
  bool ok{false};
  bool sport_mode_active{false};
  std::string message;
  sim2real::SportMode state{sim2real::SportMode::UNKNOWN};
};

// Query the controller's service table.  A successful result with
// state == RELEASED proves only that service observation, not actuator readiness.
Result QuerySportMode(const std::string& network_interface);

// Disable the Unitree sport_mode service when it is active, then verify the
// state by querying the service table again.  No motor command is sent.
Result ReleaseSportMode(const std::string& network_interface);

// Caller must hold the same output lease as the LowCmd owner, after stop confirmation.
Result EnableSportMode(const std::string& network_interface);

}  // namespace go2_motion_mode
