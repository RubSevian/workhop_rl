#pragma once

#include <string>

// This helper intentionally uses only the Unitree SDK2 robot_state service.
// It never publishes LowCmd and is safe to run before the locomotion process.
namespace go2_motion_mode {

struct Result {
  bool ok{false};
  bool sport_mode_active{false};
  std::string message;
};

// Query the controller's service table.  A successful result with
// sport_mode_active == false means that it is safe to claim rt/lowcmd.
Result QuerySportMode(const std::string& network_interface);

// Disable the Unitree sport_mode service when it is active, then verify the
// state by querying the service table again.  No motor command is sent.
Result ReleaseSportMode(const std::string& network_interface);

}  // namespace go2_motion_mode
