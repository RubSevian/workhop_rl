#include "go2_motion_mode.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void PrintUsage(const char* executable) {
  std::cout
      << "Usage: " << executable << " --interface IFACE [--status | --release-sport-mode]\n"
      << "\n"
      << "Queries or releases Unitree Go2 sport_mode through SDK2 RobotState.\n"
      << "It never publishes any motor command.\n"
      << "\n"
      << "Examples:\n"
      << "  " << executable << " --interface eth0 --status\n"
      << "  " << executable << " --interface eth0 --release-sport-mode\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string network_interface;
  if (const char* from_environment = std::getenv("GO2_NETWORK_INTERFACE")) {
    network_interface = from_environment;
  }
  bool release = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--interface" && index + 1 < argc) {
      network_interface = argv[++index];
    } else if (argument == "--status") {
      release = false;
    } else if (argument == "--release-sport-mode") {
      release = true;
    } else if (argument == "--help" || argument == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown or incomplete argument: " << argument << "\n";
      PrintUsage(argv[0]);
      return 2;
    }
  }

  const go2_motion_mode::Result result =
      release ? go2_motion_mode::ReleaseSportMode(network_interface)
              : go2_motion_mode::QuerySportMode(network_interface);
  std::cout << (result.ok ? "OK: " : "ERROR: ") << result.message << "\n";

  // Query is deliberately successful even while sport_mode is active: it is a
  // diagnostic command.  Release succeeds only after verified deactivation.
  return result.ok ? 0 : 1;
}
