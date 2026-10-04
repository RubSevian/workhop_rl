#include "go2_motion_mode.hpp"
#include "output_lease.hpp"
#include <memory>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void PrintUsage(const char* executable) {
  std::cout
      << "Usage: " << executable << " --interface IFACE [--status | --release-sport-mode | --enable-sport-mode] [--machine-readable]\n"
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
  bool release = false,enable = false,confirmed_stop = false;
  int operations=0;
  std::string output_lock;
  bool machine_readable = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--interface" && index + 1 < argc) {
      network_interface = argv[++index];
    } else if(argument=="--output-lock" && index+1<argc) {
      output_lock=argv[++index];
    } else if(argument=="--confirm-output-stopped") {
      confirmed_stop=true;
    } else if(argument=="--enable-sport-mode") {
      enable=true;++operations;
    } else if (argument == "--machine-readable") {
      machine_readable = true;
    } else if (argument == "--status") {
      release = false;++operations;
    } else if (argument == "--release-sport-mode") {
      release = true;++operations;
    } else if (argument == "--help" || argument == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown or incomplete argument: " << argument << "\n";
      PrintUsage(argv[0]);
      return 2;
    }
  }

  if(operations>1) {std::cerr<<"Choose exactly one SDK operation\n";return 2;}
  std::unique_ptr<sim2real::OutputLease> lease;
  if(enable) {
    if(!confirmed_stop||output_lock.empty()){std::cerr<<"Enable requires --confirm-output-stopped --output-lock PATH after publisher stop proof\n";return 2;}
    lease=std::make_unique<sim2real::OutputLease>(output_lock);
    if(!lease->acquired()){std::cerr<<"LowCmd output lease busy/unavailable; Sport enable forbidden\n";return 2;}
  }
  const go2_motion_mode::Result result =
      enable ? go2_motion_mode::EnableSportMode(network_interface) : release ? go2_motion_mode::ReleaseSportMode(network_interface)
              : go2_motion_mode::QuerySportMode(network_interface);
  if(machine_readable) {
    std::cout << sim2real::SportModeName(result.state) << "\n";
    std::cerr << result.message << "\n";
  } else std::cout << (result.ok ? "OK: " : "ERROR: ") << result.message << "\n";

  // Query is deliberately successful even while sport_mode is active: it is a
  // diagnostic command.  Release succeeds only after verified deactivation.
  return result.ok ? 0 : 1;
}
