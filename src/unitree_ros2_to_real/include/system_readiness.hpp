#pragma once
#include "operation_profile.hpp"
#include <string>
#include <vector>
namespace sim2real {
// Facts, not state: supplied by measured/status adapters; never grants ownership.
struct SystemReadiness {
 bool arm_control_ready=false,arm_home_ready=false;
 bool navigation_ready=false,perception_ready=false,arm_emergency_validated=false;
 bool own_output_healthy=false;
 std::vector<std::string> TakeoverBlockers(const OperationCapabilities& capabilities) const;
 std::vector<std::string> RuntimeBlockers() const;
};
} // namespace sim2real
