#include "system_readiness.hpp"
namespace sim2real {
std::vector<std::string> SystemReadiness::TakeoverBlockers(const OperationCapabilities& c) const {
 std::vector<std::string> b;
 if(!arm_control_ready)b.emplace_back("arm_control_not_ready");
 if(c.require_arm_home_for_takeover&&!arm_home_ready)b.emplace_back("arm_home_not_ready");
 if(c.require_navigation_ready&&!navigation_ready)b.emplace_back("navigation_not_ready");
 if(c.require_perception_ready&&!perception_ready)b.emplace_back("perception_not_ready");
 if(c.require_emergency_arm_validation&&!arm_emergency_validated)b.emplace_back("arm_emergency_not_validated");
 return b;
}
std::vector<std::string> SystemReadiness::RuntimeBlockers() const {
 return arm_control_ready?std::vector<std::string>{}:std::vector<std::string>{"arm_control_not_ready"};
}
} // namespace sim2real
