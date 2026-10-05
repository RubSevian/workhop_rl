#include "system_readiness.hpp"
#include <cassert>
#include <algorithm>
using namespace sim2real;
int main(){
 SystemReadiness r;auto remote=ResolveOperationCapabilities(OperationProfile::REMOTE_TEST);
 assert(r.TakeoverBlockers(remote).size()==2);r.arm_control_ready=true;
 assert(r.RuntimeBlockers().empty()&&r.TakeoverBlockers(remote).size()==1);
 r.arm_home_ready=true;assert(r.TakeoverBlockers(remote).empty());
 r.arm_home_ready=false;assert(r.RuntimeBlockers().empty()); // active manipulation is permitted
 r.arm_control_ready=false;assert(!r.RuntimeBlockers().empty());
 r.arm_control_ready=r.arm_home_ready=true;
 auto full=ResolveOperationCapabilities(OperationProfile::FULL_MISSION);
 assert(r.TakeoverBlockers(full).size()==3);
 r.navigation_ready=r.perception_ready=r.arm_emergency_validated=true;
 assert(r.TakeoverBlockers(full).empty());
 auto nav=ResolveOperationCapabilities(OperationProfile::NAV_TEST);r.navigation_ready=false;
 assert(r.TakeoverBlockers(nav)==std::vector<std::string>{"navigation_not_ready"});
}
