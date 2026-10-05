#include "operation_profile.hpp"
#include <cassert>
#include <iostream>
#include <stdexcept>
using namespace sim2real;
int main(){for(int i=0;i<7;++i){auto p=static_cast<OperationProfile>(i);assert(ParseOperationProfile(OperationProfileName(p))==p);auto c=ResolveOperationCapabilities(p);
 assert(c.physical_leg_output==(i>=2));assert(c.allow_rl==(i>=3));assert(c.allow_nonzero_velocity==(i>=4));assert(c.allow_navigation_commands==(i>=5));
 assert(c.command_source==(i<4?CommandSource::NONE:i==4?CommandSource::REMOTE:CommandSource::NAVIGATION));
 assert(c.require_emergency_arm_validation==(i==6));assert(c.allow_arm_motion==(i==1||i==6));}
 for(auto s:{"","unknown","RL_ZERO_TEST","remote"}){bool rejected=false;try{ParseOperationProfile(s);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);}
 assert(LegacyOperationProfile(true,"remote_test",true)==OperationProfile::READ_ONLY);
 assert(LegacyOperationProfile(false,"autonomy",false)==OperationProfile::RL_ZERO_TEST);
 assert(LegacyOperationProfile(false,"autonomy",true)==OperationProfile::NAV_TEST);
 auto hold=ResolveOperationCapabilities(OperationProfile::LEG_SAFETY_TEST);assert(hold.takeover_limit==TakeoverLimit::HOLD_CURRENT&&!hold.allow_rl);
 std::cout<<"PASS seven strict profiles/capabilities and legacy launch mapping; no IO\n";}
