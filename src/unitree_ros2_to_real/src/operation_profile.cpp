#include "operation_profile.hpp"
#include <array>
#include <stdexcept>
namespace sim2real {
namespace {constexpr std::array<const char*,7> names{"read_only","arm_test","leg_safety_test","rl_zero_test","remote_test","nav_test","full_mission"};}
const char* OperationProfileName(OperationProfile p){auto i=static_cast<unsigned>(p);if(i>=names.size())throw std::invalid_argument("Invalid OperationProfile enum");return names[i];}
OperationProfile ParseOperationProfile(std::string_view s){for(unsigned i=0;i<names.size();++i)if(s==names[i])return static_cast<OperationProfile>(i);throw std::invalid_argument("Unknown operation_profile: "+std::string(s));}
const char* CommandSourceName(CommandSource s){switch(s){case CommandSource::NONE:return "NONE";case CommandSource::REMOTE:return "REMOTE";case CommandSource::NAVIGATION:return "NAVIGATION";}throw std::invalid_argument("Invalid command source");}
OperationCapabilities ResolveOperationCapabilities(OperationProfile p){OperationProfileName(p);OperationCapabilities c;
 if(p==OperationProfile::READ_ONLY)return c;
 c.allow_arm_home_lifecycle=c.allow_arm_emergency=true;
 if(p==OperationProfile::ARM_TEST){c.allow_arm_motion=true;return c;}
 c.physical_leg_output=c.allow_sport_release=c.require_arm_home_for_takeover=true;c.takeover_limit=TakeoverLimit::HOLD_CURRENT;
 if(p==OperationProfile::LEG_SAFETY_TEST)return c;
 c.allow_rl=true;c.takeover_limit=TakeoverLimit::RL;
 if(p==OperationProfile::RL_ZERO_TEST)return c;
 c.allow_nonzero_velocity=true;
 if(p==OperationProfile::REMOTE_TEST){c.command_source=CommandSource::REMOTE;return c;}
 c.command_source=CommandSource::NAVIGATION;c.allow_navigation_commands=c.require_navigation_ready=true;
 if(p==OperationProfile::FULL_MISSION)c.allow_arm_motion=c.require_emergency_arm_validation=c.require_perception_ready=true;
 return c;
}
OperationProfile LegacyOperationProfile(bool ro,std::string_view mode,bool motion){if(mode!="remote_test"&&mode!="autonomy")throw std::invalid_argument("control_mode must be remote_test or autonomy");if(ro)return OperationProfile::READ_ONLY;if(!motion)return OperationProfile::RL_ZERO_TEST;return mode=="autonomy"?OperationProfile::NAV_TEST:OperationProfile::REMOTE_TEST;}
}
