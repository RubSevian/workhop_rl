#pragma once
#include <string_view>
namespace sim2real {
enum class OperationProfile { READ_ONLY, ARM_TEST, LEG_SAFETY_TEST, RL_ZERO_TEST, REMOTE_TEST, NAV_TEST, FULL_MISSION };
enum class CommandSource { NONE, REMOTE, NAVIGATION };
enum class TakeoverLimit { NONE, HOLD_CURRENT, RL };
struct OperationCapabilities {
 bool physical_leg_output=false,allow_sport_release=false,allow_rl=false,allow_nonzero_velocity=false;
 bool allow_arm_motion=false,allow_arm_home_lifecycle=false,allow_arm_emergency=false;
 bool require_arm_home_for_takeover=false,require_navigation_ready=false,allow_navigation_commands=false;
 bool require_emergency_arm_validation=false,require_perception_ready=false;
 CommandSource command_source=CommandSource::NONE;TakeoverLimit takeover_limit=TakeoverLimit::NONE;
};
OperationProfile ParseOperationProfile(std::string_view name);
const char* OperationProfileName(OperationProfile profile);
const char* CommandSourceName(CommandSource source);
OperationCapabilities ResolveOperationCapabilities(OperationProfile profile);
OperationProfile LegacyOperationProfile(bool read_only,std::string_view control_mode,bool motion_commands_enabled);
} // namespace sim2real
