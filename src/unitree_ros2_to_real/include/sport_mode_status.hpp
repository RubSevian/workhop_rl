#pragma once
#include <cstdint>
#include <string_view>
namespace sim2real {
enum class SportMode { ACTIVE, RELEASED, UNKNOWN, ERROR };
inline SportMode InterpretSportServiceStatus(int32_t status) {
 return status==0?SportMode::ACTIVE:status==1?SportMode::RELEASED:SportMode::UNKNOWN;
}
inline const char* SportModeName(SportMode mode) {
 switch(mode) {
 case SportMode::ACTIVE:return "SPORT_ACTIVE";
 case SportMode::RELEASED:return "SPORT_RELEASED";
 case SportMode::ERROR:return "SPORT_ERROR";
 default:return "SPORT_UNKNOWN";
 }
}
inline SportMode ParseSportMode(std::string_view token) {
 if(token=="SPORT_ACTIVE")return SportMode::ACTIVE;
 if(token=="SPORT_RELEASED")return SportMode::RELEASED;
 if(token=="SPORT_ERROR")return SportMode::ERROR;
 return SportMode::UNKNOWN;
}
} // namespace sim2real
