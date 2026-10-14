#pragma once
#include <cstdlib>
#include <string>
#include <stdexcept>
namespace sim2real {
inline constexpr const char* ReplayDDS="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" /></Interfaces></General></Domain></CycloneDDS>";
inline void RequireIsolatedReplay() {
  auto env=[](const char* key){const auto* s=std::getenv(key);return std::string(s?s:"");};
  if(env("ROS_DOMAIN_ID")!="228"||env("ROS_LOCALHOST_ONLY")!="1"||
     env("CYCLONEDDS_URI")!=ReplayDDS||!env("GO2_NETWORK_INTERFACE").empty()||
     !env("ROS_STATIC_PEERS").empty())
    throw std::runtime_error("Replay requires domain228, localhost, canonical loopback DDS and no SDK/peers");
}
}
