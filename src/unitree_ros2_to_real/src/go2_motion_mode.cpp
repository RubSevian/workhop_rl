#include "go2_motion_mode.hpp"

#include <chrono>
#include <sstream>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace go2_motion_mode {
namespace {

constexpr char kSportModeService[] = "sport_mode";

Result QuerySportModeWithClient(unitree::robot::go2::RobotStateClient& client) {
  std::vector<unitree::robot::go2::ServiceState> services;
  const int32_t ret = client.ServiceList(services);
  if (ret != 0) {
    return {false, false, "ServiceList failed with SDK2 return code " + std::to_string(ret)};
  }

  for (const auto& service : services) {
    if (service.name == kSportModeService) {
      std::ostringstream stream;
      stream << "sport_mode status=" << service.status
             << " protect=" << service.protect;
      return {true, service.status != 0, stream.str()};
    }
  }

  return {false, false, "sport_mode is absent from the RobotState service list"};
}

template <typename Operation>
Result WithRobotStateClient(const std::string& network_interface, Operation operation) {
  if (network_interface.empty()) {
    return {false, false,
            "network interface is empty; pass --interface IFACE or set GO2_NETWORK_INTERFACE"};
  }

  auto* factory = unitree::robot::ChannelFactory::Instance();
  factory->Init(0, network_interface);
  try {
    unitree::robot::go2::RobotStateClient client;
    client.SetTimeout(5.0F);
    client.Init();
    Result result = operation(client);
    factory->Release();
    return result;
  } catch (const std::exception& error) {
    factory->Release();
    return {false, false, std::string("SDK2 exception: ") + error.what()};
  }
}

}  // namespace

Result QuerySportMode(const std::string& network_interface) {
  return WithRobotStateClient(network_interface, [](auto& client) {
    return QuerySportModeWithClient(client);
  });
}

Result ReleaseSportMode(const std::string& network_interface) {
  return WithRobotStateClient(network_interface, [](auto& client) {
    Result before = QuerySportModeWithClient(client);
    if (!before.ok || !before.sport_mode_active) {
      return before;
    }

    int32_t status = -1;
    const int32_t ret = client.ServiceSwitch(kSportModeService, 0, status);
    if (ret != 0) {
      return Result{false, true,
                    "ServiceSwitch(sport_mode, 0) failed with SDK2 return code " +
                        std::to_string(ret) + ", status=" + std::to_string(status)};
    }

    // The service manager applies the change asynchronously on some firmware.
    for (int attempt = 0; attempt != 15; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      Result after = QuerySportModeWithClient(client);
      if (!after.ok) {
        return after;
      }
      if (!after.sport_mode_active) {
        after.message = "sport_mode released; " + after.message;
        return after;
      }
    }

    return Result{false, true, "sport_mode remained active after ServiceSwitch(sport_mode, 0)"};
  });
}

}  // namespace go2_motion_mode
