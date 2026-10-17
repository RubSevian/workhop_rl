#include "go2_motion_mode.hpp"

#include <chrono>
#include <sstream>
#include <thread>
#include <vector>
#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace go2_motion_mode {
namespace {

constexpr char kSportModeService[] = "sport_mode";
// Go2 RobotState reports 0 while the service is running and 1 after it has
// been stopped.  This was verified against the Unitree mobile application on
// the physical robot; do not treat this as a generic C++ boolean.
constexpr int32_t kServiceRunning = 0;
constexpr int32_t kServiceStopped = 1;

Result QuerySportModeWithClient(unitree::robot::go2::RobotStateClient& client) {
  std::vector<unitree::robot::go2::ServiceState> services;
  const int32_t ret = client.ServiceList(services);
  if (ret != 0) {
    return {false, false, "ServiceList failed with SDK2 return code " + std::to_string(ret), sim2real::SportMode::ERROR};
  }

  for (const auto& service : services) {
    if (service.name == kSportModeService) {
      std::ostringstream stream;
      stream << "sport_mode status=" << service.status
             << " protect=" << service.protect;
      if (service.status == kServiceRunning) {
        return {true, true, stream.str() + " (Sport Mode active)", sim2real::SportMode::ACTIVE};
      }
      if (service.status == kServiceStopped) {
        return {true, false, stream.str() + " (Sport Mode released)", sim2real::SportMode::RELEASED};
      }
      return {false, false, stream.str() + " (unknown sport_mode state)"};
    }
  }

  return {false, false, "sport_mode is absent from the RobotState service list"};
}

template <typename Operation>
Result WithRobotStateClient(const std::string& network_interface, Operation operation) {
  if (network_interface.empty()) {
    return {false, false,
            "network interface is empty; pass --interface IFACE or set GO2_NETWORK_INTERFACE", sim2real::SportMode::ERROR};
  }

  auto* factory = unitree::robot::ChannelFactory::Instance();
  bool initialized=false;
  try {
    factory->Init(0, network_interface);initialized=true;
    Result result;
    {
      unitree::robot::go2::RobotStateClient client;
      client.SetTimeout(5.0F);
      client.Init();
      result=operation(client);
    } // DDS clients must be destroyed before releasing their factory.
    factory->Release();
    return result;
  } catch (const std::exception& error) {
    if(initialized)factory->Release();
    return {false, false, std::string("SDK2 exception: ") + error.what(), sim2real::SportMode::ERROR};
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
                        std::to_string(ret) + ", status=" + std::to_string(status), sim2real::SportMode::ERROR};
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

    return Result{false, true, "sport_mode remained active after ServiceSwitch(sport_mode, 0)", sim2real::SportMode::ACTIVE};
  });
}

Result EnableSportMode(const std::string& network_interface) {
  // ServiceSwitch(name,1,status) is documented by the vendored SDK's
  // example/go2/go2_robot_state_client.cpp. RPC success is not confirmation.
  return WithRobotStateClient(network_interface, [](auto& client) {
    Result before=QuerySportModeWithClient(client);
    if(!before.ok || before.state==sim2real::SportMode::ACTIVE) return before;
    int32_t status=-1;
    const int32_t ret=client.ServiceSwitch(kSportModeService,1,status);
    if(ret!=0) return Result{false,false,"Sport enable RPC failed: "+std::to_string(ret),sim2real::SportMode::ERROR};
    for(int attempt=0;attempt<15;++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      Result after=QuerySportModeWithClient(client);
      if(!after.ok || after.state==sim2real::SportMode::ACTIVE) return after;
    }
    return Result{false,false,"Sport ACTIVE not observed after enable",sim2real::SportMode::UNKNOWN};
  });
}

int RunSession(const std::string& network_interface,const std::string& config_path) {
  if(network_interface.empty()||config_path.empty()) return 2;
  double rpc_timeout_s;int confirm_period_ms,confirm_attempts;
  try {
    const auto config=YAML::LoadFile(config_path)["real_deployment"]["sport_mode"];
    rpc_timeout_s=config["sdk_rpc_timeout_s"].as<double>();
    confirm_period_ms=config["release_confirm_period_ms"].as<int>();
    confirm_attempts=config["release_confirm_attempts"].as<int>();
    if(!std::isfinite(rpc_timeout_s)||rpc_timeout_s<=0||rpc_timeout_s>.2||confirm_period_ms<1||confirm_period_ms>100||confirm_attempts<1||confirm_attempts>15)
      throw std::invalid_argument("SDK session timing outside existing contract");
  }catch(const std::exception& error){std::cerr<<"Invalid SDK session config: "<<error.what()<<std::endl;return 2;}
  auto* factory=unitree::robot::ChannelFactory::Instance();
  bool initialized=false;
  try {
    factory->Init(0,network_interface);initialized=true;
    {
      unitree::robot::go2::RobotStateClient client;
      // Status is diagnostic IO, not a five-second control-loop operation.
      client.SetTimeout(float(rpc_timeout_s));client.Init();
      std::string line;
      while(std::getline(std::cin,line)) {
        if(line.size()>128) break;
        std::istringstream input(line);uint64_t id=0;std::string command,extra;
        if(!(input>>id>>command)||(input>>extra)||!id||
           (command!="STATUS"&&command!="RELEASE")) break;
        const auto started=std::chrono::steady_clock::now();
        Result result=QuerySportModeWithClient(client);
        if(command=="RELEASE"&&result.ok&&result.state==sim2real::SportMode::ACTIVE) {
          int32_t status=-1;
          const auto rc=client.ServiceSwitch(kSportModeService,0,status);
          if(rc!=0) result={false,true,"Sport release RPC failed: "+std::to_string(rc),sim2real::SportMode::ERROR};
          else {
            for(int attempt=0;attempt<confirm_attempts;++attempt) {
              std::this_thread::sleep_for(std::chrono::milliseconds(confirm_period_ms));
              result=QuerySportModeWithClient(client);
              if(!result.ok||result.state==sim2real::SportMode::RELEASED) break;
            }
            if(result.state!=sim2real::SportMode::RELEASED) result.ok=false;
          }
        }
        const auto ended=std::chrono::steady_clock::now();
        // Only this tagged record is protocol output; SDK diagnostics are ignored.
        for(char& c:result.message) if(c=='\n'||c=='\r') c=' ';
        std::cout<<"R3_MODE_V1 "<<id<<' '<<sim2real::SportModeName(result.state)<<' '
          <<std::chrono::duration_cast<std::chrono::nanoseconds>(started.time_since_epoch()).count()<<' '
          <<std::chrono::duration_cast<std::chrono::nanoseconds>(ended.time_since_epoch()).count()<<' '
          <<result.ok<<' '<<result.message.substr(0,512)<<std::endl;
      }
    }
    factory->Release();return 0;
  } catch(const std::exception& e) {
    std::cerr<<"SDK session failed: "<<e.what()<<std::endl;
    if(initialized)factory->Release();return 1;
  }
}

}  // namespace go2_motion_mode
