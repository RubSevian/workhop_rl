#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rars01_sim {

// The seventh YAML entry is reserved for the real stack's gripper-motor
// parameter. The paired MuJoCo gripper keeps its existing control gains.
inline constexpr double kGripperKp = 20.0;
inline constexpr double kGripperKd = 0.2;

struct ArmGains {
  std::array<double, 7> position_kp{};
  std::array<double, 7> position_kd{};
};

inline ArmGains LoadArmGains(const std::string& path) {
  try {
    const YAML::Node arm = YAML::LoadFile(path)["rars01_arm"];
    if (!arm || !arm.IsMap()) {
      throw std::runtime_error("rars01_arm map missing");
    }
    ArmGains gains;
    const auto read = [&](const char* key, std::array<double, 7>& values,
                          bool strictly_positive) {
      const YAML::Node entries = arm[key];
      if (!entries || !entries.IsSequence() || entries.size() != values.size()) {
        throw std::runtime_error(std::string(key) + " must have exactly 7 values");
      }
      for (std::size_t index = 0; index < values.size(); ++index) {
        const double value = entries[index].as<double>();
        if (!std::isfinite(value) || (strictly_positive ? value <= 0.0 : value < 0.0)) {
          throw std::runtime_error(std::string(key) + " invalid at index " +
                                   std::to_string(index));
        }
        values[index] = value;
      }
    };
    read("position_kp", gains.position_kp, true);
    read("position_kd", gains.position_kd, false);
    return gains;
  } catch (const std::exception& error) {
    throw std::runtime_error("Invalid RARS01 arm simulation config '" + path +
                             "': " + error.what());
  }
}

}  // namespace rars01_sim
