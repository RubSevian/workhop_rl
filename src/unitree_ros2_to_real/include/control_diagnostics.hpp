#pragma once

#include "operation_profile.hpp"
#include <yaml-cpp/yaml.h>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace sim2real {
// Capture owned values while holding the control mutex; format and publish
// afterwards. No references to the live supervisor escape the capture.
class ControlDiagnostics {
 public:
  using Value = std::variant<bool, int64_t, uint64_t, double, std::string,
      std::array<float, 12>, std::array<double, 3>,
      std::vector<std::string>, OperationCapabilities>;
  // CaptureStatus includes129 fields in compute-probe mode. Keep bounded
  // headroom without dropping fields or weakening any control deadline.
  static constexpr size_t capacity = 160;

  template<class T> void Add(const char* key, T&& value) {
    if (size_ == capacity) throw std::length_error("control diagnostic capacity exceeded");
    using U = std::remove_cvref_t<T>;
    auto& field = fields_[size_++];
    field.key = key;
    if constexpr (std::is_same_v<U, bool>) field.value = value;
    else if constexpr (std::is_integral_v<U> && std::is_signed_v<U>) field.value = int64_t(value);
    else if constexpr (std::is_integral_v<U>) field.value = uint64_t(value);
    else if constexpr (std::is_floating_point_v<U>) field.value = double(value);
    else if constexpr (std::is_convertible_v<T, const char*>) field.value = std::string(value);
    else field.value = std::forward<T>(value);
  }

  std::string Serialize() const {
    YAML::Emitter output;
    output << YAML::Flow << YAML::BeginMap;
    for (size_t i = 0; i < size_; ++i) {
      output << YAML::Key << fields_[i].key << YAML::Value;
      std::visit([&](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::string>) {
          // In particular ON/OFF must remain strings for YAML 1.1 readers.
          output << YAML::DoubleQuoted << value << YAML::Auto;
        } else if constexpr (std::is_same_v<T, OperationCapabilities>) {
          output << YAML::BeginMap
            << YAML::Key << "leg_output" << YAML::Value << value.physical_leg_output
            << YAML::Key << "sport_release" << YAML::Value << value.allow_sport_release
            << YAML::Key << "rl" << YAML::Value << value.allow_rl
            << YAML::Key << "nonzero_velocity" << YAML::Value << value.allow_nonzero_velocity
            << YAML::Key << "arm_motion" << YAML::Value << value.allow_arm_motion << YAML::EndMap;
        } else if constexpr (std::is_same_v<T, std::array<float, 12>> ||
                             std::is_same_v<T, std::array<double, 3>>) {
          output << YAML::BeginSeq;
          for (auto item : value) output << item;
          output << YAML::EndSeq;
        } else output << value;
      }, fields_[i].value);
    }
    output << YAML::EndMap;
    if (!output.good()) throw std::runtime_error(output.GetLastError());
    return output.c_str();
  }

 private:
  struct Field { const char* key = nullptr; Value value = false; };
  std::array<Field, capacity> fields_{};
  size_t size_ = 0;
};
}  // namespace sim2real
