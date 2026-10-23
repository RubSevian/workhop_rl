#include "optional_yaml_scalar.hpp"
#include <cassert>
#include <iostream>
#include <vector>

int main() {
  for (const auto width : {std::optional<double>{}, std::optional<double>{0.}, std::optional<double>{0.05}}) {
    YAML::Emitter out;
    out << YAML::Flow << YAML::BeginMap
        << YAML::Key << "grasp_width_m" << YAML::Value << sim2real::OptionalYamlScalar{width}
        << YAML::Key << "grasp_confirmation_s" << YAML::Value << 0.5
        << YAML::Key << "owner_state" << YAML::Value << "HOLD_HOME"
        << YAML::Key << "measured_q" << YAML::Value << std::vector<double>(6, 0.)
        << YAML::Key << "feedback_ready" << YAML::Value << true << YAML::EndMap;
    assert(out.good());
    const auto parsed = YAML::Load(out.c_str());
    assert(parsed.IsMap() && parsed.size() == 5);
    for (const auto entry : parsed) assert(entry.first.IsScalar());
    if (width) assert(parsed["grasp_width_m"].as<double>() == *width);
    else assert(parsed["grasp_width_m"].IsNull());
    assert(parsed["grasp_confirmation_s"].as<double>() == 0.5);
    assert(parsed["owner_state"].as<std::string>() == "HOLD_HOME");
    assert(parsed["measured_q"].as<std::vector<double>>().size() == 6);
    assert(parsed["feedback_ready"].as<bool>());
    std::cout << out.c_str() << '\n';
  }
}
