#pragma once
#include <optional>
#include <yaml-cpp/yaml.h>

namespace sim2real {
// Emit scalar/null directly: inserting a Node into an active emitter can
// disrupt the surrounding map on the deployed yaml-cpp version.
struct OptionalYamlScalar { std::optional<double> value; };
inline YAML::Emitter& operator<<(YAML::Emitter& out, const OptionalYamlScalar& scalar) {
  if (scalar.value) out << *scalar.value;
  else out << YAML::Null;
  return out;
}
}
