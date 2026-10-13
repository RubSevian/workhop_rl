#include "control_diagnostics.hpp"
#include <cassert>

int main() {
  sim2real::ControlDiagnostics snapshot;
  std::string fault = "quote: \"x\"\nline";
  std::array<float,12> pose{}; pose[3] = 1.25F;
  snapshot.Add("fault", fault);
  snapshot.Add("custom_leg_output", "ON");
  snapshot.Add("output_enabled", false);
  snapshot.Add("packets", uint64_t(123));
  snapshot.Add("pose", pose);
  snapshot.Add("limits", std::array<double,3>{.2,.1,.1});
  snapshot.Add("blockers", std::vector<std::string>{"one","two"});
  auto capabilities = sim2real::OperationCapabilities{};
  capabilities.physical_leg_output = true;
  snapshot.Add("capabilities", capabilities);
  fault.clear(); pose[3] = -99; // Live data must not change a captured status.
  auto parsed = YAML::Load(snapshot.Serialize());
  assert(parsed["fault"].as<std::string>() == "quote: \"x\"\nline");
  assert(parsed["custom_leg_output"].as<std::string>() == "ON");
  assert(!parsed["output_enabled"].as<bool>());
  assert(parsed["packets"].as<uint64_t>() == 123);
  assert(parsed["pose"][3].as<float>() == 1.25F);
  assert(parsed["capabilities"]["leg_output"].as<bool>());
  assert(parsed["blockers"].size() == 2);
  sim2real::ControlDiagnostics full;
  for (size_t i=0; i<sim2real::ControlDiagnostics::capacity; ++i) full.Add("x", i);
  bool bounded = false;
  try { full.Add("overflow", true); } catch (const std::length_error&) { bounded = true; }
  assert(bounded);
}
