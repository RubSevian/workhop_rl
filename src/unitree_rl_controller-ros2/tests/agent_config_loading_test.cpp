#include "rl_agent.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::filesystem::path RuntimeSourceDir() {
  return std::filesystem::path(UNITREE_RL_CONTROLLER_SOURCE_DIR).parent_path() /
         "unitree_ros2_to_real";
}

void TestLegacyGo2Config() {
  Agent agent;
  const auto config = RuntimeSourceDir() / "weights" / "go2" / "config.yaml";
  agent.ReadYaml("go2", config.string());
  Expect(agent.params.model_name == "policy_1.pt", "legacy go2 config must load its model name");
  Expect(agent.params.joint_names.size() == 12, "legacy go2 config must load 12 leg joints");
}

void TestUnifiedSelfDescribingConfig() {
  Agent agent;
  const auto config = RuntimeSourceDir() / "config" / "go2_rars01_unified.yaml";
  // The caller still supplies the legacy runtime identity. ReadYaml must find
  // the self-describing go2_rars01 root for an explicit unified config path.
  agent.ReadYaml("go2", config.string());
  Expect(agent.params.observation_layout == "go2_rars01_unified_v1",
         "unified config fallback must select go2_rars01 layout");
  agent.InitRL();
  const auto history = agent.UnifiedActorHistory();
  Expect(history.dim() == 2 && history.size(0) == 1 && history.size(1) == 315,
         "unified config must initialize [1, 315] history");
}

}  // namespace

int main() {
  try {
    TestLegacyGo2Config();
    TestUnifiedSelfDescribingConfig();
    std::cout << "agent_config_loading_test: PASS" << std::endl;
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "agent_config_loading_test: FAIL: " << error.what() << std::endl;
    return EXIT_FAILURE;
  }
}
