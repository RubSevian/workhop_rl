#include "rl_agent.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void ConfigureCurrentState(Agent& agent) {
  agent.params.observation_layout = "go2_rars01_unified_v1";
  agent.obs.ang_vel = torch::tensor({1.0F, -2.0F, 3.0F});
  agent.obs.command = torch::tensor({0.3F, -0.2F, 0.1F});
  agent.obs.base_quat = torch::tensor({0.0F, 0.0F, 0.0F, 1.0F});
  agent.obs.dof_pos = torch::arange(0.0F, 12.0F, 1.0F);
  agent.obs.dof_vel = torch::arange(12.0F, 24.0F, 1.0F);
  agent.obs.arm_pos = torch::arange(20.0F, 26.0F, 1.0F);
  agent.obs.arm_vel = torch::arange(30.0F, 36.0F, 1.0F);
  agent.obs.arm_target = torch::arange(40.0F, 46.0F, 1.0F);
}

void TestResetClearsPreviousActionAndRepeatsFrame() {
  Agent agent;
  ConfigureCurrentState(agent);
  agent.obs.action = torch::full({go2_rars01::Dimensions::PreviousAction}, 7.0F);

  agent.ResetPolicyState();

  Expect(torch::allclose(agent.obs.action,
                         torch::zeros({go2_rars01::Dimensions::PreviousAction})),
         "reset must clear previous policy action");
  const auto history = agent.UnifiedActorHistory();
  Expect(history.dim() == 2 && history.size(0) == 1 &&
             history.size(1) == go2_rars01::Dimensions::ActorInput,
         "reset actor history must have shape [1, 315]");
  const auto frames = history.view({go2_rars01::Dimensions::History,
                                    go2_rars01::Dimensions::ActorFrame});
  const auto zero_actions = torch::zeros({go2_rars01::Dimensions::PreviousAction});
  for (int index = 0; index < go2_rars01::Dimensions::History; ++index) {
    Expect(torch::allclose(frames.index({index}).slice(0, 33, 45), zero_actions),
           "every reset frame must contain zero previous action");
    Expect(torch::allclose(frames.index({index}), frames.index({0})),
           "reset must repeat the first current frame five times");
  }
}

}  // namespace

int main() {
  try {
    TestResetClearsPreviousActionAndRepeatsFrame();
    std::cout << "agent_unified_reset_test: PASS" << std::endl;
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "agent_unified_reset_test: FAIL: " << error.what() << std::endl;
    return EXIT_FAILURE;
  }
}
