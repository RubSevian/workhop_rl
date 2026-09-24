#include "unified_observation_contract.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <stdexcept>

using go2_rars01::Dimensions;
using go2_rars01::UnifiedObservationContract;
using go2_rars01::UnifiedObservationState;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void ExpectClose(float actual, float expected, const std::string& message) {
  if (std::abs(actual - expected) > 1.0e-6F) {
    throw std::runtime_error(message + ": expected " + std::to_string(expected) +
                             ", got " + std::to_string(actual));
  }
}

torch::Tensor Range(float start, int count) {
  return torch::arange(start, start + count, 1.0F, torch::kFloat32);
}

UnifiedObservationState MarkedState() {
  UnifiedObservationState state;
  state.ang_vel = torch::tensor({4.0F, 8.0F, 12.0F});
  state.command = torch::tensor({1.0F, 2.0F, 4.0F});
  state.base_quat = torch::tensor({0.0F, 0.0F, 0.0F, 1.0F});
  state.leg_pos = Range(10.0F, Dimensions::LegPos);
  state.leg_vel = Range(30.0F, Dimensions::LegVel);
  state.previous_action = Range(50.0F, Dimensions::PreviousAction);
  state.arm_pos = Range(70.0F, Dimensions::ArmPos);
  state.arm_vel = Range(80.0F, Dimensions::ArmVel);
  state.arm_target = Range(90.0F, Dimensions::ArmTarget);
  return state;
}

void TestFrameLayout() {
  UnifiedObservationContract contract;
  auto state = MarkedState();
  const auto frame = contract.BuildFrame(state);
  Expect(frame.numel() == Dimensions::ActorFrame, "frame shape must be 63");
  ExpectClose(frame.index({0}).item<float>(), 1.0F, "angular velocity scale");
  ExpectClose(frame.index({2}).item<float>(), 3.0F, "angular velocity scale");
  ExpectClose(frame.index({3}).item<float>(), 0.0F, "identity gravity x");
  ExpectClose(frame.index({5}).item<float>(), -1.0F, "identity gravity z");
  ExpectClose(frame.index({6}).item<float>(), 2.0F, "vx scale");
  ExpectClose(frame.index({7}).item<float>(), 4.0F, "vy scale");
  ExpectClose(frame.index({8}).item<float>(), 1.0F, "yaw scale");
  ExpectClose(frame.index({9}).item<float>(), 9.9F, "leg default subtraction");
  ExpectClose(frame.index({21}).item<float>(), 1.5F, "leg velocity scale");
  ExpectClose(frame.index({33}).item<float>(), 50.0F, "previous action placement");
  ExpectClose(frame.index({45}).item<float>(), 70.0F, "raw arm position placement");
  ExpectClose(frame.index({51}).item<float>(), 4.0F, "arm velocity scale");
  ExpectClose(frame.index({57}).item<float>(), 90.0F, "arm target placement");

  state.leg_pos = torch::tensor({0.1F, 0.8F, -1.5F, -0.1F, 0.8F, -1.5F,
                                  0.1F, 0.8F, -1.5F, -0.1F, 0.8F, -1.5F});
  const auto zero_legs = contract.BuildFrame(state).slice(0, 9, 21);
  Expect(torch::allclose(zero_legs, torch::zeros({Dimensions::LegPos})),
         "default leg pose must produce zero leg error");
}

void TestNamedPolicyOrderAndExcludedTerms() {
  Expect(go2_rars01::LegPolicyIndex("FL_hip_joint") == 0, "front-left hip index");
  Expect(go2_rars01::LegPolicyIndex("RR_calf_joint") == 11, "rear-right calf index");
  Expect(go2_rars01::LegPolicyIndex("unknown_joint") == -1, "unknown leg index");
  Expect(go2_rars01::ArmPolicyIndex("joint1") == 0, "first arm index");
  Expect(go2_rars01::ArmPolicyIndex("joint6") == 5, "last arm index");

  // lin_vel, phase sin/cos, and height samples have no fields in
  // UnifiedObservationState. Supplying legacy values cannot affect this frame.
  UnifiedObservationContract contract;
  const auto baseline = contract.BuildFrame(MarkedState());
  const auto legacy_lin_vel = torch::full({12}, 999.0F);
  const auto legacy_phase = torch::full({2}, 999.0F);
  const auto legacy_height = torch::full({187}, 999.0F);
  (void)legacy_lin_vel;
  (void)legacy_phase;
  (void)legacy_height;
  Expect(torch::allclose(baseline, contract.BuildFrame(MarkedState())),
         "excluded legacy terms must not enter the unified frame");
}

void TestProjectedGravity() {
  UnifiedObservationContract contract;
  auto state = MarkedState();
  const float root_half = std::sqrt(0.5F);
  state.base_quat = torch::tensor({root_half, 0.0F, 0.0F, root_half});
  auto gravity = contract.BuildFrame(state).slice(0, 3, 6);
  ExpectClose(gravity.index({0}).item<float>(), 0.0F, "90 degree roll gravity x");
  ExpectClose(gravity.index({1}).item<float>(), -1.0F, "90 degree roll gravity y");
  state.base_quat = torch::tensor({0.0F, root_half, 0.0F, root_half});
  gravity = contract.BuildFrame(state).slice(0, 3, 6);
  ExpectClose(gravity.index({0}).item<float>(), 1.0F, "90 degree pitch gravity x");
  ExpectClose(gravity.index({2}).item<float>(), 0.0F, "90 degree pitch gravity z");
  state.base_quat = torch::tensor({0.2F, -0.3F, 0.1F, 0.9F});
  Expect(torch::isfinite(contract.BuildFrame(state)).all().item<bool>(),
         "normalized nontrivial quaternion must be finite");
  state.base_quat = torch::zeros({4});
  bool rejected = false;
  try { contract.BuildFrame(state); } catch (const std::invalid_argument&) { rejected = true; }
  Expect(rejected, "zero quaternion must fail clearly");
}

void TestHistoryAndActionTiming() {
  UnifiedObservationContract contract;
  auto frame0 = torch::full({Dimensions::ActorFrame}, 0.0F);
  contract.Reset(frame0);
  Expect(torch::allclose(contract.History(), torch::zeros({1, Dimensions::ActorInput})),
         "reset must repeat first frame rather than zero-pad");
  for (int i = 1; i <= 4; ++i) contract.Insert(torch::full({Dimensions::ActorFrame}, static_cast<float>(i)));
  auto history = contract.History().view({Dimensions::History, Dimensions::ActorFrame});
  for (int i = 0; i < Dimensions::History; ++i) {
    ExpectClose(history.index({i, 0}).item<float>(), static_cast<float>(i), "history chronology");
  }
  contract.Insert(torch::full({Dimensions::ActorFrame}, 5.0F));
  history = contract.History().view({Dimensions::History, Dimensions::ActorFrame});
  for (int i = 0; i < Dimensions::History; ++i) {
    ExpectClose(history.index({i, 0}).item<float>(), static_cast<float>(i + 1), "history roll left");
  }

  auto state = MarkedState();
  state.previous_action = torch::zeros({Dimensions::PreviousAction});
  const auto before_action = contract.BuildFrame(state);
  state.previous_action = torch::full({Dimensions::PreviousAction}, 7.0F);
  const auto after_action = contract.BuildFrame(state);
  ExpectClose(before_action.index({33}).item<float>(), 0.0F, "previous action before policy output");
  ExpectClose(after_action.index({33}).item<float>(), 7.0F, "previous action next frame");
}

void TestClippingAndDimensions() {
  UnifiedObservationContract contract;
  auto state = MarkedState();
  state.arm_pos = torch::full({Dimensions::ArmPos}, 1000.0F);
  ExpectClose(contract.BuildFrame(state).index({45}).item<float>(), 100.0F, "observation clip");
  state.arm_pos = torch::zeros({5});
  bool rejected = false;
  try { contract.BuildFrame(state); } catch (const std::invalid_argument&) { rejected = true; }
  Expect(rejected, "wrong arm dimension must fail");
}

void DumpParityFixture() {
  UnifiedObservationContract contract;
  auto state = MarkedState();
  const auto frame = contract.BuildFrame(state);
  contract.Reset(frame);
  contract.Insert(torch::full({Dimensions::ActorFrame}, 1.0F));
  contract.Insert(torch::full({Dimensions::ActorFrame}, 2.0F));
  const auto history = contract.History().flatten();
  std::cout << std::setprecision(9) << "FRAME";
  for (int i = 0; i < frame.numel(); ++i) std::cout << ' ' << frame.index({i}).item<float>();
  std::cout << "\nHISTORY";
  for (int i = 0; i < history.numel(); ++i) std::cout << ' ' << history.index({i}).item<float>();
  std::cout << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--dump-parity-fixture") {
      DumpParityFixture();
      return EXIT_SUCCESS;
    }
    TestFrameLayout();
    TestNamedPolicyOrderAndExcludedTerms();
    TestProjectedGravity();
    TestHistoryAndActionTiming();
    TestClippingAndDimensions();
    std::cout << "unified_observation_contract_test: PASS" << std::endl;
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "unified_observation_contract_test: FAIL: " << error.what() << std::endl;
    return EXIT_FAILURE;
  }
}
