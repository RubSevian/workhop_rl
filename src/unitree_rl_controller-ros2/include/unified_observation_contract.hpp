#pragma once

#include <torch/torch.h>

#include <array>
#include <string>
#include <vector>

namespace go2_rars01 {

struct Dimensions {
  static constexpr int BaseAngVel = 3;
  static constexpr int Gravity = 3;
  static constexpr int Command = 3;
  static constexpr int LegPos = 12;
  static constexpr int LegVel = 12;
  static constexpr int PreviousAction = 12;
  static constexpr int ArmPos = 6;
  static constexpr int ArmVel = 6;
  static constexpr int ArmTarget = 6;
  static constexpr int ActorFrame = 63;
  static constexpr int History = 5;
  static constexpr int ActorInput = ActorFrame * History;
  static constexpr int ActorOutput = 12;
};

// Policy order is deliberately independent from simulator joint indexing.
static const std::array<std::string, Dimensions::LegPos> kLegJointNames = {{
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}};

static const std::array<std::string, Dimensions::ArmPos> kArmJointNames = {{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}};

inline int LegPolicyIndex(const std::string& joint_name) {
  for (std::size_t index = 0; index < kLegJointNames.size(); ++index) {
    if (kLegJointNames[index] == joint_name) return static_cast<int>(index);
  }
  return -1;
}

inline int ArmPolicyIndex(const std::string& joint_name) {
  for (std::size_t index = 0; index < kArmJointNames.size(); ++index) {
    if (kArmJointNames[index] == joint_name) return static_cast<int>(index);
  }
  return -1;
}

struct UnifiedObservationState {
  torch::Tensor ang_vel;
  torch::Tensor command;
  torch::Tensor base_quat;  // ROS/MuJoCo convention: [x, y, z, w].
  torch::Tensor leg_pos;
  torch::Tensor leg_vel;
  torch::Tensor previous_action;
  torch::Tensor arm_pos;
  torch::Tensor arm_vel;
  torch::Tensor arm_target;
};

struct UnifiedObservationScales {
  float ang_vel = 0.25F;
  float dof_pos = 1.0F;
  float dof_vel = 0.05F;
  std::array<float, Dimensions::Command> command = {{2.0F, 2.0F, 0.25F}};
  float clip_observations = 100.0F;
};

class UnifiedObservationContract {
 public:
  UnifiedObservationContract();
  explicit UnifiedObservationContract(const torch::Tensor& default_leg_pos,
                                      const UnifiedObservationScales& scales = {});

  void Configure(const torch::Tensor& default_leg_pos,
                 const UnifiedObservationScales& scales = {});
  torch::Tensor BuildFrame(const UnifiedObservationState& state) const;
  void Reset(const torch::Tensor& first_frame);
  void Insert(const torch::Tensor& frame);
  bool initialized() const;
  torch::Tensor History() const;
  void ResetFromState(const UnifiedObservationState& state);
  torch::Tensor InsertAndGetHistory(const UnifiedObservationState& state);

 private:
  static void RequireVector(const torch::Tensor& value, int size, const char* name);
  static torch::Tensor ProjectedGravity(const torch::Tensor& quaternion);
  void RequireFrame(const torch::Tensor& frame) const;

  torch::Tensor default_leg_pos_;
  UnifiedObservationScales scales_;
  std::vector<torch::Tensor> frames_;
};

}  // namespace go2_rars01
