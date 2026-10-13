#include "unified_observation_contract.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace go2_rars01 {

namespace {
torch::Tensor DefaultLegPose() {
  return torch::tensor({0.1F, 0.8F, -1.5F,
                        -0.1F, 0.8F, -1.5F,
                         0.1F, 0.8F, -1.5F,
                        -0.1F, 0.8F, -1.5F}, torch::kFloat32);
}
bool CpuFloatVector(const torch::Tensor& value) {
  return value.defined() && value.device().is_cpu() &&
         value.scalar_type()==torch::kFloat32 && value.is_contiguous();
}
}  // namespace

UnifiedObservationContract::UnifiedObservationContract()
    : default_leg_pos_(DefaultLegPose()) {}

UnifiedObservationContract::UnifiedObservationContract(
    const torch::Tensor& default_leg_pos, const UnifiedObservationScales& scales) {
  Configure(default_leg_pos, scales);
}

void UnifiedObservationContract::Configure(const torch::Tensor& default_leg_pos,
                                           const UnifiedObservationScales& scales) {
  RequireVector(default_leg_pos, Dimensions::LegPos, "default_leg_pos");
  if (!std::isfinite(scales.ang_vel) || !std::isfinite(scales.dof_pos) ||
      !std::isfinite(scales.dof_vel) || !std::isfinite(scales.clip_observations)) {
    throw std::invalid_argument("Unified observation scales must be finite");
  }
  for (const float scale : scales.command) {
    if (!std::isfinite(scale)) throw std::invalid_argument("command scale must be finite");
  }
  default_leg_pos_ = default_leg_pos.to(torch::kFloat32).contiguous().clone();
  scales_ = scales;
  frames_ = {};
  initialized_ = false;
}

void UnifiedObservationContract::RequireVector(const torch::Tensor& value, int size,
                                               const char* name) {
  if (!value.defined() || value.dim() != 1 || value.numel() != size) {
    const auto actual = value.defined() ? value.numel() : 0;
    throw std::invalid_argument(std::string(name) + " must be a 1D vector of " +
                                std::to_string(size) + " values, got " +
                                std::to_string(actual));
  }
  if (CpuFloatVector(value)) {
    const auto* data=value.data_ptr<float>();
    for(int i=0;i<size;++i)if(!std::isfinite(data[i]))
      throw std::invalid_argument(std::string(name)+" must contain finite values");
  } else if (!torch::isfinite(value).all().item<bool>()) {
    throw std::invalid_argument(std::string(name) + " must contain finite values");
  }
}

torch::Tensor UnifiedObservationContract::ProjectedGravity(const torch::Tensor& quaternion) {
  RequireVector(quaternion, 4, "base_quat");
  auto q = quaternion.to(torch::kFloat32);
  const float norm = torch::norm(q).item<float>();
  if (!std::isfinite(norm) || norm < 1.0e-8F) {
    throw std::invalid_argument("base_quat must be a non-zero quaternion [x, y, z, w]");
  }
  q = q / norm;
  const auto x = q.index({0});
  const auto y = q.index({1});
  const auto z = q.index({2});
  const auto w = q.index({3});
  const auto gravity_world = torch::tensor({0.0F, 0.0F, -1.0F}, torch::kFloat32);
  const auto q_vec = torch::stack({x, y, z});
  // Inverse quaternion rotation, equivalent to the training projected-gravity path.
  return gravity_world * (2.0F * w * w - 1.0F) -
         torch::cross(q_vec, gravity_world, 0) * w * 2.0F +
         q_vec * torch::dot(q_vec, gravity_world) * 2.0F;
}

torch::Tensor UnifiedObservationContract::BuildFrame(const UnifiedObservationState& state) const {
  RequireVector(state.ang_vel, Dimensions::BaseAngVel, "ang_vel");
  RequireVector(state.command, Dimensions::Command, "command");
  RequireVector(state.base_quat, 4, "base_quat");
  RequireVector(state.leg_pos, Dimensions::LegPos, "leg_pos");
  RequireVector(state.leg_vel, Dimensions::LegVel, "leg_vel");
  RequireVector(state.previous_action, Dimensions::PreviousAction, "previous_action");
  RequireVector(state.arm_pos, Dimensions::ArmPos, "arm_pos");
  RequireVector(state.arm_vel, Dimensions::ArmVel, "arm_vel");
  RequireVector(state.arm_target, Dimensions::ArmTarget, "arm_target");

  // One owned immutable frame, rather than many tiny ATen temporaries.
  // Keep the original tensor expression for autograd/non-CPU/non-float inputs.
  if(c10::InferenceMode::is_enabled() && scales_.clip_observations>=0 &&
     CpuFloatVector(default_leg_pos_) && CpuFloatVector(state.ang_vel) &&
     CpuFloatVector(state.command) && CpuFloatVector(state.base_quat) &&
     CpuFloatVector(state.leg_pos) && CpuFloatVector(state.leg_vel) &&
     CpuFloatVector(state.previous_action) && CpuFloatVector(state.arm_pos) &&
     CpuFloatVector(state.arm_vel) && CpuFloatVector(state.arm_target)) {
    const auto* q=state.base_quat.data_ptr<float>();
    const float norm=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(!std::isfinite(norm)||norm<1.e-8F)
      throw std::invalid_argument("base_quat must be a non-zero quaternion [x, y, z, w]");
    const float x=q[0]/norm,y=q[1]/norm,z=q[2]/norm,w=q[3]/norm;
    auto frame=torch::empty({Dimensions::ActorFrame},torch::kFloat32);
    auto* out=frame.data_ptr<float>();
    for(int i=0;i<3;++i)out[i]=state.ang_vel.data_ptr<float>()[i]*scales_.ang_vel;
    out[3]=2*y*w-2*x*z;
    out[4]=-2*x*w-2*y*z;
    out[5]=-(2*w*w-1)-2*z*z;
    for(int i=0;i<3;++i)out[6+i]=state.command.data_ptr<float>()[i]*scales_.command[i];
    for(int i=0;i<12;++i) {
      out[9+i]=(state.leg_pos.data_ptr<float>()[i]-default_leg_pos_.data_ptr<float>()[i])*scales_.dof_pos;
      out[21+i]=state.leg_vel.data_ptr<float>()[i]*scales_.dof_vel;
      out[33+i]=state.previous_action.data_ptr<float>()[i];
    }
    for(int i=0;i<6;++i) {
      out[45+i]=state.arm_pos.data_ptr<float>()[i];
      out[51+i]=state.arm_vel.data_ptr<float>()[i]*scales_.dof_vel;
      out[57+i]=state.arm_target.data_ptr<float>()[i];
    }
    RequireFrame(frame); // Also rejects overflow before clipping.
    for(int i=0;i<Dimensions::ActorFrame;++i)
      out[i]=std::clamp(out[i],-scales_.clip_observations,scales_.clip_observations);
    return frame;
  }
  const auto command_scale = torch::tensor(
      {scales_.command[0], scales_.command[1], scales_.command[2]}, torch::kFloat32);
  auto frame = torch::cat({
      state.ang_vel.to(torch::kFloat32) * scales_.ang_vel,
      ProjectedGravity(state.base_quat),
      state.command.to(torch::kFloat32) * command_scale,
      (state.leg_pos.to(torch::kFloat32) - default_leg_pos_) * scales_.dof_pos,
      state.leg_vel.to(torch::kFloat32) * scales_.dof_vel,
      state.previous_action.to(torch::kFloat32),
      state.arm_pos.to(torch::kFloat32),
      state.arm_vel.to(torch::kFloat32) * scales_.dof_vel,
      state.arm_target.to(torch::kFloat32)}, 0);
  RequireFrame(frame);
  return torch::clamp(frame, -scales_.clip_observations, scales_.clip_observations);
}

void UnifiedObservationContract::RequireFrame(const torch::Tensor& frame) const {
  RequireVector(frame, Dimensions::ActorFrame, "actor frame");
}

void UnifiedObservationContract::Reset(const torch::Tensor& first_frame) {
  RequireFrame(first_frame);
  frames_.fill(first_frame.to(torch::kFloat32).contiguous().clone());
  initialized_ = true;
}

void UnifiedObservationContract::Insert(const torch::Tensor& frame) {
  RequireFrame(frame);
  if (!initialized_) {
    Reset(frame);
    return;
  }
  auto owned_frame = frame.to(torch::kFloat32).contiguous().clone();
  for (int i = 0; i < Dimensions::History - 1; ++i)
    frames_[i] = std::move(frames_[i + 1]);
  frames_.back() = std::move(owned_frame);
}

bool UnifiedObservationContract::initialized() const { return initialized_; }

torch::Tensor UnifiedObservationContract::History() const {
  if (!initialized()) throw std::logic_error("Unified observation history is not initialized");
  return torch::cat(frames_, 0).view({1, Dimensions::ActorInput});
}

void UnifiedObservationContract::ResetFromState(const UnifiedObservationState& state) {
  Reset(BuildFrame(state));
}

torch::Tensor UnifiedObservationContract::InsertAndGetHistory(const UnifiedObservationState& state) {
  Insert(BuildFrame(state));
  return History();
}

}  // namespace go2_rars01
