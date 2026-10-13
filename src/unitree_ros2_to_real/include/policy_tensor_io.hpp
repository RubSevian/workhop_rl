#pragma once

#include <torch/torch.h>
#include <algorithm>
#include <array>

namespace sim2real {
// Only worker-owned observation inputs may be reused. Never pass action/history
// tensors: those are immutable snapshots used by PolicyStateTransaction.
template<std::size_t N>
void CopyPolicyInput(torch::Tensor& destination, const std::array<float,N>& source) {
  // Preserve the generic/autograd path used outside the real inference worker.
  if (!c10::InferenceMode::is_enabled()) {
    destination=torch::tensor(std::vector<float>(source.begin(),source.end()),torch::kFloat32);
    return;
  }
  if (!destination.defined() || destination.dim()!=1 || destination.numel()!=N ||
      !destination.device().is_cpu() || destination.scalar_type()!=torch::kFloat32 ||
      !destination.is_contiguous() || destination.requires_grad() ||
      !destination.is_inference()) {
    destination=torch::empty({static_cast<int64_t>(N)},torch::kFloat32);
  }
  std::copy(source.begin(),source.end(),destination.data_ptr<float>());
}

inline const float* PolicyOutputData(const torch::Tensor& value) {
  if (!value.defined() || value.dim()!=1 || value.numel()!=12 ||
      !value.device().is_cpu() || value.scalar_type()!=torch::kFloat32 ||
      !value.is_contiguous()) {
    throw std::runtime_error("Policy output must be contiguous CPU float32 [12]");
  }
  return value.data_ptr<float>();
}
}  // namespace sim2real
