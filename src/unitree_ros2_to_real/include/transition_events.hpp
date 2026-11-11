#pragma once
#include "control_diagnostics.hpp"
#include <array>
#include <optional>
#include <utility>
namespace sim2real {
struct TransitionEvent { ControlDiagnostics data; bool fault=false; };
// All access is serialized by the controller mutex. Formatting/logging is
// exclusively done by the status callback after releasing that mutex.
template<size_t Capacity=64> class TransitionEvents {
 public:
  void Push(TransitionEvent event) {
    if(size_==Capacity){++dropped_;if(event.fault)overflow_fault_=std::move(event);return;}
    slots_[(head_+size_)%Capacity]=std::move(event);++size_;
  }
  std::optional<TransitionEvent> Pop() {
    if(!size_)return {};
    auto result=std::move(slots_[head_]);slots_[head_].reset();head_=(head_+1)%Capacity;--size_;return result;
  }
  std::optional<TransitionEvent> TakeOverflowFault(){return std::exchange(overflow_fault_,{});}
  size_t dropped()const{return dropped_;}
  static constexpr size_t capacity=Capacity;
 private:
  std::array<std::optional<TransitionEvent>,Capacity> slots_{};
  std::optional<TransitionEvent> overflow_fault_;
  size_t head_=0,size_=0,dropped_=0;
};
}
