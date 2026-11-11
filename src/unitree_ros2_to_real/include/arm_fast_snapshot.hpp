#pragma once
#include "arm_status_snapshot.hpp"
#include <rars01_state_interfaces/msg/arm_fast_state.hpp>
#include <limits>
namespace sim2real {
// Same admission contract as ParseArmStatus, with direct fixed-array reads.
// No YAML nodes, string serialization or dynamic numeric vectors on this path.
inline ParsedArmStatus ParseArmFastStatus(const rars01_state_interfaces::msg::ArmFastState& m,
                                        double home_tolerance,SafetyTime now) {
 ParsedArmStatus p;
 const bool fault=m.owner_state=="FAULT_LATCHED";
 auto stamp=[](uint64_t n){
  if(n>uint64_t(std::numeric_limits<int64_t>::max()))throw std::runtime_error("arm timestamp overflow");
  return SafetyTime(std::chrono::nanoseconds(int64_t(n)));
 };
 auto age=[](SafetyTime a,SafetyTime b){return std::chrono::duration<double>(a-b).count();};
 auto six=[](const auto& a){std::array<float,6> out;
  for(size_t i=0;i<6;++i){out[i]=float(a[i]);if(!std::isfinite(out[i]))throw std::runtime_error("Invalid arm values");}return out;};
 try {
  if(m.schema_version!=2)throw std::runtime_error("ARM_STATE_SCHEMA_VERSION");
  p.q=six(m.measured_q);p.dq=six(m.measured_dq);
  for(int i=0;i<6;++i)if(m.motor_ids[i]!=i+1||!m.valid[i])throw std::runtime_error("arm identity invalid");
  if(!std::isfinite(m.common_feedback_age_s)||m.common_feedback_age_s<0||age(now,stamp(m.frame_received_ns))<0)
   throw std::runtime_error("invalid arm frame age");
  const auto observed=stamp(m.observed_ns);
  if(age(now,observed)<0)throw std::runtime_error("future arm observation");
  // Reject ages that cannot be represented before converting floating seconds.
  if(m.common_feedback_age_s>double(std::numeric_limits<int64_t>::max())/1e9)
   throw std::runtime_error("arm age overflow");
  p.inputs.arm_stamp=observed-std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(m.common_feedback_age_s));
  p.inputs.ready.arm_feedback_ready=m.feedback_ready;
  p.inputs.arm_emergency_validated=m.arm_emergency_validated;
  p.inputs.arm_static_hold=m.static_hold;
  if(m.arm_home_ready){
   if(m.home_joint_count!=6&&m.home_joint_count!=7)throw std::runtime_error("invalid HOME joint scope");
   if(m.home_joint_count==6&&(!m.mission_capable||(m.grasp_verification!="stub"&&m.grasp_verification!="torque_width")))
    throw std::runtime_error("HOME6 requires explicit mission contract");
   const double grip_target=m.gripper_hold_confirmed?m.gripper_hold_target:0.;
   if(!std::isfinite(grip_target))throw std::runtime_error("invalid gripper hold target");
   for(size_t i=0;i<7;++i){
    // YAML reader historically rounds these arrays to float; preserve boundaries.
    const float q=float(m.q[i]),dq=float(m.dq[i]),target=float(m.home_target[i]);
    if(m.motor_id[i]!=int(i+1)||!m.valid7[i]||m.motor_status[i]!=1||!std::isfinite(q)||
       (i<m.home_joint_count&&std::abs(q-(i==6?grip_target:0.))>home_tolerance)||!std::isfinite(dq)||target!=0)
     throw std::runtime_error("HOME motor diagnostics invalid");
   }
   p.inputs.arm_home_ready=m.connected&&m.enabled_local&&!m.watchdog_tripped&&!m.stm32_watchdog_tripped;
  }
  p.inputs.arm_fault_confirmed=fault;
  const bool owner_healthy=m.connected&&m.enabled_local&&!m.watchdog_tripped&&!m.stm32_watchdog_tripped&&!fault;
  const bool motors_healthy=std::all_of(m.motor_status.begin(),m.motor_status.end(),[](auto v){return v==1;});
  if(m.target_valid){
   if(!m.accepted_target_valid)throw std::runtime_error("missing accepted target");
   p.target=six(m.accepted_target);
   if(p.inputs.arm_home_ready)for(float v:p.target)if(std::abs(v)>home_tolerance)throw std::runtime_error("HOME accepted target outside tolerance");
   p.inputs.target_stamp=stamp(m.accepted_ns);
   p.inputs.ready.arm_target_ready=age(now,p.inputs.target_stamp)>=0;
   p.inputs.arm_control_ready=owner_healthy&&motors_healthy&&p.inputs.ready.arm_feedback_ready&&p.inputs.ready.arm_target_ready;
  }
 }catch(const std::exception& e){p=ParsedArmStatus{};p.inputs.arm_fault_confirmed=fault;p.error=e.what();}
 return p;
}
}
