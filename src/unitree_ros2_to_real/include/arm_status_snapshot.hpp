#pragma once
#include "r3_commissioning.hpp"
#include <algorithm>
#include <cmath>
namespace sim2real {
struct ParsedArmStatus {
 R3Inputs inputs{};
 std::array<float,6> q{},dq{},target{};
 std::string error;
};
inline ParsedArmStatus ParseArmStatus(const std::string& text,double home_tolerance,SafetyTime now) {
 ParsedArmStatus result;
 auto TimeNs=[](int64_t ns){return SafetyTime(std::chrono::nanoseconds(ns));};
 auto Age=[](SafetyTime a,SafetyTime b){return std::chrono::duration<double>(a-b).count();};
 auto Six=[](const YAML::Node& n){
  auto v=n.as<std::vector<float>>();
  if(v.size()!=6)throw std::runtime_error("Exactly six arm values required");
  std::array<float,6> a;std::copy(v.begin(),v.end(),a.begin());
  for(float x:a)if(!std::isfinite(x))throw std::runtime_error("Invalid arm values");
  return a;
 };
 try {
    const auto y=YAML::Load(text);
    auto q=Six(y["measured_q"]),dq=Six(y["measured_dq"]);
    auto ids=y["motor_ids"].as<std::vector<int>>();auto valid=y["valid"].as<std::vector<bool>>();
    if(ids.size()!=6||valid.size()!=6)throw std::runtime_error("arm identity length");
    for(int i=0;i<6;++i)if(ids[i]!=i+1||!valid[i])throw std::runtime_error("arm identity invalid");
    const auto received=TimeNs(y["frame_received_ns"].as<int64_t>());
    const double age=y["common_feedback_age_s"].as<double>();
    if(!std::isfinite(age)||age<0||Age(now,received)<0)throw std::runtime_error("invalid arm frame age");
    const auto observed=TimeNs(y["observed_ns"].as<int64_t>());
    if(Age(now,observed)<0)throw std::runtime_error("future arm observation");
    result.inputs.arm_stamp=observed-std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(age));
    result.inputs.ready.arm_feedback_ready=y["feedback_ready"].as<bool>();
    result.q=q;result.dq=dq;
    result.inputs.arm_emergency_validated=y["arm_emergency_validated"]&&y["arm_emergency_validated"].as<bool>();
    result.inputs.arm_static_hold=y["static_hold"].as<bool>();
    // New owner publishes all seven motors. Legacy six-only status cannot pass
    // the deployment HOME gate; gripper remains excluded from actor tensors.
    if(y["arm_home_ready"]&&y["arm_home_ready"].as<bool>()) {
     const auto ids7=y["motor_id"].as<std::vector<int>>();
     const auto states=y["motor_status"].as<std::vector<int>>();
     const auto valid7=y["valid7"].as<std::vector<bool>>();
     const auto q7=y["q"].as<std::vector<float>>(),dq7=y["dq"].as<std::vector<float>>();
     const auto target7=y["home_target"].as<std::vector<float>>();
     if(ids7.size()!=7||states.size()!=7||valid7.size()!=7||q7.size()!=7||dq7.size()!=7||target7.size()!=7)
      throw std::runtime_error("HOME requires seven motor diagnostics");
     const int home_joint_count=y["home_joint_count"]?y["home_joint_count"].as<int>():7;
     if(home_joint_count!=6&&home_joint_count!=7)throw std::runtime_error("invalid HOME joint scope");
     if(home_joint_count==6&&(!y["mission_capable"]||!y["mission_capable"].as<bool>()||
          !y["grasp_verification"]||y["grasp_verification"].as<std::string>()!="stub"))
      throw std::runtime_error("HOME6 requires explicit mission contract");
     const bool gripper_hold=y["gripper_hold_confirmed"]&&y["gripper_hold_confirmed"].as<bool>();
     const double grip_target=gripper_hold?y["gripper_hold_target"].as<double>():0.;
     if(!std::isfinite(grip_target))throw std::runtime_error("invalid gripper hold target");
     for(size_t i=0;i<7;++i)if(ids7[i]!=int(i+1)||!valid7[i]||states[i]!=1||!std::isfinite(q7[i])||(i<size_t(home_joint_count)&&std::abs(q7[i]-(i==6?grip_target:0.))>home_tolerance)||!std::isfinite(dq7[i])||target7[i]!=0)
      throw std::runtime_error("HOME motor diagnostics invalid");
     result.inputs.arm_home_ready=y["connected"].as<bool>()&&y["enabled_local"].as<bool>()&&
      !y["watchdog_tripped"].as<bool>()&&!y["stm32_watchdog_tripped"].as<bool>();
    }
    const bool owner_healthy=y["connected"].as<bool>()&&y["enabled_local"].as<bool>()&&
     !y["watchdog_tripped"].as<bool>()&&!y["stm32_watchdog_tripped"].as<bool>()&&
     y["owner_state"].as<std::string>()!="FAULT_LATCHED";
    const auto motor_states=y["motor_status"].as<std::vector<int>>();
    if(motor_states.size()!=7)throw std::runtime_error("seven motor control health values required");
    const bool motors_healthy=std::all_of(motor_states.begin(),motor_states.end(),[](int v){return v==1;});
    if(y["target_valid"].as<bool>()) {
     result.target=Six(y["accepted_target"]);
     if(result.inputs.arm_home_ready)for(float v:result.target)if(v!=0)throw std::runtime_error("HOME accepted target must be zero");
     result.inputs.target_stamp=TimeNs(y["accepted_ns"].as<int64_t>());
     result.inputs.ready.arm_target_ready=Age(now,result.inputs.target_stamp)>=0;
     result.inputs.arm_control_ready=owner_healthy&&motors_healthy&&result.inputs.ready.arm_feedback_ready&&result.inputs.ready.arm_target_ready;
    }
 }catch(const std::exception& e){result=ParsedArmStatus{};result.error=e.what();}
 return result;
}
// Call only under the controller lock. Parsing, validation and allocations are
// finished first. Invalid messages invalidate every arm gate atomically.
inline void ApplyArmStatus(const ParsedArmStatus& parsed,R3Inputs& inputs,
 std::array<float,6>& q,std::array<float,6>& dq,std::array<float,6>& target) {
 const auto& p=parsed.inputs;
 inputs.ready.arm_feedback_ready=p.ready.arm_feedback_ready;
 inputs.ready.arm_target_ready=p.ready.arm_target_ready;
 inputs.arm_static_hold=p.arm_static_hold;inputs.arm_home_ready=p.arm_home_ready;
 inputs.arm_control_ready=p.arm_control_ready;inputs.arm_emergency_validated=p.arm_emergency_validated;
 if(!parsed.error.empty())return;
 inputs.arm_stamp=p.arm_stamp;q=parsed.q;dq=parsed.dq;
 if(p.ready.arm_target_ready){inputs.target_stamp=p.target_stamp;target=parsed.target;}
}
} // namespace sim2real
