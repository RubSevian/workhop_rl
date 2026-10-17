#include "arm_status_snapshot.hpp"
#include <cassert>
#include <iostream>
using namespace sim2real;
static std::string arm_json(int64_t stamp) {
 const auto s=std::to_string(stamp);
 return "{\"synthetic_replay\":true,\"observed_ns\":"+s+",\"frame_received_ns\":"+s+",\"accepted_ns\":"+s+
 ",\"common_feedback_age_s\":0,\"feedback_ready\":true,\"measured_q\":[0,0,0,0,0,0],\"measured_dq\":[0,0,0,0,0,0],\"motor_ids\":[1,2,3,4,5,6],\"valid\":[true,true,true,true,true,true],\"static_hold\":true,\"arm_emergency_validated\":false,\"arm_home_ready\":true,\"motor_id\":[1,2,3,4,5,6,7],\"motor_status\":[1,1,1,1,1,1,1],\"valid7\":[true,true,true,true,true,true,true],\"q\":[0,0,0,0,0,0,0],\"dq\":[0,0,0,0,0,0,0],\"home_target\":[0,0,0,0,0,0,0],\"connected\":true,\"enabled_local\":true,\"watchdog_tripped\":false,\"stm32_watchdog_tripped\":false,\"owner_state\":\"HOLD_HOME\",\"target_valid\":true,\"accepted_target\":[0,0,0,0,0,0]}";
}
int main(){
 const auto now=SafetyTime(std::chrono::seconds(2));
 const auto valid=YAML::Load(arm_json(1000000000));
 auto parse=[&](const YAML::Node& n){return ParseArmStatus(YAML::Dump(n),.15,now);};
 auto good=parse(valid);assert(good.error.empty()&&good.inputs.arm_home_ready&&good.inputs.arm_control_ready);
 R3Inputs inputs;std::array<float,6> q{},dq{},target{};
 ApplyArmStatus(good,inputs,q,dq,target);assert(inputs.arm_home_ready&&inputs.ready.arm_target_ready);
 assert(inputs.arm_stamp==SafetyTime(std::chrono::seconds(1)));
 for(const char* key:{"connected","enabled_local","feedback_ready","target_valid"}){
  auto n=YAML::Clone(valid);n[key]=false;auto p=parse(n);assert(p.error.empty()&&!p.inputs.arm_control_ready);
 }
 for(const char* key:{"watchdog_tripped","stm32_watchdog_tripped"}){
  auto n=YAML::Clone(valid);n[key]=true;auto p=parse(n);assert(p.error.empty()&&!p.inputs.arm_home_ready&&!p.inputs.arm_control_ready);
 }
 auto bad=[&](YAML::Node n){auto p=parse(n);assert(!p.error.empty());
  ApplyArmStatus(good,inputs,q,dq,target);ApplyArmStatus(p,inputs,q,dq,target);
  assert(!inputs.arm_home_ready&&!inputs.arm_control_ready&&!inputs.ready.arm_feedback_ready&&!inputs.ready.arm_target_ready&&!inputs.arm_emergency_validated);
 };
 {auto n=YAML::Clone(valid);n["motor_ids"][0]=2;bad(n);}
 {auto n=YAML::Clone(valid);n["valid7"][6]=false;bad(n);}
 {auto n=YAML::Clone(valid);n["motor_status"][6]=0;bad(n);}
 {auto n=YAML::Clone(valid);n["q"][6]=.16;bad(n);}
 {auto n=YAML::Clone(valid);n["home_target"][6]=.01;bad(n);}
 {auto n=YAML::Clone(valid);n["accepted_target"][0]=.01;bad(n);}
 {auto n=YAML::Clone(valid);n["measured_dq"][0]=".nan";bad(n);}
 for(const char* key:{"observed_ns","frame_received_ns"}){auto n=YAML::Clone(valid);n[key]=int64_t(3000000000);bad(n);}
 {auto n=YAML::Clone(valid);n["accepted_ns"]=int64_t(3000000000);auto p=parse(n);assert(!p.inputs.ready.arm_target_ready&&!p.inputs.arm_control_ready);}
 {auto n=YAML::Clone(valid);n["common_feedback_age_s"]=-1;bad(n);}
 {auto n=YAML::Clone(valid);n.remove("motor_status");bad(n);}
 assert(!ParseArmStatus("not: [valid",.15,now).error.empty());
 std::cout<<"PASS arm snapshot: health, seven motors, targets, invalidation, timestamps preserved\n";
}
