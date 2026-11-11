#include "arm_fast_snapshot.hpp"
#include <cassert>
#include <functional>
#include <limits>
using namespace sim2real;
using Msg=rars01_state_interfaces::msg::ArmFastState;
YAML::Node yaml(const Msg& m){YAML::Node y;
 y["observed_ns"] = m.observed_ns;
 y["frame_received_ns"] = m.frame_received_ns;
 y["accepted_ns"] = m.accepted_ns;
 y["common_feedback_age_s"] = m.common_feedback_age_s;
 y["feedback_ready"] = m.feedback_ready;
 y["arm_emergency_validated"] = m.arm_emergency_validated;
 y["static_hold"] = m.static_hold;
 y["arm_home_ready"] = m.arm_home_ready;
 y["home_joint_count"] = m.home_joint_count;
 y["mission_capable"] = m.mission_capable;
 y["grasp_verification"] = m.grasp_verification;
 y["gripper_hold_confirmed"] = m.gripper_hold_confirmed;
 y["gripper_hold_target"] = m.gripper_hold_target;
 y["connected"] = m.connected;
 y["enabled_local"] = m.enabled_local;
 y["watchdog_tripped"] = m.watchdog_tripped;
 y["stm32_watchdog_tripped"] = m.stm32_watchdog_tripped;
 y["owner_state"] = m.owner_state;
 y["target_valid"] = m.target_valid;
 for(auto x:m.measured_q)y["measured_q"].push_back(x);
 for(auto x:m.measured_dq)y["measured_dq"].push_back(x);
 for(auto x:m.motor_ids)y["motor_ids"].push_back(x);
 for(auto x:m.valid)y["valid"].push_back(x);
 for(auto x:m.motor_id)y["motor_id"].push_back(x);
 for(auto x:m.motor_status)y["motor_status"].push_back(x);
 for(auto x:m.valid7)y["valid7"].push_back(x);
 for(auto x:m.q)y["q"].push_back(x);
 for(auto x:m.dq)y["dq"].push_back(x);
 for(auto x:m.home_target)y["home_target"].push_back(x);
 for(auto x:m.accepted_target)y["accepted_target"].push_back(x);
 if(!m.accepted_target_valid){y.remove("accepted_target");y.remove("accepted_ns");}return y;}
void compare(const Msg& m){
 auto now=SafetyTime(std::chrono::seconds(2));
 auto a=ParseArmStatus(YAML::Dump(yaml(m)),.15,now),b=ParseArmFastStatus(m,.15,now);
 assert(a.error.empty()==b.error.empty());
 assert(a.q==b.q&&a.dq==b.dq&&a.target==b.target);
 const auto &x=a.inputs,&y=b.inputs;
 assert(x.arm_stamp==y.arm_stamp&&x.target_stamp==y.target_stamp);
 assert(x.arm_home_ready==y.arm_home_ready&&x.arm_control_ready==y.arm_control_ready);
 assert(x.arm_fault_confirmed==y.arm_fault_confirmed&&x.arm_static_hold==y.arm_static_hold);
 assert(x.arm_emergency_validated==y.arm_emergency_validated);
 assert(x.ready.arm_feedback_ready==y.ready.arm_feedback_ready&&x.ready.arm_target_ready==y.ready.arm_target_ready);
}
int main(){
 Msg good;good.observed_ns=good.frame_received_ns=good.accepted_ns=1000000000;
 good.feedback_ready=good.arm_home_ready=good.connected=good.enabled_local=good.target_valid=good.accepted_target_valid=true;
 good.owner_state="HOLD_HOME";good.home_joint_count=7;
 for(int i=0;i<6;++i){good.motor_ids[i]=i+1;good.valid[i]=true;}
 for(int i=0;i<7;++i){good.motor_id[i]=i+1;good.valid7[i]=true;good.motor_status[i]=1;}
 compare(good);
 for(auto mutate:std::initializer_list<std::function<void(Msg&)>>{
  [](auto& m){m.connected=false;},[](auto& m){m.enabled_local=false;},
  [](auto& m){m.feedback_ready=false;},[](auto& m){m.target_valid=false;},
  [](auto& m){m.watchdog_tripped=true;},[](auto& m){m.stm32_watchdog_tripped=true;},
  [](auto& m){m.owner_state="FAULT_LATCHED";},[](auto& m){m.motor_ids[0]=9;},
  [](auto& m){m.valid7[6]=false;},[](auto& m){m.motor_status[6]=0;},
  [](auto& m){m.home_target[6]=.01;},[](auto& m){m.common_feedback_age_s=-1;},
  [](auto& m){m.observed_ns=3000000000;},[](auto& m){m.frame_received_ns=3000000000;},
  [](auto& m){m.accepted_ns=3000000000;},[](auto& m){m.accepted_target_valid=false;},
  [](auto& m){m.measured_q[0]=std::numeric_limits<double>::quiet_NaN();},
  [](auto& m){m.measured_dq[1]=std::numeric_limits<double>::infinity();},
  [](auto& m){m.home_joint_count=6;},
  [](auto& m){m.home_joint_count=6;m.mission_capable=true;m.grasp_verification="stub";},
  [](auto& m){m.gripper_hold_confirmed=true;m.gripper_hold_target=.1;m.q[6]=.1;}
 }){auto m=good;mutate(m);compare(m);}
 for(int i=0;i<7;++i)for(double q:{-.150001,-.149999,0.,.149999,.150001}){
  auto m=good;m.q[i]=q;if(i<6)m.accepted_target[i]=q;compare(m);
 }
 auto bad=good;bad.schema_version=99;assert(!ParseArmFastStatus(bad,.15,SafetyTime(std::chrono::seconds(2))).error.empty());
 bad=good;bad.observed_ns=UINT64_MAX;assert(!ParseArmFastStatus(bad,.15,SafetyTime(std::chrono::seconds(2))).error.empty());
}
