#include "real_controller_core.hpp"
#include <cassert>
#include <iostream>
#include <ATen/Parallel.h>
using namespace sim2real;
int main(int argc,char** argv){
 assert(argc==3);at::set_num_threads(1);at::set_num_interop_threads(1);
 RealControllerCore core;core.Load(argv[1],argv[2]);assert(core.safety().state()==SafetyState::DISARMED);
 Legs measured{};measured.fill(.2F);core.SetMeasuredLegs(measured,{});
 MockRarsTransport arm(rars_arm::ArmConfiguration{});RarsBridge bridge(arm);
 assert(!core.SetArmFromBridge(bridge,SafetyTime{}));
 RarsFrame frame;frame.communication.connected=true;frame.communication.feedback_received=true;
 for(int i=0;i<7;++i){frame.joints.position[i]=.1F*(i+1);frame.joints.velocity[i]=.2F*(i+1);frame.joints.valid[i]=true;frame.joints.motor_id[i]=i+1;}
 arm.frames.push_back(frame);AcceptedArmTarget accepted;accepted.valid=true;accepted.q.fill(.3F);arm.target=accepted;
 assert(core.SetArmFromBridge(bridge,SafetyTime{}));
 assert(core.agent().obs.arm_pos[5].item<float>()==frame.joints.position[5]);
 assert(core.agent().obs.arm_target[5].item<float>()==.3F);
 assert(!core.SetArmFromBridge(bridge,SafetyTime{}+std::chrono::seconds(1)));
 Readiness r{true,true,true,true};
 core.agent().params.rl_kp=torch::arange(12)+20;core.agent().params.rl_kd=torch::arange(12)*.125F+1;
 auto verify=[&](float time){auto target=core.Tick(time);assert(target);auto cmd=MakeLowCmd(target->q,target->kp,target->kd);
  for(int i=0;i<12;++i){assert(cmd.motor_cmd[i].q==target->q[i]);assert(cmd.motor_cmd[i].kp==target->kp[i]);assert(cmd.motor_cmd[i].kd==target->kd[i]);assert(cmd.motor_cmd[i].tau==0);}
  auto b=SerializeLowCmd(cmd);assert(Go2Crc(std::span(b).first(808))==cmd.crc);
 };
 assert(core.RequestMode(Mode::STAND,r));verify(0);verify(4);verify(8);assert(core.mode()==Mode::HOLD);verify(9);
 assert(core.RequestMode(Mode::RL,r));verify(0);verify(.02);
 assert(core.RequestMode(Mode::HOLD_TRANSITION,r));verify(0);verify(.5);verify(1);
 // Offline calculations and cmd_vel never arm the separate FSM.
 assert(core.safety().state()==SafetyState::DISARMED);
 MockActuatorTransport transport;SafetyReadiness live;
 assert(!core.SendTargets(transport,live,SafetyTime{},0));assert(transport.sent.empty());
 SafetyReadiness synthetic{true,true,true,true,true,true,true,true,true,true};
 core.safety().ObserveSportMode(SportMode::RELEASED,SafetyTime{});core.safety().TakeoverRequest();
 for(int i=0;i<4;++i)core.safety().Update(synthetic,SafetyTime{});
 assert(core.SendTargets(transport,synthetic,SafetyTime{},0));assert(transport.sent.size()==1);
 auto stale=synthetic;stale.arm_feedback_ready=false;
 assert(!core.SendTargets(transport,stale,SafetyTime{},0));assert(transport.sent.size()==1);
 assert(core.RequestMode(Mode::DISARMED,{}));assert(core.safety().state()==SafetyState::DISARMED);
 assert(!core.SendTargets(transport,synthetic,SafetyTime{},0));assert(transport.sent.size()==1);
 core.RequestMode(Mode::FAULT,{});assert(core.safety().state()==SafetyState::FAULT_LATCHED);
 std::cout<<"PASS RealControllerCore stand/hold/RL/hold transition -> deterministic mapped offline LowCmd; gated dispatch uses mock transport only\n";
}
