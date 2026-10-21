// Approved X failed-arm sequence. Synthetic clock/packets only; no DDS or SDK IO.
#include "system_fsm_fixture.hpp"
#include <iostream>
#include <set>
R3Profile profile(){auto p=system_fixture();p.failed_arm_pd_stop_enabled=true;p.lie_down_s=8;p.lie_down_timeout_s=12;p.arm_home_timeout_s=10;p.lie_down_settle_s=.2;return p;}
R3Inputs facts(double s,bool arm=true){auto i=system_facts(s,false);i.navigation_ready=i.perception_ready=true;
 if(!arm){i.ready.arm_feedback_ready=i.ready.arm_target_ready=i.arm_control_ready=false;i.arm_stamp=i.target_stamp=time_at(1);}
 return i;
}
void active(R3Supervisor& f){auto i=facts(100);i.arm_home_ready=i.arm_static_hold=true;f.Observe(i,time_at(100));assert(f.Takeover(time_at(100)).success);f.Observe(i,time_at(100));assert(f.EnableOutput(true,time_at(100)).success);assert(f.Tick(time_at(100)));assert(f.RequestStand(time_at(100)).success);
 i=facts(106);i.arm_home_ready=i.arm_static_hold=true;f.Observe(i,time_at(106));assert(f.Tick(time_at(106)));assert(f.RequestRl(time_at(106)).success);assert(f.ConsumePolicyReset());auto w=f.BeginPolicy(time_at(106));assert(w);std::array<float,12> q;q.fill(.25F);assert(f.PolicyResult(*w,q,2,time_at(106)));assert(f.Tick(time_at(106)));
}
std::optional<unitree_go::msg::LowCmd> step(R3Supervisor& f,double t,R3Inputs i,bool publish=true){f.Observe(i,time_at(t));if(f.NeedsPolicy()&&!f.fault_latched()){auto w=f.BeginPolicy(time_at(t));if(w){std::array<float,12> q;q.fill(.25F);assert(f.PolicyResult(*w,q,2,time_at(t)));}}
 auto packet=f.Tick(time_at(t));if(packet&&publish){if(!f.AllowsPacket(*packet,time_at(t)))std::cerr<<"packet rejection t="<<t<<" phase="<<SystemPhaseName(f.phase())<<" fault="<<f.last_fault()<<" q0="<<packet->motor_cmd[0].q<<" target="<<f.fixed_target()[0]<<" kp="<<packet->motor_cmd[0].kp<<" policy="<<f.NeedsPolicy()<<"\n";assert(f.AllowsPacket(*packet,time_at(t)));f.NotifyPacketPublished(*packet,time_at(t));}return packet;
}
void cancel(R3Supervisor& f){active(f);assert(f.ControlledAbort(time_at(106)).success);f.ArmHomeRequestAccepted(false,time_at(106));assert(f.phase()==SystemPhase::CANCEL_TASKS);auto ports=f.ConsumePortRequests();assert(ports.cancel_navigation&&ports.cancel_manipulation&&!ports.arm_return_home&&!ports.arm_planned_disable);}
void capture(R3Supervisor& f,bool publish=true){cancel(f);auto packet=step(f,106.002,facts(106.002),publish);assert(packet&&f.phase()==SystemPhase::PD_CAPTURE&&!f.NeedsPolicy());for(size_t j=0;j<12;++j){assert(packet->motor_cmd[j].q==.35F);assert(packet->motor_cmd[j].kp==40&&packet->motor_cmd[j].kd==1);}assert(!f.ConsumePortRequests().arm_planned_disable);}
void disable(R3Supervisor& f){capture(f);step(f,106.004,facts(106.004));assert(f.phase()==SystemPhase::PD_HOLD_CONFIRMED&&f.pd_hold_confirmed());assert(!f.ConsumePortRequests().arm_planned_disable);step(f,106.006,facts(106.006));assert(f.phase()==SystemPhase::ARM_DISABLE);assert(f.ConsumePortRequests().arm_planned_disable);assert(!f.ConsumePortRequests().arm_planned_disable);}
void success(){auto p=profile();R3Supervisor f(p);disable(f);f.ArmPlannedDisableResult(true,time_at(106.008));assert(step(f,106.008,facts(106.008,false)));assert(f.phase()==SystemPhase::LIE_DOWN);const auto start=f.fixed_target();
 for(int n=1;n<=800;++n){const double t=106.008+n*.01;auto i=facts(t,false);float u=float(n)/800;for(size_t j=0;j<12;++j)i.measured_q[j]=start[j]*(1-u)+(*p.lie_down)[j]*u;auto packet=step(f,t,i);assert(packet&&!f.fault_latched());for(size_t j=0;j<12;++j)assert(std::abs(packet->motor_cmd[j].q-i.measured_q[j])<1e-4);if(n<800)assert(f.phase()==SystemPhase::LIE_DOWN);}
 assert(f.phase()==SystemPhase::LIE_DOWN_VERIFY);auto i=facts(114.210,false);i.measured_q=*p.lie_down;auto packet=step(f,114.210,i);assert(packet&&f.phase()==SystemPhase::LIE_DOWN_PASSIVE&&packet->motor_cmd[0].mode==0);
 for(int n=1;n<=10;++n){double t=114.210+n*.002;i=facts(t,false);i.measured_q=*p.lie_down;step(f,t,i);}
 assert(!f.output_enabled()&&f.passive_sequence_complete());f.ConfirmOutputStopped();assert(f.system_state()==SystemState::SYSTEM_HOLD&&!f.fault_latched());
 auto disabled=facts(115,false);f.Observe(disabled,time_at(115));assert(!f.StartRemoteSequence(time_at(115)).success&&!f.fault_latched());assert(!f.EnableOutput(true,time_at(115)).success);
}
void failures(){
 for(int kind=0;kind<2;++kind){R3Supervisor f(profile());capture(f,kind==1);auto i=facts(106.010);if(kind==1)i.measured_q.fill(.8F);step(f,106.010,i);assert(f.phase()==SystemPhase::PD_CAPTURE&&!f.pd_hold_confirmed());i=facts(106.044);step(f,106.044,i);assert(f.fault_latched()&&!f.ConsumePortRequests().arm_planned_disable);}
 for(bool timeout:{false,true}){R3Supervisor f(profile());disable(f);if(!timeout)f.ArmPlannedDisableResult(false,time_at(106.008));else step(f,116.008,facts(116.008,false));assert(f.phase()==SystemPhase::ARM_DISABLE_BLOCKED);assert(!f.fault_latched());f.ArmPlannedDisableResult(true,time_at(116.010));auto packet=step(f,116.010,facts(116.010,false));assert(packet&&packet->motor_cmd[0].q==.35F&&!f.NeedsPolicy());}
 for(bool unapproved_target:{false,true}){auto p=profile();p.lie_down_validated=false;if(unapproved_target){p.lie_down_validated=true;(*p.lie_down)[0]+=.1F;}R3Supervisor f(p);disable(f);f.ArmPlannedDisableResult(true,time_at(106.008));step(f,106.008,facts(106.008,false));assert(f.phase()==SystemPhase::LIE_DOWN_BLOCKED&&!f.fault_latched());assert(f.fixed_target()[0]==.35F);}
 R3Supervisor timeout(profile());disable(timeout);timeout.ArmPlannedDisableResult(true,time_at(106.008));step(timeout,106.008,facts(106.008,false));step(timeout,118.010,facts(118.010,false));assert(timeout.phase()==SystemPhase::LIE_DOWN_BLOCKED&&timeout.stop_blocker()=="lie_down_timeout_holding_last_planned_target");auto i=facts(118.012,false);i.measured_q=*profile().lie_down;step(timeout,118.012,i);assert(timeout.phase()==SystemPhase::LIE_DOWN_BLOCKED&&!timeout.passive_sequence_complete());
}
void watchdogs(){
 for(int kind=0;kind<5;++kind){R3Supervisor f(profile());active(f);double t=106.;if(kind==0){f.Fault("prior_fault",time_at(t));}if(kind==1)t+=.041;if(kind==2){auto i=facts(t);i.lowstate_stamp=time_at(t-.041);f.Observe(i,time_at(t));}if(kind==3){auto i=facts(t);i.ready.arm_feedback_ready=false;f.Observe(i,time_at(t));}if(kind==4){auto w=f.BeginPolicy(time_at(t));assert(w);t+=.041;}
  const auto reply=f.ControlledAbort(time_at(t));if(reply.success)f.ArmHomeRequestAccepted(false,time_at(t));assert(f.fault_latched());assert(!f.failed_arm_stop());assert(!f.ConsumePortRequests().arm_planned_disable);}
 for(int kind=0;kind<4;++kind){R3Supervisor f(profile());disable(f);auto i=facts(106.008,false);if(kind==0)i.lowstate_stamp=time_at(105);if(kind==1)i.ready.motor_state_valid=false;if(kind==2)i.remote_stamp=time_at(105);if(kind==3)i.arm_fault_confirmed=true;step(f,106.008,i);assert(f.fault_latched());}
}
void priority(){for(auto phase:{SystemPhase::CANCEL_TASKS,SystemPhase::PD_CAPTURE,SystemPhase::PD_HOLD_CONFIRMED,SystemPhase::ARM_DISABLE,SystemPhase::ARM_DISABLE_BLOCKED,SystemPhase::LIE_DOWN_BLOCKED,SystemPhase::LIE_DOWN,SystemPhase::LIE_DOWN_VERIFY,SystemPhase::LIE_DOWN_PASSIVE}){R3Supervisor f(profile());disable(f);f.Dispatch(SystemEvent::ADVANCE_PHASE,time_at(106.006),phase);const std::array<SystemEvent,3> events{SystemEvent::REQUEST_X,SystemEvent::REQUEST_A,SystemEvent::REQUEST_B};assert(f.DispatchEvents(events,time_at(106.006)).success);assert(f.fault_latched()&&f.system_state()==SystemState::EMERGENCY_FAULT);auto ports=f.ConsumePortRequests();assert(ports.arm_emergency&&!ports.arm_planned_disable&&!ports.arm_return_home);f.ArmPlannedDisableResult(true,time_at(106.008));assert(f.fault_latched());auto packet=f.Tick(time_at(106.008));assert(packet);for(int j=0;j<12;++j)assert(packet->motor_cmd[j].kp==0&&packet->motor_cmd[j].kd==3);}}
void timeout_trigger(){R3Supervisor f(profile());active(f);f.ControlledAbort(time_at(106));for(int n=1;n<=1001;++n){double t=106+n*.01;step(f,t,facts(t));if(f.phase()==SystemPhase::CANCEL_TASKS)break;}assert(f.phase()==SystemPhase::CANCEL_TASKS);}
void pd_lift_boundary(){
 R3Supervisor f(profile());auto i=facts(100);i.arm_home_ready=i.arm_static_hold=true;f.Observe(i,time_at(100));assert(f.StartRemoteSequence(time_at(100)).success);
 // SDK release is represented only by an offline supervisor observation.
 f.Observe(i,time_at(100));assert(f.EnableOutput(true,time_at(100)).success);f.Tick(time_at(100));assert(f.RequestStand(time_at(100)).success);
 i=facts(106,false);f.Observe(i,time_at(106));assert(f.Tick(time_at(106)));assert(f.state()==R3State::HOLDING&&!f.fault_latched());
 assert(f.RemoteSequenceNext(time_at(106))==R3SequenceAction::NONE);
 i=facts(110.002,false);f.Observe(i,time_at(110.002));assert(f.RemoteSequenceNext(time_at(110.002))==R3SequenceAction::NONE);assert(f.phase()==SystemPhase::CANCEL_TASKS&&!f.NeedsPolicy()&&!f.fault_latched());
 step(f,110.004,facts(110.004,false));assert(f.phase()==SystemPhase::PD_CAPTURE);
 step(f,110.006,facts(110.006,false));assert(f.phase()==SystemPhase::PD_HOLD_CONFIRMED);
 step(f,110.008,facts(110.008,false));assert(f.phase()==SystemPhase::ARM_DISABLE&&f.ConsumePortRequests().arm_planned_disable);
}
int main(){pd_lift_boundary();success();failures();watchdogs();priority();timeout_trigger();std::cout<<"PASS X failed-arm: ordered cancel/PD publish+feedback confirmation/planned-disable/8s descent/verify/passive; 12s timeout, qualification gate, stale/fault rejection, B priority; synthetic only\n";}
