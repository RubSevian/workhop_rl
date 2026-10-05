#pragma once
#include "r3_commissioning.hpp"
#include <cassert>
using namespace sim2real;
inline SafetyTime time_at(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
inline R3Profile system_fixture(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);p.stand.fill(.2F);
 p.emergency_kd=std::array<float,12>{};p.emergency_kd->fill(3);p.emergency_evidence="OFFLINE_ONLY";
 p.gate0_verified=p.mapping_verified=p.robot_supported=p.remote_chords_verified=p.emergency_validated=p.policy_timing_reviewed=p.lie_down_validated=true;
 p.lie_down=std::array<float,12>{.01F,1.30F,-2.70F,-.01F,1.30F,-2.70F,-.30F,1.30F,-2.70F,.30F,1.30F,-2.70F};
 p.stand_s=6;p.hold_s=4;p.arm_home_timeout_s=.08;p.arm_home_settle_s=.02;p.lie_down_s=.10;p.lie_down_timeout_s=.20;p.lie_down_settle_s=.01;return p;
}
inline R3Inputs system_facts(double s,bool home=true){R3Inputs i;i.ready={true,true,true,true,true,true,true,true,false,true};
 i.measured_q.fill(.35F);i.arm_static_hold=i.arm_home_ready=home;i.arm_control_ready=i.own_output_healthy=true;
 i.sport=SportMode::RELEASED;i.sport_stamp=i.lowstate_stamp=i.remote_stamp=i.arm_stamp=i.target_stamp=time_at(s);return i;
}
inline void enter_active(R3Supervisor& f){auto i=system_facts(0);f.Observe(i,time_at(0));assert(f.Takeover(time_at(0)).success);f.Observe(i,time_at(0));assert(f.EnableOutput(true,time_at(0)).success);f.Tick(time_at(0));assert(f.RequestStand(time_at(0)).success);
 i=system_facts(6);f.Observe(i,time_at(6));assert(f.Tick(time_at(6)));assert(f.RequestRl(time_at(6)).success);assert(f.ConsumePolicyReset());
 auto work=f.BeginPolicy(time_at(6));assert(work);std::array<float,12> q;q.fill(.25F);assert(f.PolicyResult(*work,q,2,time_at(6)));assert(f.Tick(time_at(6)));
}
inline std::optional<unitree_go::msg::LowCmd> run_tick(R3Supervisor& f,double s,R3Inputs i){f.Observe(i,time_at(s));
 if(f.NeedsPolicy()){auto w=f.BeginPolicy(time_at(s));if(w){std::array<float,12> q;q.fill(.25F);assert(f.PolicyResult(*w,q,2,time_at(s)));}}
 return f.Tick(time_at(s));
}
