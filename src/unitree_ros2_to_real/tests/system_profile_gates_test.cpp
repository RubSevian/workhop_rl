#include "r3_commissioning.hpp"
#include <cassert>
using namespace sim2real;
R3Profile fixture(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);p.emergency_kd=std::array<float,12>{};p.emergency_kd->fill(3);p.emergency_evidence="mock";p.gate0_verified=p.mapping_verified=p.robot_supported=p.remote_chords_verified=p.emergency_validated=p.policy_timing_reviewed=true;return p;}
R3Inputs facts(){R3Inputs i;i.ready={true,true,true,true,true,true,true,true,false,true};i.arm_static_hold=i.arm_home_ready=i.arm_control_ready=true;i.sport=SportMode::RELEASED;return i;}
int main(){for(auto op:{OperationProfile::READ_ONLY,OperationProfile::ARM_TEST,OperationProfile::LEG_SAFETY_TEST,OperationProfile::RL_ZERO_TEST,OperationProfile::REMOTE_TEST,OperationProfile::NAV_TEST,OperationProfile::FULL_MISSION}){
 auto p=fixture();R3Supervisor s(p,op);auto i=facts();s.Observe(i,{});auto a=s.StartRemoteSequence({});
 const auto c=s.capabilities();assert(a.success==(c.physical_leg_output&&!c.require_navigation_ready));
 s.Takeover({});s.Observe(i,{});auto output=s.EnableOutput(true,{});
 assert(output.success==(c.physical_leg_output&&!c.require_navigation_ready));
 if(c.physical_leg_output&&!output.success){i.navigation_ready=i.perception_ready=i.arm_emergency_validated=true;s.Observe(i,{});output=s.EnableOutput(true,{});assert(output.success);}
 if(output.success){s.Tick({});assert(s.RequestStand({}).success==c.allow_rl);}
 assert(!s.RequestRl({}).success); // no service skips stand/hold
 assert(!s.RemoteTestCommand({.1,0,0},{}).success);
 assert(!s.NavigationCommand({.1,0,0},{}).success);
 if(op==OperationProfile::LEG_SAFETY_TEST){assert(s.RemoteSequenceNext({})==R3SequenceAction::NONE);assert(!s.RequestStand({}).success);}
 }
 // Even a correctly staged ACTIVE session cannot bypass source/zero capabilities.
 for(auto op:{OperationProfile::RL_ZERO_TEST,OperationProfile::REMOTE_TEST,OperationProfile::NAV_TEST,OperationProfile::FULL_MISSION}){
  auto p=fixture();R3Supervisor active(p,op);auto i=facts();i.navigation_ready=i.perception_ready=i.arm_emergency_validated=true;
  active.Observe(i,{});assert(active.Takeover({}).success);active.Observe(i,{});assert(active.Dispatch(SystemEvent::ENABLE_OUTPUT,{}).success);
  active.Tick({});assert(active.Dispatch(SystemEvent::REQUEST_STAND,{}).success);
  const auto done=SafetyTime{}+std::chrono::seconds(8);i.sport_stamp=i.lowstate_stamp=i.remote_stamp=i.arm_stamp=i.target_stamp=done;active.Observe(i,done);active.Tick(done);
  assert(active.Dispatch(SystemEvent::REQUEST_RL,done).success&&active.ConsumePolicyReset());
  auto work=active.BeginPolicy(done);assert(work&&active.PolicyResult(*work,p.stand,2,done));
  assert(active.RemoteTestCommand({.1,0,0},done).success==(op==OperationProfile::REMOTE_TEST));
  assert(active.NavigationCommand({.1,0,0},done).success==(op==OperationProfile::NAV_TEST||op==OperationProfile::FULL_MISSION));
  if(op==OperationProfile::RL_ZERO_TEST){assert(!active.ManualCommand({.1,0,0},.1,done).success);assert((active.command()==std::array<double,3>{}));}
 }
 R3Supervisor nav(fixture(),OperationProfile::NAV_TEST);auto i=facts();i.navigation_ready=true;nav.Observe(i,{});assert(nav.StartRemoteSequence({}).success);
}
