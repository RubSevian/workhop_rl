#include "system_fsm_fixture.hpp"
#include <limits>
#include <iostream>
int main(){auto p=system_fixture();p.remote_stick_bounds=std::array<double,3>{.5,.5,.5};
 RemoteStatus remote;remote.remote_valid=true;remote.ly=1;remote.rx=-1;remote.lx=-1;
 assert((RemoteStickCommand(remote,p)==std::array<double,3>{.5,.5,.5}));
 remote.ly=-2;remote.rx=2;remote.lx=2;assert((RemoteStickCommand(remote,p)==std::array<double,3>{-.5,-.5,-.5}));
 remote.ly=remote.rx=remote.lx=0;assert((RemoteStickCommand(remote,p)==std::array<double,3>{}));
 R3Supervisor f(p,OperationProfile::REMOTE_TEST);enter_active(f);
 assert(f.RemoteTestCommand({.5,-.5,.5},time_at(6)).success);
 for(auto c:{std::array<double,3>{.501,0,0},{0,.501,0},{0,0,.501}})assert(!f.RemoteTestCommand(c,time_at(6)).success);
 assert(!f.NavigationCommand({.1,0,0},time_at(6)).success);
 assert(f.ControlledAbort(time_at(6)).success&&!f.RemoteTestCommand({.5,0,0},time_at(6)).success);
 R3Supervisor zero(p,OperationProfile::RL_ZERO_TEST);enter_active(zero);assert(!zero.RemoteTestCommand({.5,0,0},time_at(6)).success);
 auto fact=system_facts(0);fact.navigation_ready=true;R3Supervisor nav(p,OperationProfile::NAV_TEST);nav.Observe(fact,time_at(0));
 assert(nav.Takeover(time_at(0)).success);nav.Observe(fact,time_at(0));assert(nav.EnableOutput(true,time_at(0)).success);nav.Tick(time_at(0));assert(nav.RequestStand(time_at(0)).success);
 fact=system_facts(6);fact.navigation_ready=true;nav.Observe(fact,time_at(6));nav.Tick(time_at(6));assert(nav.RequestRl(time_at(6)).success);nav.ConsumePolicyReset();auto job=nav.BeginPolicy(time_at(6));assert(job&&nav.PolicyResult(*job,p.stand,2,time_at(6)));
 assert(nav.NavigationCommand({.2,.1,.1},time_at(6)).success&&!nav.NavigationCommand({.5,0,0},time_at(6)).success);
 p.remote_stick_bounds=std::array<double,3>{.51,.5,.5};try{R3Supervisor bad(p);assert(false);}catch(const std::invalid_argument&){}
 std::cout<<"PASS remote +/-0.5 sticks/clamp/deadband/profile/X gates; NAV +/-0.2/0.1/0.1 unchanged\n";
}
