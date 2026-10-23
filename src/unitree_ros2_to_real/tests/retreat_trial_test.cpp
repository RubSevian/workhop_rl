#include "retreat_trial.hpp"
#include <cassert>
#include <iostream>
using namespace sim2real;
int main(){
 RetreatTrialApproval g;assert(!g.Approve("before-wait",true));g.Stage("ownerA_plan1_epoch1");
 assert(!g.Approve("ownerB_plan1_epoch1",true));assert(!g.Approve("ownerA_plan1_epoch1",false));
 assert(!g.Consume("ownerA_plan1_epoch1",true));assert(g.Approve("ownerA_plan1_epoch1",true));
 assert(!g.Approve("ownerA_plan1_epoch1",true));assert(!g.Consume("ownerA_plan1_epoch1",false));
 assert(g.Consume("ownerA_plan1_epoch1",true));assert(!g.Consume("ownerA_plan1_epoch1",true));
 g.Stage("ownerA_plan2_epoch2");assert(!g.Approve("ownerA_plan1_epoch1",true));
 assert(g.Approve("ownerA_plan2_epoch2",true));g.Revoke();assert(!g.Consume("ownerA_plan2_epoch2",true));
 g.Stage("ownerC_plan1_epoch3");assert(!g.Approve("ownerA_plan2_epoch2",true));
 std::vector<std::array<float,7>> p(3);p[0].fill(0);p[1].fill(.04);p[2].fill(.08);
 auto slow=SlowRetreat(p,9);assert(slow.size()==9&&slow.front()==p.front()&&slow.back()==p.back());
 assert(slow[4]==p[1]);for(size_t i=1;i<slow.size();++i)for(int j=0;j<7;++j)assert(std::abs(slow[i][j]-slow[i-1][j])<=.01000001);
 for(size_t cap:{0,1,8}){bool failed=false;try{SlowRetreat(p,cap);}catch(const std::invalid_argument&){failed=true;}assert(failed);}
 p[1][0]=NAN;bool failed=false;try{SlowRetreat(p,20);}catch(const std::invalid_argument&){failed=true;}assert(failed);
 std::cout<<"PASS one-shot epoch/plan identity, live approval/consume, replay rejection, cancellation and same-polyline 4x slowdown within original cap\n";
}
