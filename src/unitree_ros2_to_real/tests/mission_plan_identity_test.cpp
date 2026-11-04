#include "mission_plan_identity.hpp"
#include <cassert>
int main(){
 const std::string prefix="{\"request_identity\":{\"id\":9223372036854775819,\"session_id\":\"mission\",\"owner_session_id\":\"owner\",\"navigation_session_id\":\"nav\",\"goal_revision\":4}},";
 // Build the real outer object prefix (one open outer brace).
 const auto header=prefix.substr(0,prefix.size()-2)+",\n";
 const auto identity=sim2real::PlanWireIdentity(header+std::string(100000,'x'));
 assert(identity&&identity->id==9223372036854775819ULL&&identity->session=="mission");
 auto other=*identity;assert(identity->Matches(other));other.session="old";assert(!identity->Matches(other));
 other=*identity;other.id++;assert(!identity->Matches(other));
 assert(!sim2real::PlanWireIdentity("broken"));
 assert(!sim2real::PlanWireIdentity(std::string(2049,' ')+header));
 assert(!sim2real::PlanWireIdentity("{\"request_identity\":{},\n"));

 const std::string body="\"id\":9223372036854775819,\"session_id\":\"mission\",\"owner_session_id\":\"owner\",\"navigation_session_id\":\"nav\",\"goal_revision\":4,\"points\":[[0,0,0,0,0,0,0],[1,0,0,0,0,0,0]]}";
 const auto parsed=sim2real::ParseMissionPlan(header+body,2,identity);
 assert(parsed.error.empty()&&parsed.points.size()==2&&parsed.identity->Matches(*identity));
 const auto capped=sim2real::ParseMissionPlan(header+body,1,identity);
 assert(capped.error=="plan sample cap exceeded"&&capped.identity->Matches(*identity));
 const auto malformed=sim2real::ParseMissionPlan(header+"broken body",2,identity);
 assert(!malformed.error.empty()&&malformed.identity->Matches(*identity));
 auto wrong=*identity;wrong.id++;
 const auto mismatch=sim2real::ParseMissionPlan(header+body,2,wrong);
 assert(mismatch.error=="PLAN_IDENTITY_MISMATCH"&&mismatch.identity->id==wrong.id);
}
