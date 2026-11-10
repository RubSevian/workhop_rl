#include "mission_health_freshness.hpp"
#include <cassert>
#include <limits>
int main(){
 // Replay the measured failure: only the Mission heartbeat was late.
 assert(!MissionSourcesFresh(.031247,.026371,.257844,.016,.25,.25,.1));
 assert(MissionSourcesFresh(.031247,.026371,.257844,.016,.25,.5,.1));
 assert(!MissionSourcesFresh(.251,.02,.1,.016,.25,.5,.1));
 assert(!MissionSourcesFresh(.02,.251,.1,.016,.25,.5,.1));
 assert(!MissionSourcesFresh(.02,.02,.501,.016,.25,.5,.1));
 assert(!MissionSourcesFresh(.02,.02,.1,.101,.25,.5,.1));
 assert(!MissionSourcesFresh(.02,.02,-1,.016,.25,.5,.1));
 assert(!MissionSourcesFresh(.02,.02,std::numeric_limits<double>::quiet_NaN(),.016,.25,.5,.1));
}
