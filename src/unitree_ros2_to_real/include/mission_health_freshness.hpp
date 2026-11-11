#pragma once
#include <cmath>
// Independent NAV lease; R3 and actuator feedback retain their strict budgets.
inline bool MissionSourcesFresh(double r3_age,double nav_age,double health_age,
                                double feedback_age,double source_ttl,double health_ttl,double feedback_ttl,
                                double nav_ttl = -1.){
 const auto fresh=[](double age,double ttl){return std::isfinite(age)&&age>=0&&age<=ttl;};
 return fresh(r3_age,source_ttl)&&fresh(nav_age,nav_ttl < 0 ? source_ttl : nav_ttl)&&
        fresh(health_age,health_ttl)&&fresh(feedback_age,feedback_ttl);
}
