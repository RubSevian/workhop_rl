#pragma once
#include <cmath>
// Only the Mission lease gets its own budget. R3/NAV and feedback stay strict.
inline bool MissionSourcesFresh(double r3_age,double nav_age,double health_age,
                                double feedback_age,double source_ttl,double health_ttl,double feedback_ttl){
 const auto fresh=[](double age,double ttl){return std::isfinite(age)&&age>=0&&age<=ttl;};
 return fresh(r3_age,source_ttl)&&fresh(nav_age,source_ttl)&&
        fresh(health_age,health_ttl)&&fresh(feedback_age,feedback_ttl);
}
