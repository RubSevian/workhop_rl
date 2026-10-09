#pragma once
#include "safety_io.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <string>
namespace sim2real {
// Input freshness/cancellation gate only; locomotion remains in R3Supervisor.
class NavigationRequestGate {
 public:
 NavigationRequestGate(std::array<double,3> bounds,double timeout=.25,double readiness_timeout=.5)
 :bounds_(bounds),timeout_(timeout),readiness_timeout_(readiness_timeout){}
 bool Observe(const std::string& session,bool ready,bool enabled,SafetyTime observed,SafetyTime now) {
  if(session.empty()||session.size()>128||!Fresh(now,observed,readiness_timeout_)||
     (seen_&&observed<=observed_)||
     (seen_&&session!=session_&&Fresh(now,observed_,readiness_timeout_))){Invalidate();return false;}
  if(session!=session_||!ready||!enabled||!ready_||!enabled_)valid_=false;
  session_=session;observed_=observed;ready_=ready;enabled_=enabled;seen_=true;return true;
 }
 void Invalidate(){ready_=enabled_=valid_=false;}
 bool Ready(SafetyTime now) const {return seen_&&ready_&&Fresh(now,observed_,readiness_timeout_);}
 bool Enabled(SafetyTime now) const {return Ready(now)&&enabled_;}
 bool Accept(const std::string& session,const std::array<double,3>& value,double header_age,SafetyTime observed,SafetyTime now) {
  valid_=false;
  if(session!=session_||observed!=observed_||!Enabled(now)||
     !std::isfinite(header_age)||header_age<0)return false;
  header_age+=std::chrono::duration<double>(now-observed).count();
  if(header_age>timeout_)return false;
  for(double v:value)if(!std::isfinite(v))return false;
  for(int i=0;i<3;++i)command_[i]=std::clamp(value[i],-bounds_[i],bounds_[i]);
  received_=now;header_age_=header_age;valid_=true;return true;
 }
 bool FreshCommand(SafetyTime now) const {
  const double age=std::chrono::duration<double>(now-received_).count();
  return valid_&&Enabled(now)&&age>=0&&age+header_age_<=timeout_;
 }
 std::array<double,3> Get(SafetyTime now) const {return FreshCommand(now)?command_:std::array<double,3>{};}
 const std::string& session() const {return session_;}
 private:
 static bool Fresh(SafetyTime now,SafetyTime stamp,double limit){double age=std::chrono::duration<double>(now-stamp).count();return age>=0&&age<=limit;}
 std::array<double,3> bounds_,command_{};double timeout_,readiness_timeout_,header_age_=0;
 bool seen_=false,ready_=false,enabled_=false,valid_=false;std::string session_;
 SafetyTime observed_{},received_{};
};
}
