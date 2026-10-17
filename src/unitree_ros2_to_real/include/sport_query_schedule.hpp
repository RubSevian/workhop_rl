#pragma once
#include "safety_io.hpp"

namespace sim2real {
// Process-local STATUS plan; RELEASE confirmation is a separate pre-output
// operation. Failed startup STATUS never silently retries in startup-only mode.
class SportQuerySchedule {
 public:
 SportQuerySchedule(bool startup_only,int period_ms):startup_only_(startup_only),period_(period_ms){}
 bool Due(SafetyTime now,bool busy) const {return !busy&&(!submitted_||(!startup_only_&&now>=next_));}
 void Submitted(){submitted_=true;}
 void Completed(SafetyTime now){next_=now+period_;}
 private:
 bool startup_only_,submitted_=false;
 std::chrono::milliseconds period_;
 SafetyTime next_{};
};
}
