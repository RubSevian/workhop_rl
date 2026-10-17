#pragma once
#include <cerrno>
#include <cstdint>
#include <ctime>
#include <stdexcept>
#include <system_error>

namespace sim2real {
inline int64_t PolicyMonotonicNs() {
 timespec time{};
 if(clock_gettime(CLOCK_MONOTONIC,&time))throw std::system_error(errno,std::generic_category());
 return int64_t(time.tv_sec)*1000000000+time.tv_nsec;
}
// One dedicated worker, absolute CLOCK_MONOTONIC releases. Skip missed
// releases after an overrun; never run a burst of catch-up inferences.
// No ROS waitset, graph notification or executor mutex in the release path.
template<class Running,class Tick>
void RunPeriodicPolicy(int64_t period_ns,Running running,Tick tick) {
 if(period_ns<=0)throw std::invalid_argument("Positive policy period required");
 auto expected=PolicyMonotonicNs()+period_ns;
 while(running()) {
  const timespec deadline{expected/1000000000,expected%1000000000};
  int result;
  do {result=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,nullptr);}
  while(result==EINTR&&running());
  if(!running())break;
  if(result)throw std::system_error(result,std::generic_category());
  tick(expected,PolicyMonotonicNs());
  expected+=period_ns;
  const auto completed=PolicyMonotonicNs();
  if(completed>=expected)expected+=((completed-expected)/period_ns+1)*period_ns;
 }
}
} // namespace sim2real
