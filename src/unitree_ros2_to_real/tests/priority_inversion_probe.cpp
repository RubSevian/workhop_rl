// Offline Linux scheduler experiment only. No ROS, DDS, SDK or device access.
// Requires RLIMIT_RTPRIO=20 on this process only. All workers terminate.
#include "control_latency.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/resource.h>
#include <sched.h>
using namespace sim2real;
void At(int64_t ns){timespec t{ns/1000000000,ns%1000000000};while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr)==EINTR){}}
void Burn(int64_t ns){const auto end=ControlCpuNs()+ns;while(ControlCpuNs()<end){}}
void Configure(int cpu,int priority){cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpu,&mask);
 if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))std::terminate();
 sched_param p{};p.sched_priority=priority;
 if(pthread_setschedparam(pthread_self(),priority?SCHED_FIFO:SCHED_OTHER,&p))std::terminate();}
template<class Mutex> void Run(const char* name,int cpu){
 Mutex mutex;const auto start=ControlMonotonicNs()+100000000;
 std::atomic<bool> held{false};int64_t wait=0,hold=0;
 std::thread owner([&]{Configure(cpu,0);At(start);std::lock_guard lock(mutex);
  const auto began=ControlMonotonicNs();held=true;Burn(10000000);hold=ControlMonotonicNs()-began;});
 std::thread medium([&]{Configure(cpu,10);At(start+1000000);Burn(50000000);});
 std::thread high([&]{Configure(cpu,20);At(start+2000000);
  if(!held)std::terminate();const auto began=ControlMonotonicNs();
  {std::lock_guard lock(mutex);wait=ControlMonotonicNs()-began;}});
 owner.join();medium.join();high.join();
 std::cout<<name<<","<<hold/1.e6<<","<<wait/1.e6<<"\n";
}
int main(){rlimit limit{};getrlimit(RLIMIT_RTPRIO,&limit);
 if(limit.rlim_cur<20){std::cerr<<"Requires process-local RLIMIT_RTPRIO=20\n";return 77;}
 cpu_set_t mask;CPU_ZERO(&mask);sched_getaffinity(0,sizeof(mask),&mask);int cpu=0;
 while(cpu<CPU_SETSIZE&&!CPU_ISSET(cpu,&mask))++cpu;
 std::cout<<"mutex,owner_hold_ms,policy_wait_ms\n";
 for(int i=0;i<3;++i){Run<std::mutex>("ordinary",cpu);Run<PriorityMutex>("inherit",cpu);}
}
