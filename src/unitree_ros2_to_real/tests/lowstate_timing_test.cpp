#include "lowstate_timing.hpp"
#include <cassert>
#include <thread>
using namespace sim2real;
using namespace std::chrono_literals;
int main() {
 LowStateTiming timing;const auto start=LowStateTiming::Time{}+1s;
 timing.Receive(start);assert(timing.ReceiveGapMaxMs()==0);
 timing.Receive(start+2ms);timing.Receive(start+19ms);
 timing.Receive(start+21ms);assert(timing.ReceiveGapMaxMs()==17);
 timing.Use({},start);timing.Use(start+1s,start);
 assert(timing.IoAgeMaxMs()==0);
 timing.Use(start,start+7ms);timing.Use(start,start+2ms);
 assert(timing.IoAgeMaxMs()==7);
 // Receiver, IO and status operate independently without shared plain data.
 std::thread receiver([&]{for(int n=1;n<=10000;++n)timing.Receive(start+1s+n*2ms);});
 std::thread io([&]{for(int n=1;n<=10000;++n)timing.Use(start,start+n*1ms);});
 double last_gap=0,last_age=0;
 for(int n=0;n<10000;++n){
  const auto gap=timing.ReceiveGapMaxMs(),age=timing.IoAgeMaxMs();
  assert(gap>=last_gap && age>=last_age);last_gap=gap;last_age=age;
 }
 receiver.join();io.join();assert(timing.IoAgeMaxMs()==10000);
}
