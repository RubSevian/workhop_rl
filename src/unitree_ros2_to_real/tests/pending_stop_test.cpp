#include "pending_stop.hpp"
#include <cassert>
#include <thread>
using namespace sim2real;
int main() {
 PendingStopLatch latch;
 assert(latch.Take()==PendingStop::NONE);
 latch.Publish(PendingStop::ABORT);latch.Publish(PendingStop::NONE);
 assert(latch.Take()==PendingStop::ABORT);assert(latch.Take()==PendingStop::NONE);
 for(bool emergency_first:{true,false}) {
  latch.Publish(emergency_first?PendingStop::EMERGENCY:PendingStop::ABORT);
  latch.Publish(emergency_first?PendingStop::ABORT:PendingStop::EMERGENCY);
  assert(latch.Take()==PendingStop::EMERGENCY);assert(latch.Take()==PendingStop::NONE);
 }
 // One emergency per handshake: racing Take must never lose it, duplicate it,
 // or downgrade it to ABORT. The producer's acknowledgement bounds this test.
 std::atomic<unsigned> acknowledged{0},published{0};
 constexpr unsigned count=10000;
 std::thread producer([&]{
  for(unsigned i=1;i<=count;++i) {
   while(acknowledged.load(std::memory_order_acquire)!=i-1)std::this_thread::yield();
   latch.Publish(PendingStop::EMERGENCY);
   published.store(i,std::memory_order_release);
  }
 });
 for(unsigned i=1;i<=count;++i) {
  PendingStop stop;
  do {stop=latch.Take();}while(stop==PendingStop::NONE);
  assert(stop==PendingStop::EMERGENCY);
  while(published.load(std::memory_order_acquire)!=i)std::this_thread::yield();
  assert(latch.Take()==PendingStop::NONE);
  acknowledged.store(i,std::memory_order_release);
 }
 producer.join();
}
