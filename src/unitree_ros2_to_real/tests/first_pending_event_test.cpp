#include "first_pending_event.hpp"
#include <cassert>
#include <cstdint>
#include <thread>
struct Event { std::uint64_t sequence=0,reason=0; };
int main() {
 sim2real::FirstPendingEvent<Event> slot;Event event;
 assert(!slot.Take(event));assert(slot.Publish({1,5}));assert(!slot.Publish({2,6}));
 assert(slot.Take(event)&&event.sequence==1&&event.reason==5);assert(!slot.Take(event));
 constexpr std::uint64_t count=100000;
 std::thread producer([&]{
  for(std::uint64_t i=1;i<=count;++i)while(!slot.Publish({i,i*127}))std::this_thread::yield();
 });
 for(std::uint64_t i=1;i<=count;++i) {
  while(!slot.Take(event))std::this_thread::yield();
  assert(event.sequence==i&&event.reason==i*127);
 }
 producer.join();
}
