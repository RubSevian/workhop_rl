#include "latest_snapshot.hpp"
#include <array>
#include <cassert>
#include <cstdint>
#include <thread>

struct Sample { std::uint64_t sequence{}; std::array<std::uint64_t,64> payload{}; };
Sample sample(std::uint64_t sequence) {
  Sample result; result.sequence=sequence;
  for(std::size_t i=0;i<result.payload.size();++i)result.payload[i]=sequence*127+i;
  return result;
}
int main() {
  sim2real::LatestSnapshot<Sample> mailbox; Sample result;
  assert(!mailbox.Read(result));
  for(unsigned i=1;i<=1000;++i)mailbox.Publish(sample(i));
  assert(mailbox.Read(result) && result.sequence==1000);
  assert(!mailbox.Read(result));
  constexpr std::uint64_t count=200000;
  std::atomic<bool> finished{false};
  std::thread producer([&]{
    for(std::uint64_t i=1001;i<=count;++i)mailbox.Publish(sample(i));
    finished.store(true,std::memory_order_release);
  });
  std::uint64_t last=1000;
  while(last<count) {
    if(mailbox.Read(result)) {
      assert(result.sequence>last);
      for(std::size_t i=0;i<result.payload.size();++i)assert(result.payload[i]==result.sequence*127+i);
      last=result.sequence;
      // A stalled reader must not block writes or tear the next snapshot.
      if(last%7==0)std::this_thread::yield();
    } else if(finished.load(std::memory_order_acquire)) {
      // Completion may race the initial Read; perform one final acquire/read.
      if(mailbox.Read(result)) {
        assert(result.sequence==count);
        for(std::size_t i=0;i<result.payload.size();++i)assert(result.payload[i]==count*127+i);
        last=result.sequence;
      } else assert(last==count);
    }
  }
  producer.join();
}
