#include "async_mode_worker.hpp"
#include <cassert>
#include <chrono>
#include <future>

int main() {
  using namespace sim2real;
  std::promise<void> finish;
  auto gate = finish.get_future().share();
  std::atomic<int> calls{0};
  AsyncModeWorker worker([&](bool release, int timeout) {
    ++calls; gate.wait();
    ModeSessionResult result; result.ok=true;
    result.state=release?"SPORT_RELEASED":"SPORT_ACTIVE";
    result.detail=std::to_string(timeout);return result;
  });
  assert(worker.Submit(true,8000));
  assert(worker.busy() && !worker.Submit(false,2000));
  assert(!worker.Take());
  finish.set_value();
  auto take = [&] {
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    std::optional<ModeSessionResult> result;
    while (!(result=worker.Take()) && std::chrono::steady_clock::now()<end)
      std::this_thread::yield();
    assert(result);return *result;
  };
  auto result=take();
  assert(result.ok && result.state=="SPORT_RELEASED" && result.detail=="8000");
  assert(!worker.busy() && !worker.Take());
  assert(worker.Submit(false,2000));assert(take().state=="SPORT_ACTIVE");
  assert(calls==2);
  AsyncModeWorker failing([](bool,int)->ModeSessionResult{throw std::runtime_error("fake failure");});
  assert(failing.Submit(false,2000));
  std::optional<ModeSessionResult> failed;
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while (!(failed=failing.Take()) && std::chrono::steady_clock::now()<end)std::this_thread::yield();
  assert(failed && !failed->ok && failed->detail=="fake failure");
}
