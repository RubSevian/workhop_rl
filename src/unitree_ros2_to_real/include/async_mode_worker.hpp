#pragma once

#include "sdk_mode_session.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace sim2real {
// One persistent caller for the existing SDK-only process. No per-query
// threads and no queue of stale requests. RPC never holds the mailbox mutex.
class AsyncModeWorker {
 public:
  using Request = std::function<ModeSessionResult(bool, int)>;
  explicit AsyncModeWorker(Request request) : request_(std::move(request)),
      worker_([this] { Run(); }) {}
  ~AsyncModeWorker() {
    { std::lock_guard lock(mutex_); stopping_ = true; }
    ready_.notify_one();
    worker_.join();
  }
  AsyncModeWorker(const AsyncModeWorker&) = delete;
  AsyncModeWorker& operator=(const AsyncModeWorker&) = delete;
  bool busy() const { return busy_.load(std::memory_order_acquire); }
  bool Submit(bool release, int timeout_ms) {
    std::lock_guard lock(mutex_);
    if (stopping_ || busy()) return false;
    release_ = release; timeout_ms_ = timeout_ms; pending_ = true;
    busy_.store(true, std::memory_order_release);
    ready_.notify_one();
    return true;
  }
  std::optional<ModeSessionResult> Take() {
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (!lock || !result_) return std::nullopt;
    auto result = std::move(result_); result_.reset();
    busy_.store(false, std::memory_order_release);
    return result;
  }
 private:
  void Run() {
    for (;;) {
      bool release; int timeout;
      {
        std::unique_lock lock(mutex_);
        ready_.wait(lock, [&] { return stopping_ || pending_; });
        if (stopping_) return;
        release = release_; timeout = timeout_ms_; pending_ = false;
      }
      ModeSessionResult result;
      try { result = request_(release, timeout); }
      catch (const std::exception& e) { result.detail = e.what(); }
      catch (...) { result.detail = "unexpected_mode_worker_exception"; }
      { std::lock_guard lock(mutex_); result_ = std::move(result); }
    }
  }
  Request request_;
  std::atomic<bool> busy_{false};
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_ = false, pending_ = false, release_ = false;
  int timeout_ms_ = 0;
  std::optional<ModeSessionResult> result_;
  std::thread worker_;
};
}  // namespace sim2real
