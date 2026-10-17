#pragma once
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <memory>
#include <pthread.h>
#include <string>
#include <thread>
#include <vector>

namespace sim2real {
// No fallback to a non-PI mutex: startup fails if the kernel rejects PI.
class PriorityMutex {
 public:
  PriorityMutex();
  ~PriorityMutex();
  PriorityMutex(const PriorityMutex&) = delete;
  PriorityMutex& operator=(const PriorityMutex&) = delete;
  void lock();
  void unlock() noexcept;
 private:
  pthread_mutex_t mutex_{};
};

// All wall stamps use CLOCK_MONOTONIC. CPU time distinguishes work from
// off-CPU time, but cannot distinguish runnable delay from blocking by itself.
struct ControlLatencySample {
  int kind=0,site=0,tid=0;
  int64_t begin=0,acquired=0,end=0,cpu_ns=0,expected=0,actual=0;
};
int64_t ControlMonotonicNs() noexcept;
int64_t ControlCpuNs() noexcept;
int ControlTid() noexcept;

class ControlLatency {
 public:
  static_assert(std::atomic<size_t>::is_always_lock_free);
  ControlLatency(const std::string& path,size_t capacity,double seconds,int flush_ms,int writer_idle_ms=5);
  ~ControlLatency();
  bool enabled() const noexcept { return enabled_.load(std::memory_order_relaxed); }
  void Submit(ControlLatencySample sample) noexcept;
  uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
 private:
  void Write();
  FILE* file_=nullptr;
  struct Cell {std::atomic<size_t> sequence{0};ControlLatencySample sample{};};
  std::unique_ptr<Cell[]> queue_;
  size_t capacity_=0,head_=0;
  alignas(64) std::atomic<size_t> tail_{0};
  int flush_ms_,writer_idle_ms_;
  int64_t deadline_;
  std::atomic<bool> enabled_{true},stop_{false};
  std::atomic<uint64_t> dropped_{0};
  std::thread writer_;
};

// Records a complete scope, including its tail. Sites are source line numbers;
// retain the source hash/source copy alongside each binary and timing capture.
class ControlScope {
 public:
  ControlScope(ControlLatency* trace,int kind,int site,int64_t expected=0,int64_t actual=0) noexcept;
  ~ControlScope();
 private:
  ControlLatency* trace_;
  ControlLatencySample sample_{};
  int64_t cpu_=0;
};
class ControlLock {
 public:
  ControlLock(PriorityMutex& mutex,ControlLatency* trace,int site);
  ~ControlLock();
  ControlLock(const ControlLock&)=delete;
 private:
  PriorityMutex& mutex_;
  ControlLatency* trace_;
  ControlLatencySample sample_{};
  int64_t cpu_=0;
};
} // namespace sim2real
