#include "control_latency.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <system_error>
#include <sys/syscall.h>
#include <unistd.h>

namespace sim2real {
namespace {
void Check(int result) { if(result)throw std::system_error(result,std::generic_category()); }
int64_t Clock(clockid_t clock) noexcept {
  timespec t{};if(clock_gettime(clock,&t))return -1;
  return int64_t(t.tv_sec)*1000000000+t.tv_nsec;
}
}
PriorityMutex::PriorityMutex() {
  pthread_mutexattr_t attr;
  Check(pthread_mutexattr_init(&attr));
  const int protocol=pthread_mutexattr_setprotocol(&attr,PTHREAD_PRIO_INHERIT);
  const int result=protocol?protocol:pthread_mutex_init(&mutex_,&attr);
  pthread_mutexattr_destroy(&attr);
  Check(result);
}
PriorityMutex::~PriorityMutex(){pthread_mutex_destroy(&mutex_);}
void PriorityMutex::lock(){Check(pthread_mutex_lock(&mutex_));}
void PriorityMutex::unlock() noexcept {if(pthread_mutex_unlock(&mutex_))std::terminate();}
int64_t ControlMonotonicNs() noexcept {return Clock(CLOCK_MONOTONIC);}
int64_t ControlCpuNs() noexcept {return Clock(CLOCK_THREAD_CPUTIME_ID);}
int ControlTid() noexcept {thread_local const int tid=int(syscall(SYS_gettid));return tid;}
ControlLatency::ControlLatency(const std::string& path,size_t capacity,double seconds,int flush_ms,int writer_idle_ms)
 :flush_ms_(flush_ms),writer_idle_ms_(writer_idle_ms) {
  if(capacity<256||capacity>65536||!std::isfinite(seconds)||seconds<=0||seconds>600||flush_ms<1||flush_ms>1000||writer_idle_ms<1||writer_idle_ms>1000)
    throw std::invalid_argument("Invalid control_latency YAML configuration");
  capacity_=capacity;queue_=std::make_unique<Cell[]>(capacity);
  for(size_t i=0;i<capacity;++i)queue_[i].sequence.store(i,std::memory_order_relaxed);
  deadline_=ControlMonotonicNs()+int64_t(seconds*1e9);
  file_=fopen(path.c_str(),"w");if(!file_)throw std::runtime_error("Cannot open control latency trace: "+path);
  fprintf(file_,"kind,site,tid,begin_ns,acquired_ns,end_ns,cpu_ns,expected_ns,actual_ns\n");fflush(file_);
  try{writer_=std::thread([this]{Write();});}catch(...){fclose(file_);throw;}
}
ControlLatency::~ControlLatency(){enabled_=false;stop_=true;if(writer_.joinable())writer_.join();}
void ControlLatency::Submit(ControlLatencySample s) noexcept {
  if(!enabled())return;
  if(s.end>=deadline_){enabled_=false;return;}
  // Bounded MPSC queue: producers never block on a logger mutex or disk.
  auto pos=tail_.load(std::memory_order_relaxed);
  for(int attempt=0;attempt<16;++attempt){
    auto& cell=queue_[pos%capacity_];
    const auto seq=cell.sequence.load(std::memory_order_acquire);
    if(seq==pos){
      if(tail_.compare_exchange_weak(pos,pos+1,std::memory_order_relaxed)){
        cell.sample=s;cell.sequence.store(pos+1,std::memory_order_release);return;
      }
    }else if(seq<pos){break;}else pos=tail_.load(std::memory_order_relaxed);
  }
  ++dropped_;
}
void ControlLatency::Write() {
  std::array<ControlLatencySample,256> batch;
  auto flushed=ControlMonotonicNs();
  for(;;) {
    size_t count=0;
    while(count<batch.size()){
      auto& cell=queue_[head_%capacity_];
      if(cell.sequence.load(std::memory_order_acquire)!=head_+1)break;
      batch[count++]=cell.sample;
      cell.sequence.store(head_+capacity_,std::memory_order_release);++head_;
    }
    for(size_t i=0;i<count;++i){const auto& s=batch[i];
      fprintf(file_,"%d,%d,%d,%lld,%lld,%lld,%lld,%lld,%lld\n",s.kind,s.site,s.tid,
        (long long)s.begin,(long long)s.acquired,(long long)s.end,(long long)s.cpu_ns,(long long)s.expected,(long long)s.actual);}
    const auto now=ControlMonotonicNs();
    if(now-flushed>=int64_t(flush_ms_)*1000000){fflush(file_);flushed=now;}
    if(ferror(file_)){enabled_=false;break;}
    if(now>=deadline_)enabled_=false;
    if(!count){if((stop_||!enabled())&&head_==tail_.load())break;std::this_thread::sleep_for(std::chrono::milliseconds(writer_idle_ms_));}
  }
  // Final metadata is machine readable and also present in native status.
  fprintf(file_,"# dropped=%llu\n",(unsigned long long)dropped());
  fclose(file_);
}
ControlScope::ControlScope(ControlLatency* trace,int kind,int site,int64_t expected,int64_t actual) noexcept
 :trace_(trace&&trace->enabled()?trace:nullptr) {
  if(trace_){sample_={kind,site,ControlTid(),ControlMonotonicNs(),0,0,0,expected,actual};cpu_=ControlCpuNs();}
}
ControlScope::~ControlScope(){if(trace_){sample_.end=ControlMonotonicNs();sample_.cpu_ns=ControlCpuNs()-cpu_;trace_->Submit(sample_);}}
ControlLock::ControlLock(PriorityMutex& mutex,ControlLatency* trace,int site)
 :mutex_(mutex),trace_(trace&&trace->enabled()?trace:nullptr) {
  if(trace_){sample_={1,site,ControlTid(),ControlMonotonicNs()};cpu_=ControlCpuNs();}
  mutex_.lock();if(trace_)sample_.acquired=ControlMonotonicNs();
}
ControlLock::~ControlLock(){
  if(trace_){sample_.end=ControlMonotonicNs();sample_.cpu_ns=ControlCpuNs()-cpu_;}
  mutex_.unlock(); // Never queue diagnostic data while owning the control lock.
  if(trace_)trace_->Submit(sample_);
}
} // namespace sim2real
