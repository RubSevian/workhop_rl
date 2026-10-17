#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>
namespace offline_rt {
inline uint64_t now(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return uint64_t(t.tv_sec)*1000000000+t.tv_nsec;}
struct LeaseData {
 char magic[8];
 std::atomic<uint64_t> state{0},policy_progress{0},io_progress{0},input_stamp{0},inflight{0},jobs{0},ios{0};
 std::atomic<uint64_t> exit_policy{99},exit_priority{99},reset_rc{99},fault{0};
};
static_assert(sizeof(std::atomic<uint64_t>)==8 && std::atomic<uint64_t>::is_always_lock_free);
class PolicyLease {
 public:
 int fd=-1;LeaseData* data=nullptr;
 PolicyLease(){
  fd=memfd_create("r3-offline-lease",MFD_CLOEXEC);if(fd<0||ftruncate(fd,4096))throw std::runtime_error("memfd lease failed");
  auto* ptr=mmap(nullptr,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);if(ptr==MAP_FAILED)throw std::runtime_error("mmap lease failed");
  data=new(ptr) LeaseData;memcpy(data->magic,"R3OFF001",8);
 }
 ~PolicyLease(){if(data){munmap(data,4096);}if(fd>=0)close(fd);}
 // Destruction must run ON the registered policy thread, after executor exits.
 struct Exit {
  PolicyLease* owner;
  ~Exit(){
   auto& d=*owner->data;
   if(flock(owner->fd,LOCK_EX)){d.reset_rc=1;d.state=3;return;}
   sched_param p{};int rc=sched_setscheduler(0,SCHED_OTHER,&p);
   d.exit_policy=sched_getscheduler(0);sched_getparam(0,&p);d.exit_priority=p.sched_priority;d.reset_rc=rc?1:0;
   d.state.store(2,std::memory_order_release);
   // Exclusive lease remains held until process exit. A setter cannot race exit.
  }
 };
};
}
