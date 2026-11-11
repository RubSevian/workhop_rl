#pragma once
#include <charconv>
#include <cstdint>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

namespace sim2real {
inline int64_t ParseRunnableWaitNs(std::string_view text) {
  uint64_t runtime=0,wait=0;
  const auto end=text.data()+text.size();
  auto first=std::from_chars(text.data(),end,runtime);
  if(first.ec!=std::errc{}||first.ptr==end||*first.ptr!=' ')return -1;
  auto second=std::from_chars(first.ptr+1,end,wait);
  if(second.ec!=std::errc{}||second.ptr==end||*second.ptr!=' '||wait>INT64_MAX)return -1;
  return int64_t(wait);
}
// Diagnostic only. Disabled/unavailable kernel accounting is unknown, not zero.
inline int64_t ReceiverRunnableWaitNs() {
  struct Reader {
    int fd=-1;
    Reader() {
      const int enabled=open("/proc/sys/kernel/sched_schedstats",O_RDONLY|O_CLOEXEC);
      char flag=0;
      if(enabled>=0){const auto n=read(enabled,&flag,1);close(enabled);if(n!=1)flag=0;}
      if(flag=='1')fd=open("/proc/thread-self/schedstat",O_RDONLY|O_CLOEXEC);
    }
    ~Reader(){if(fd>=0)close(fd);}
  };
  thread_local Reader reader;
  if(reader.fd<0)return -1;
  char text[128];const auto count=pread(reader.fd,text,sizeof(text),0);
  return count>0?ParseRunnableWaitNs(std::string_view(text,size_t(count))):-1;
}
}
