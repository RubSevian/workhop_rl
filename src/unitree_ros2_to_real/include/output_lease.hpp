#pragma once
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <string>
namespace sim2real {
class OutputLease {
 public:
 explicit OutputLease(const std::string& path) {
  if(path.empty())return;
  const int fd=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
  if(fd<0)return;
  if(flock(fd,LOCK_EX|LOCK_NB)!=0){close(fd);return;}fd_=fd;
 }
 ~OutputLease(){if(fd_>=0){flock(fd_,LOCK_UN);close(fd_);}}
 OutputLease(const OutputLease&)=delete;OutputLease& operator=(const OutputLease&)=delete;
 bool acquired() const {return fd_>=0;}
 private:int fd_=-1;
};
} // namespace sim2real
