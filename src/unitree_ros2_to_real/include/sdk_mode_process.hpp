#pragma once
// SDK2 stays in its own process. No ROS/Torch DDS mixing, no shell execution.
#include <spawn.h>
#include <sys/wait.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <algorithm>
#include <string>
#include <cerrno>
extern char** environ;
namespace sim2real {
struct ModeProcessResult { bool ok=false;std::string state="SPORT_ERROR",detail; };
inline ModeProcessResult RunModeProcess(const std::string& executable,const std::string& interface,
                                      bool release,int timeout_ms=8000) {
 if(interface.empty()||timeout_ms<=0)return {false,"SPORT_ERROR","missing interface / timeout"};
 int pipefd[2];if(pipe2(pipefd,O_CLOEXEC)!=0)return {false,"SPORT_ERROR","pipe failed"};
 posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
 posix_spawn_file_actions_adddup2(&actions,pipefd[1],STDOUT_FILENO);
 posix_spawn_file_actions_adddup2(&actions,pipefd[1],STDERR_FILENO);
 posix_spawn_file_actions_addclose(&actions,pipefd[0]);
 posix_spawn_file_actions_addclose(&actions,pipefd[1]);
 char* args[]{const_cast<char*>(executable.c_str()),const_cast<char*>("--interface"),
  const_cast<char*>(interface.c_str()),const_cast<char*>(release?"--release-sport-mode":"--status"),
  const_cast<char*>("--machine-readable"),nullptr};
 pid_t pid=-1;int rc=posix_spawn(&pid,executable.c_str(),&actions,nullptr,args,environ);
 posix_spawn_file_actions_destroy(&actions);close(pipefd[1]);
 if(rc){close(pipefd[0]);return {false,"SPORT_ERROR","spawn failed: "+std::to_string(rc)};}
 fcntl(pipefd[0],F_SETFL,fcntl(pipefd[0],F_GETFL)|O_NONBLOCK);
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
 std::string output;int status=0;bool exited=false,timedout=false;
 auto drain=[&]{char bytes[1024];ssize_t count;while((count=read(pipefd[0],bytes,sizeof(bytes)))>0)
  if(output.size()<8192)output.append(bytes,std::min<size_t>(count,8192-output.size()));};
 while(!exited) {
  drain();const auto waited=waitpid(pid,&status,WNOHANG);
  if(waited==pid){exited=true;break;}
  if(waited<0&&errno!=EINTR){timedout=true;break;}
  if(std::chrono::steady_clock::now()>=deadline){timedout=true;break;}
  pollfd fd{pipefd[0],POLLIN,0};poll(&fd,1,10);
 }
 if(timedout){kill(pid,SIGKILL);while(waitpid(pid,&status,0)<0&&errno==EINTR){};}
 drain();close(pipefd[0]);
 if(timedout)return {false,"SPORT_ERROR","SDK helper timeout; release may have occurred; output remains blocked"};
 std::string state;size_t start=0;int found=0;
 while(start<output.size()) {const auto end=output.find('\n',start);auto line=output.substr(start,end==std::string::npos?end:end-start);
  if(line=="SPORT_ACTIVE"||line=="SPORT_RELEASED"||line=="SPORT_UNKNOWN"||line=="SPORT_ERROR"){state=line;++found;}
  if(end==std::string::npos)break;start=end+1;
 }
 const bool ok=WIFEXITED(status)&&WEXITSTATUS(status)==0&&found==1&&
  (state=="SPORT_ACTIVE"||state=="SPORT_RELEASED")&&(!release||state=="SPORT_RELEASED");
 return {ok,ok?state:"SPORT_ERROR",output};
}
} // namespace sim2real
