#pragma once
// One SDK-only child, one request in flight; no ROS/SDK DDS library mixing.
#include <spawn.h>
#include <sys/wait.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <string>
#include <cerrno>
#include <cstdint>
extern char** environ;
#include <sys/socket.h>
#include <sstream>
#include <utility>
#include <thread>

namespace sim2real {
struct ModeSessionResult {
 bool ok=false;
 std::string state="SPORT_ERROR",detail;
 int64_t started_ns=0,completed_ns=0;
};
class ModeSessionProcess final {
 public:
 ModeSessionProcess(std::string executable,std::string interface,std::string config={})
   :executable_(std::move(executable)),interface_(std::move(interface)),config_(std::move(config)){}
 ~ModeSessionProcess(){Close();}
 ModeSessionProcess(const ModeSessionProcess&)=delete;
 ModeSessionProcess& operator=(const ModeSessionProcess&)=delete;
 // Calls must be serialized by the caller. Poll/read/write never run on leg IO.
 ModeSessionResult Request(bool release,int timeout_ms=2000) {
  if(timeout_ms<=0||interface_.empty())return Failure("invalid session request");
  if(fd_<0&&!Start())return Failure("SDK session spawn failed");
  request_sent_ns_=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const uint64_t id=++request_id_;
  const std::string request=std::to_string(id)+(release?" RELEASE\n":" STATUS\n");
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
  size_t offset=0;
  while(offset<request.size()) {
   const auto sent=send(fd_,request.data()+offset,request.size()-offset,MSG_NOSIGNAL);
   if(sent>0){offset+=sent;continue;}
   if(sent<0&&errno==EINTR)continue;
   if(sent<0&&(errno==EAGAIN||errno==EWOULDBLOCK)&&Wait(POLLOUT,deadline))continue;
   return Failure("SDK session write failed");
  }
  for(;;) {
   size_t end;
   while((end=buffer_.find('\n'))!=std::string::npos) {
    const auto line=buffer_.substr(0,end);buffer_.erase(0,end+1);
    if(line.rfind("R3_MODE_V1 ",0)!=0)continue;
    std::istringstream record(line);std::string tag;uint64_t reply=0;int ok=0;
    ModeSessionResult result;
    if(!(record>>tag>>reply>>result.state>>result.started_ns>>result.completed_ns>>ok)||
       reply!=id||(ok!=0&&ok!=1)||result.started_ns<=0||result.completed_ns<result.started_ns)
      return Failure("invalid SDK session record");
    const auto now_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if(result.completed_ns>now_ns||result.started_ns<request_sent_ns_)
      return Failure("invalid SDK session timestamp");
    if(result.state!="SPORT_ACTIVE"&&result.state!="SPORT_RELEASED"&&
       result.state!="SPORT_UNKNOWN"&&result.state!="SPORT_ERROR")
      return Failure("invalid SDK session state");
    std::getline(record,result.detail);
    result.ok=ok&&(result.state=="SPORT_ACTIVE"||result.state=="SPORT_RELEASED")&&
                (!release||result.state=="SPORT_RELEASED");
    return result;
   }
   if(!Wait(POLLIN,deadline))return Failure("SDK session response timeout");
   char bytes[1024];const auto count=read(fd_,bytes,sizeof(bytes));
   if(count>0){buffer_.append(bytes,count);if(buffer_.size()>8192)return Failure("SDK session output overflow");}
   else if(count==0)return Failure("SDK session exited");
   else if(errno!=EINTR&&errno!=EAGAIN&&errno!=EWOULDBLOCK)return Failure("SDK session read failed");
  }
 }
 private:
 bool Wait(short events,std::chrono::steady_clock::time_point deadline) {
  for(;;) {
   const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
   if(remaining<=0)return false;
   pollfd fd{fd_,events,0};const auto result=poll(&fd,1,static_cast<int>(remaining));
   if(result>0)return fd.revents&(events|POLLHUP|POLLERR);
   if(result<0&&errno==EINTR)continue;
   return false;
  }
 }
 bool Start() {
  int pair[2];if(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,pair)!=0)return false;
  posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions,pair[1],STDIN_FILENO);
  posix_spawn_file_actions_adddup2(&actions,pair[1],STDOUT_FILENO);
  posix_spawn_file_actions_addopen(&actions,STDERR_FILENO,"/dev/null",O_WRONLY,0);
  posix_spawn_file_actions_addclose(&actions,pair[0]);posix_spawn_file_actions_addclose(&actions,pair[1]);
  char* args[]{const_cast<char*>(executable_.c_str()),const_cast<char*>("--interface"),
   const_cast<char*>(interface_.c_str()),const_cast<char*>("--session"),
   config_.empty()?nullptr:const_cast<char*>("--session-config"),config_.empty()?nullptr:const_cast<char*>(config_.c_str()),nullptr};
  const int result=posix_spawn(&pid_,executable_.c_str(),&actions,nullptr,args,environ);
  posix_spawn_file_actions_destroy(&actions);close(pair[1]);
  if(result){close(pair[0]);pid_=-1;return false;}
  fd_=pair[0];fcntl(fd_,F_SETFL,fcntl(fd_,F_GETFL)|O_NONBLOCK);
  return true;
 }
 void Close() noexcept {
  if(fd_>=0){close(fd_);fd_=-1;}
  if(pid_>0){
   int status=0;
   // EOF lets the SDK client/factory unwind normally. Bound shutdown if the
   // child is inside an unresponsive RPC; this runs on the async worker only.
   pid_t waited=0;
   for(int attempt=0;attempt<20;++attempt) {
    waited=waitpid(pid_,&status,WNOHANG);
    if(waited==pid_||(waited<0&&errno!=EINTR))break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
   }
   if(waited==0||(waited<0&&errno==EINTR)) {
    kill(pid_,SIGKILL);while(waitpid(pid_,&status,0)<0&&errno==EINTR){}
   }
   pid_=-1;
  }
  buffer_.clear();
 }
 ModeSessionResult Failure(const std::string& message){Close();return {false,"SPORT_ERROR",message};}
 std::string executable_,interface_,config_,buffer_;
 pid_t pid_=-1;int fd_=-1;uint64_t request_id_=0;int64_t request_sent_ns_=0;
};
} // namespace sim2real
