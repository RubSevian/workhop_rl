#pragma once
// Cooperative policy-thread lease, promoted from the offline-qualified protocol.
// Only the owning guardian changes scheduling; no RT inheritance by IO/DDS.
#include "policy_rt_lease.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <fstream>
#include <string>
#include <sstream>
#include <cerrno>
#include <unistd.h>

namespace sim2real {
class PolicyRtRegistration {
 public:
  offline_rt::PolicyLease lease;
  std::atomic<bool> ready{false};
  uint64_t registration_timeout_ns=8000000000ULL;
  int registration_poll_ms=1;
  bool Ready() const {return ready.load(std::memory_order_acquire)&&lease.data->fault.load()==0;}
  static uint64_t Start(int tid) {
    std::ifstream f("/proc/"+std::to_string(tid)+"/stat");
    std::string line;std::getline(f,line);
    auto pos=line.rfind(')');
    if(!f||pos==std::string::npos)throw std::runtime_error("RT identity unreadable");
    std::istringstream fields(line.substr(pos+2));std::string value;
    for(int i=0;i<=19;++i)if(!(fields>>value))throw std::runtime_error("RT identity malformed");
    return std::stoull(value);
  }
  void Enter() {
    static thread_local bool registered=false;if(registered)return;
    const int pid=getpid(),tid=syscall(SYS_gettid),parent=getppid();
    std::ifstream f("/proc/"+std::to_string(parent)+"/comm");std::string comm;std::getline(f,comm);
    if(comm!="r3_rt_guardian"||getuid()==0||sched_getscheduler(parent)!=SCHED_OTHER||sched_getscheduler(pid)!=SCHED_OTHER)
      throw std::runtime_error("Cooperating non-root OTHER guardian required");
    prctl(PR_SET_NAME,"r3_rt_policy",0,0,0);
    static thread_local std::unique_ptr<offline_rt::PolicyLease::Exit> exit;
    exit=std::make_unique<offline_rt::PolicyLease::Exit>();exit->owner=&lease;
    lease.data->policy_progress=offline_rt::now();
    std::ostringstream message;
    message<<"\n{\"ready\":true,\"pid\":"<<pid<<",\"tid\":"<<tid
      <<",\"pid_start\":"<<Start(pid)<<",\"tid_start\":"<<Start(tid)
      <<",\"uid\":"<<getuid()<<",\"lease_fd\":"<<lease.fd<<"}\n";
    // One short write: chained iostream insertions can interleave with ROS
    // logging and destroy the guardian's identity record before RT admission.
    const auto record=message.str();ssize_t written;
    do{written=::write(STDOUT_FILENO,record.data(),record.size());}while(written<0&&errno==EINTR);
    if(written!=static_cast<ssize_t>(record.size()))throw std::runtime_error("RT identity write incomplete");
    const auto deadline=offline_rt::now()+registration_timeout_ns;
    while(lease.data->state!=1&&rclcpp::ok()&&offline_rt::now()<deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(registration_poll_ms));
    if(!rclcpp::ok()||lease.data->state!=1||sched_getscheduler(0)!=SCHED_FIFO)
      throw std::runtime_error("FIFO20 registration not armed");
    sched_param p{};if(sched_getparam(0,&p)||p.sched_priority!=20)
      throw std::runtime_error("Policy priority must be exactly 20");
    registered=true;ready.store(true,std::memory_order_release);
  }
  void Begin(){lease.data->inflight=offline_rt::now();++lease.data->jobs;}
  void Complete(){lease.data->policy_progress=offline_rt::now();lease.data->inflight=0;}
  void Io(){lease.data->io_progress=offline_rt::now();++lease.data->ios;}
  void Input(){lease.data->input_stamp=offline_rt::now();}
};
}
