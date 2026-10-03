#include "rars_bridge.hpp"
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
namespace sim2real {
namespace {
bool age_valid(SafetyTime now,SafetyTime then,double timeout) {
 const double age=std::chrono::duration<double>(now-then).count();return age>=0&&age<=timeout;
}
}
RarsBridge::RarsBridge(RarsTransport& transport,double feedback_timeout,double target_timeout)
 :transport_(transport),feedback_timeout_s_(feedback_timeout),target_timeout_s_(target_timeout) {
 if(!std::isfinite(feedback_timeout)||feedback_timeout<=0||!std::isfinite(target_timeout)||target_timeout<=0)
  throw std::invalid_argument("RARS timeouts must be finite and positive");
 // Keep the caller's saved SDK configuration, do not supply fallback offsets.
 for(const auto& motor:transport_.Configuration().motors)
  if(!std::isfinite(motor.zero_offset)||!std::isfinite(motor.direction)||std::abs(motor.direction)!=1)
   throw std::invalid_argument("Invalid SDK calibration direction/offset");
}
const ArmSnapshot& RarsBridge::Poll(SafetyTime now) {
 if(auto frame=transport_.Read(now))frame_=*frame;
 snapshot_.feedback_ready=false;snapshot_.target_ready=false;snapshot_.rejection.clear();
 if(frame_) {
  const auto& f=*frame_;snapshot_.received=f.received;
  const double elapsed=std::chrono::duration<double,std::milli>(now-f.received).count();
  snapshot_.feedback_age_ms=f.communication.feedback_age.count()+elapsed;
  bool valid=f.communication.connected&&f.communication.feedback_received&&
    !f.communication.watchdog_tripped&&!f.communication.stm32_watchdog_tripped&&
    f.communication.feedback_age.count()>=0&&elapsed>=0&&
    snapshot_.feedback_age_ms<=feedback_timeout_s_*1000;
  for(size_t i=0;i<6;++i) {
   // SDK direction*(q_raw-zero_offset) and direction*dq_raw already applied.
   snapshot_.q[i]=f.joints.position[i];snapshot_.dq[i]=f.joints.velocity[i];
   snapshot_.valid[i]=f.joints.valid[i]&&f.joints.motor_id[i]==i+1&&
     std::isfinite(snapshot_.q[i])&&std::isfinite(snapshot_.dq[i]);
   valid=valid&&snapshot_.valid[i]&&f.joints.error[i]<8;
  }
  snapshot_.feedback_ready=valid;
  if(!valid)snapshot_.rejection="invalid_or_stale_complete_sdk_frame";
 } else snapshot_.rejection="no_measured_feedback";
 // Missing state has no valid bits. Consumers MUST check feedback_ready.
 snapshot_.target=transport_.LastAcceptedTarget();
 if(snapshot_.target) {
  const auto& target=*snapshot_.target;
  bool valid=target.valid&&age_valid(now,target.accepted,target_timeout_s_);
  for(float q:target.q)valid=valid&&std::isfinite(q);
  snapshot_.target_ready=valid;
 }
 return snapshot_;
}
std::string RarsBridge::CalibrationDiagnostic() const {
 std::ostringstream s;s<<"source=SDK_GUI_OPERATOR_SAVED operator_verified=true physical_pose_validated=false per_joint_freshness_proven=false";
 const auto& c=transport_.Configuration();
 for(size_t i=0;i<7;++i)s<<"\n"<<(i==6?"gripper":"joint"+std::to_string(i+1))<<" direction="<<c.motors[i].direction<<" zero_offset="<<c.motors[i].zero_offset;
 return s.str();
}
SerialOwnerLease::SerialOwnerLease(const std::string& directory,const std::string& device) {
 // Hex-encoded full device path avoids basename/hash collisions.
 if(directory.empty()||device.empty())throw std::invalid_argument("Serial lease directory/device required");
 constexpr char hex[]="0123456789abcdef";std::string key;
 const std::string canonical_device=std::filesystem::weakly_canonical(device).string();
 for(unsigned char c:canonical_device){key+=hex[c>>4];key+=hex[c&15];}
 const std::string path=directory+"/rars-"+key+".lock";
 const int fd=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
 if(fd<0)return;
 if(flock(fd,LOCK_EX|LOCK_NB)!=0){close(fd);return;}
 fd_=fd;
}
SerialOwnerLease::~SerialOwnerLease(){if(fd_>=0){flock(fd_,LOCK_UN);close(fd_);}}
} // namespace sim2real
