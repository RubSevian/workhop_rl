#include "rars_auto_home.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
namespace sim2real {
namespace {
double Seconds(ArmTime now,ArmTime then){return std::chrono::duration<double>(now-then).count();}
ArmClock::duration Duration(double s){return std::chrono::duration_cast<ArmClock::duration>(std::chrono::duration<double>(s));}
}
FileAutoHomeJournal::FileAutoHomeJournal(std::string path,std::string boot_id):path_(std::move(path)),boot_id_(std::move(boot_id)) {
 if(path_.empty()||boot_id_.empty())throw std::invalid_argument("Persistent journal path and boot ID required");
 if(std::filesystem::is_symlink(path_))throw std::runtime_error("Journal cannot be a symlink");
 if(std::filesystem::exists(path_)) {
  std::ifstream file(path_);std::string kind,boot;std::getline(file,kind);std::getline(file,boot);
  if(!file||kind!="ENABLE_ATTEMPT")blocked_="persistent_fault_or_invalid_journal";
  else if(boot==boot_id_)blocked_="enable_already_attempted_this_boot";
 }
}
bool FileAutoHomeJournal::Save(const std::string& value) {
 // Atomic replace + file and directory fsync. Journal is outside /run.
 const std::string temporary=path_+".tmp";
 int fd=open(temporary.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC|O_NOFOLLOW,0600);
 if(fd<0)return false;
 size_t offset=0;bool ok=true;
 while(offset<value.size()){auto n=write(fd,value.data()+offset,value.size()-offset);if(n<=0){ok=false;break;}offset+=n;}
 if(ok)ok=fsync(fd)==0;close(fd);
 if(ok)ok=rename(temporary.c_str(),path_.c_str())==0;
 if(ok){const auto parent=std::filesystem::path(path_).parent_path();int directory=open(parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);if(directory<0)ok=false;else {ok=fsync(directory)==0;close(directory);}}
 if(!ok)unlink(temporary.c_str());return ok;
}
bool FileAutoHomeJournal::RecordEnableAttempt(){return Save("ENABLE_ATTEMPT\n"+boot_id_+"\n");}
bool FileAutoHomeJournal::RecordFault(const std::string& reason){return Save("FAULT\n"+boot_id_+"\n"+reason+"\n");}
const char* AutoHomeStateName(AutoHomeState s){switch(s){
 case AutoHomeState::READ_ONLY:return "READ_ONLY";case AutoHomeState::WAIT_DEVICE:return "WAIT_DEVICE";
 case AutoHomeState::WAIT_COMMUNICATION:return "WAIT_COMMUNICATION";case AutoHomeState::STARTUP_DELAY:return "STARTUP_DELAY";
 case AutoHomeState::HOLD_HOME:return "HOLD_HOME";case AutoHomeState::FAULT_LATCHED:return "FAULT_LATCHED";}return "UNKNOWN";}
AutoHomeController::AutoHomeController(AutoHomeConfig config,AutoHomeTransport& transport,AutoHomeJournal& journal)
 :config_(std::move(config)),transport_(transport),journal_(journal) {
 for(double v:{config_.command_rate_hz,config_.feedback_timeout_s,config_.target_timeout_s,config_.home_tolerance_rad,config_.connect_retry_s,config_.enable_feedback_grace_s})
  if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Positive finite auto HOME settings required");
 if(!std::isfinite(config_.startup_delay_s)||config_.startup_delay_s<0||config_.command_rate_hz>1000||config_.target_timeout_s<=1/config_.command_rate_hz)
  throw std::invalid_argument("Invalid startup delay/rate/target timeout");
 for(float target:config_.home_target)if(target!=0)throw std::invalid_argument("This deployment requires seven zero HOME targets");
 status_.state=config_.enabled?AutoHomeState::WAIT_DEVICE:AutoHomeState::READ_ONLY;
 if(config_.enabled&&!journal_.BlockReason().empty())Fault(journal_.BlockReason());
}
void AutoHomeController::Fault(const std::string& reason) {
 status_.state=AutoHomeState::FAULT_LATCHED;status_.last_error=reason;status_.arm_home_ready=false;status_.target_fresh=false;
 if(!journal_.RecordFault(reason))status_.last_error+="; journal_write_failed";
}
bool AutoHomeController::UsableFeedback() const {
 if(!status_.feedback_ready)return false;
 for(auto motor_status:status_.joints.error)if(motor_status!=0&&motor_status!=1)return false;
 return true;
}
void AutoHomeController::Observe(ArmTime now) {
 rars_arm::JointState state;
 const bool fresh=transport_.Read(state);
 status_.communication=transport_.Status();const auto& comm=status_.communication;
 if(fresh){status_.joints=state;last_read_=now;age_at_read_s_=comm.feedback_age.count()/1000.;}
 status_.feedback_age_s=last_read_?std::max(comm.feedback_age.count()/1000.,age_at_read_s_+Seconds(now,*last_read_)):-1;
 status_.feedback_ready=last_read_.has_value()&&comm.connected&&comm.feedback_received&&!comm.watchdog_tripped&&!comm.stm32_watchdog_tripped&&
  status_.feedback_age_s>=0&&status_.feedback_age_s<=config_.feedback_timeout_s;
 status_.motors_enabled=true;bool at_home=true;
 for(size_t i=0;i<7;++i){const auto& joints=status_.joints;
  status_.feedback_ready=status_.feedback_ready&&joints.valid[i]&&joints.motor_id[i]==i+1&&
   std::isfinite(joints.position[i])&&std::isfinite(joints.velocity[i])&&joints.error[i]<8;
  status_.motors_enabled=status_.motors_enabled&&joints.error[i]==1;
  status_.home_error[i]=joints.position[i]-config_.home_target[i];
  at_home=at_home&&std::isfinite(status_.home_error[i])&&std::abs(status_.home_error[i])<=config_.home_tolerance_rad;
 }
 status_.feedback_stamp=last_read_?std::optional<ArmTime>(now-Duration(status_.feedback_age_s)):std::nullopt;
 status_.target_age_s=status_.accepted_stamp?Seconds(now,*status_.accepted_stamp):-1;
 status_.target_fresh=status_.accepted_stamp&&status_.target_age_s>=0&&status_.target_age_s<=config_.target_timeout_s;
 status_.arm_home_ready=status_.state==AutoHomeState::HOLD_HOME&&comm.enabled&&status_.feedback_ready&&status_.motors_enabled&&status_.target_fresh&&at_home;
}
void AutoHomeController::Tick(ArmTime now,bool connect_allowed,bool command_timer_tick) {
 Observe(now);
 if(status_.state==AutoHomeState::FAULT_LATCHED){status_.arm_home_ready=false;status_.target_fresh=false;return;}
 if(!status_.communication.connected){
  if(status_.enable_attempted){Fault("runtime_disconnected");return;}
  countdown_.reset();last_read_.reset();status_.state=config_.enabled?AutoHomeState::WAIT_DEVICE:AutoHomeState::READ_ONLY;
  if(connect_allowed&&now>=next_connect_){next_connect_=now+Duration(config_.connect_retry_s);if(!transport_.Connect())status_.last_error="connect_failed: "+transport_.Error();}
  return;
 }
 if(!config_.enabled)return;
 if(!status_.enable_attempted){
  // Successful serial/receiver connection is startup communication. This STM
  // supplies motor feedback only after enable; requiring it here deadlocks boot.
  if(status_.communication.watchdog_tripped||status_.communication.stm32_watchdog_tripped){Fault("startup_watchdog_reported");return;}
  if(!countdown_)countdown_=now;
  status_.state=AutoHomeState::STARTUP_DELAY;status_.last_error.clear();
  if(Seconds(now,*countdown_)<config_.startup_delay_s)return;
  if(!journal_.RecordEnableAttempt()){Fault("cannot_persist_enable_attempt");return;}
  status_.enable_attempted=true;
  if(!transport_.Enable()){Fault("enable_failed: "+transport_.Error());return;}
  enabled_at_=now;status_.state=AutoHomeState::HOLD_HOME;next_send_=now;
  last_read_.reset();have_usable_feedback_=false;status_.feedback_ready=false;status_.arm_home_ready=false;
  // SDK enable can block for its protocol delay. Next callback starts the HOME
  // stream immediately, with readiness false until real feedback arrives.
  return;
 }
 if(UsableFeedback())have_usable_feedback_=true;
 const bool initial_feedback_wait=!have_usable_feedback_&&Seconds(now,*enabled_at_)<=config_.enable_feedback_grace_s;
 // SDK read() returns a USB payload even when motor IDs are still zero. Such
 // incomplete initial payloads may arrive before CAN replies or the first send.
 // Do not grant readiness, but permit zero streaming within initial grace.
 // A reported hardware fault, corrupt valid measurement or wrong nonzero ID
 // remains fatal immediately, even while waiting for the complete first frame.
 if(last_read_)for(size_t i=0;i<7;++i){
  const auto& j=status_.joints;
  if((j.motor_id[i]!=0&&j.motor_id[i]!=i+1)||
     (j.valid[i]&&(!std::isfinite(j.position[i])||!std::isfinite(j.velocity[i])||j.error[i]>1))){
   Fault("motor_feedback_reported_invalid");return;
  }
 }
 if(status_.communication.watchdog_tripped||status_.communication.stm32_watchdog_tripped){Fault("runtime_watchdog");return;}
 if(!UsableFeedback()&&!initial_feedback_wait){Fault("runtime_feedback_invalid_stale_or_watchdog");return;}
 // During bounded initial grace, stream HOME even without any feedback. This
 // matches SDK/GUI: first send arms the feedback watchdog; readiness stays false.
 if(!status_.communication.enabled){Fault("sdk_locally_disabled");return;}
 if(!status_.motors_enabled&&Seconds(now,*enabled_at_)>config_.enable_feedback_grace_s){Fault("motor_disabled_after_enable_grace");return;}
 if(status_.accepted_stamp&&!status_.target_fresh){Fault("home_target_stream_stale");return;}
 if(!status_.accepted_stamp&&Seconds(now,*enabled_at_)>config_.enable_feedback_grace_s){Fault("first_home_target_timeout");return;}
 if(command_timer_tick||now>=*next_send_){
  if(!transport_.Send(config_.home_target)){Fault("home_target_send_failed: "+transport_.Error());return;}
  if(status_.accepted_stamp){
   status_.last_send_gap_ms=Seconds(now,*status_.accepted_stamp)*1000;
   status_.max_send_gap_ms=std::max(status_.max_send_gap_ms,status_.last_send_gap_ms);
  }
  if(!status_.first_accepted_stamp)status_.first_accepted_stamp=now;
  ++status_.successful_sends;status_.accepted_stamp=now;
  const auto period=Duration(1/config_.command_rate_hz);
  // Retain the configured grid, skip missed slots without catch-up bursts.
  if(command_timer_tick)*next_send_=now+period;
  else {const auto slots=(now-*next_send_)/period+1;*next_send_+=period*slots;}
  status_.target_age_s=0;status_.target_fresh=true;
  // The sample remains measured; a send never substitutes HOME for q/dq.
  bool near=true;for(float error:status_.home_error)near=near&&std::isfinite(error)&&std::abs(error)<=config_.home_tolerance_rad;
  status_.arm_home_ready=status_.feedback_ready&&status_.motors_enabled&&near;
 }
}
} // namespace sim2real
