#include "motion_trace.hpp"
#include <cmath>
#include <iomanip>
#include <stdexcept>
namespace sim2real {
MotionTrace::MotionTrace(const std::string& path,double duration_s):duration_s_(duration_s) {
 if(!std::isfinite(duration_s)||duration_s<=0||duration_s>600)throw std::invalid_argument("motion trace duration must be 0..600 seconds");
 file_.open(path,std::ios::out|std::ios::trunc);
 if(!file_)throw std::runtime_error("cannot open motion trace file: "+path);
 file_<<"kind,phase,accepted,steady_ns,packets,interval_s,io_min_s,io_max_s,low_age_s,remote_age_s,arm_age_s,target_age_s,compute_ms,job_age_s,result_age_s";
 for(const auto& field:{"sticks","requested","command","gyro"})for(int n=0;n<3;++n)file_<<','<<field<<n;
 for(int n=0;n<4;++n)file_<<",quat"<<n;
 for(const auto& field:{"q","dq","target","action","kp","kd"})for(int n=0;n<12;++n)file_<<','<<field<<n;
 file_<<'\n';file_.flush();writer_=std::thread([this]{Write();});
}
MotionTrace::~MotionTrace(){Stop();if(writer_.joinable())writer_.join();}
bool MotionTrace::Submit(const MotionTraceSample& sample) {
 if(!enabled())return false;
 std::unique_lock lock(queue_mutex_,std::try_to_lock);
 if(!lock.owns_lock()){++dropped_;return false;}
 if(!enabled())return false;
 const auto now=std::chrono::steady_clock::now();
 if(!started_){started_=true;first_=now;}
 if(std::chrono::duration<double>(now-first_).count()>=duration_s_){Stop();return false;}
 if(size_==queue_.size()){++dropped_;return false;}
 queue_[(head_+size_)%queue_.size()]=sample;++size_;return true;
}
void MotionTrace::Write() {
 file_<<std::setprecision(9);
 for(;;) {
  MotionTraceSample s;bool have=false;
  {
   std::lock_guard lock(queue_mutex_);
   if(started_&&std::chrono::duration<double>(std::chrono::steady_clock::now()-first_).count()>=duration_s_)Stop();
   if(size_){s=queue_[head_];head_=(head_+1)%queue_.size();--size_;have=true;}
  }
  if(have) {
   file_<<s.kind<<','<<s.phase<<','<<s.accepted<<','<<s.stamp_ns<<','<<s.packets<<','<<s.interval_s<<','<<s.io_min_s<<','<<s.io_max_s<<','<<s.low_age_s<<','<<s.remote_age_s<<','<<s.arm_age_s<<','<<s.target_age_s<<','<<s.compute_ms<<','<<s.job_age_s<<','<<s.result_age_s;
   auto values=[this](const auto& a){for(auto v:a)file_<<','<<v;};
   values(s.sticks);values(s.requested);values(s.command);values(s.gyro);values(s.quat);values(s.q);values(s.dq);values(s.target);values(s.action);values(s.kp);values(s.kd);file_<<'\n';
   if(!file_){Stop();return;}
  }else {
   if(!enabled())break;
   std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
 }
 file_.flush();file_.close();
}
}
