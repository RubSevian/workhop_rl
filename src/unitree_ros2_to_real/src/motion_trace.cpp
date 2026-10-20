#include "motion_trace.hpp"
#include <cmath>
#include <iomanip>
#include <stdexcept>
namespace sim2real {
MotionTrace::MotionTrace(const std::string& path,double duration_s,size_t queue_capacity,int writer_idle_ms,double max_duration_s):writer_idle_ms_(writer_idle_ms),duration_s_(duration_s) {
 if(!std::isfinite(max_duration_s)||max_duration_s<=0||max_duration_s>600||!std::isfinite(duration_s)||duration_s<=0||duration_s>max_duration_s||queue_capacity<1||queue_capacity>4096||writer_idle_ms<1||writer_idle_ms>1000)throw std::invalid_argument("Invalid bounded motion trace configuration");
 capacity_=queue_capacity;queue_=std::make_unique<Cell[]>(capacity_);
 for(size_t i=0;i<capacity_;++i)queue_[i].sequence.store(i,std::memory_order_relaxed);
 file_.open(path,std::ios::out|std::ios::trunc);
 if(!file_)throw std::runtime_error("cannot open motion trace file: "+path);
 file_<<"kind,phase,accepted,steady_ns,packets,interval_s,io_min_s,io_max_s,low_age_s,remote_age_s,arm_age_s,target_age_s,compute_ms,job_age_s,result_age_s";
 for(const auto& field:{"sticks","requested","command","gyro"})for(int n=0;n<3;++n)file_<<','<<field<<n;
 for(int n=0;n<4;++n)file_<<",quat"<<n;
 for(const auto& field:{"q","dq","target","action","kp","kd"})for(int n=0;n<12;++n)file_<<','<<field<<n;
 file_<<",prepare_ms,compute_cpu_ms,postprocess_ms,result_lock_wait_ms,callback_lock_wait_ms,callback_lock_hold_ms,graph_ms";
 file_<<'\n';file_.flush();writer_=std::thread([this]{Write();});
}
MotionTrace::~MotionTrace(){Stop();if(writer_.joinable())writer_.join();}
bool MotionTrace::Submit(const MotionTraceSample& sample) {
 // Fixed-storage bounded MPSC queue, as used by ControlLatency. A consumer
 // copying a row can no longer cause a false timing gap through try_lock.
 struct Active {std::atomic<size_t>& n;Active(std::atomic<size_t>& v):n(v){n.fetch_add(1);}~Active(){n.fetch_sub(1);}} active(submitting_);
 if(!enabled())return false;
 const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
 int64_t zero=0;first_ns_.compare_exchange_strong(zero,now);
 if((now-first_ns_.load())/1.e9>=duration_s_){Stop();return false;}
 auto pos=tail_.load(std::memory_order_relaxed);
 for(int attempt=0;attempt<16;++attempt){
  auto& cell=queue_[pos%capacity_];const auto seq=cell.sequence.load(std::memory_order_acquire);
  if(seq==pos){
   if(tail_.compare_exchange_weak(pos,pos+1,std::memory_order_relaxed)){
    cell.sample=sample;cell.sequence.store(pos+1,std::memory_order_release);return true;
   }
  }else if(seq<pos)break;else pos=tail_.load(std::memory_order_relaxed);
 }
 ++dropped_;return false;
}
void MotionTrace::Write() {
 file_<<std::setprecision(9);
 for(;;) {
  MotionTraceSample s;bool have=false;
  const auto first=first_ns_.load();
  const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if(first&&(now-first)/1.e9>=duration_s_)Stop();
  auto& cell=queue_[head_%capacity_];
  if(cell.sequence.load(std::memory_order_acquire)==head_+1){
   s=cell.sample;cell.sequence.store(head_+capacity_,std::memory_order_release);++head_;have=true;
  }
  if(have) {
   file_<<s.kind<<','<<s.phase<<','<<s.accepted<<','<<s.stamp_ns<<','<<s.packets<<','<<s.interval_s<<','<<s.io_min_s<<','<<s.io_max_s<<','<<s.low_age_s<<','<<s.remote_age_s<<','<<s.arm_age_s<<','<<s.target_age_s<<','<<s.compute_ms<<','<<s.job_age_s<<','<<s.result_age_s;
   auto values=[this](const auto& a){for(auto v:a)file_<<','<<v;};
   values(s.sticks);values(s.requested);values(s.command);values(s.gyro);values(s.quat);values(s.q);values(s.dq);values(s.target);values(s.action);values(s.kp);values(s.kd);
   file_<<','<<s.prepare_ms<<','<<s.compute_cpu_ms<<','<<s.postprocess_ms<<','<<s.result_lock_wait_ms<<','<<s.callback_lock_wait_ms<<','<<s.callback_lock_hold_ms<<','<<s.graph_ms<<'\n';
   if(!file_){Stop();return;}
  }else {
   if(!enabled()&&submitting_.load()==0&&head_==tail_.load())break;
   std::this_thread::sleep_for(std::chrono::milliseconds(writer_idle_ms_));
  }
 }
 file_.flush();file_.close();
}
}
