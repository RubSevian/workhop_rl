#pragma once
#include "rars_auto_home.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
namespace sim2real {
struct GraspAssessmentConfig {
 bool enabled=false;
 double contact_torque_nm=1.5,min_grasp_width_m=.05,confirmation_duration_s=.5;
 double radius=.0375,rod=.040,carriage=.030,angle_limit=1.,negative_tolerance=0.;
};
// Exact scalar port of RarsGripperGeometry.opening_width/linkage_angle_for_width.
// Same YAML geometry; no new jaw calibration. Parity against Python is tested.
inline double GripperInnerWidth(double q,const GraspAssessmentConfig& c){
 if(!std::isfinite(q)||q < -c.negative_tolerance||q>c.angle_limit+1e-9)throw std::invalid_argument("gripper measured angle invalid");
 q=std::max(0.,q);
 const double x=c.carriage/2.;
 const double cosine=(x*x+c.radius*c.radius-c.rod*c.rod)/(2*x*c.radius);
 if(cosine < -1.-1e-9||cosine>1.+1e-9)throw std::invalid_argument("gripper linkage invalid");
 const double theta=std::acos(std::clamp(cosine,-1.,1.))-q;
 return 2*(c.radius*std::cos(theta)+std::sqrt(std::max(0.,c.rod*c.rod-c.radius*c.radius*std::sin(theta)*std::sin(theta))))-c.carriage;
}
inline double TotalMitTorque(double kp,double q_des,double q,double kd,double dq_des,double dq,double ff){
 for(double v:{kp,q_des,q,kd,dq_des,dq,ff})if(!std::isfinite(v))throw std::invalid_argument("nonfinite MIT term");
 return kp*(q_des-q)+kd*(dq_des-dq)+ff;
}
// Host-side estimate only. No motor-side saturation/configuration API exists in
// this SDK/protocol; sampling q/dq cannot bound changes between host commands.
inline constexpr bool ControllerTotalTorqueCapSupported(){return false;}
class GraspAssessment {
 public:
 explicit GraspAssessment(GraspAssessmentConfig cfg={}):cfg_(cfg){
  for(double v:{cfg_.contact_torque_nm,cfg_.min_grasp_width_m,cfg_.confirmation_duration_s,cfg_.radius,cfg_.rod,cfg_.carriage,cfg_.angle_limit})if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid grasp assessment config");
  if(!std::isfinite(cfg_.negative_tolerance)||cfg_.negative_tolerance<0)throw std::invalid_argument("invalid torque direction/tolerance");
 }
 void Begin(ArmTime now){Reset();earliest_=now;}
 void Reset(){earliest_.reset();since_.reset();last_stamp_.reset();last_frame_=0;state_="UNKNOWN";reason_="NO_CLOSE_FEEDBACK";width_.reset();torque_.reset();}
 void Observe(ArmTime now,const AutoHomeStatus& s,bool active,bool complete,double ttl){
  if(!cfg_.enabled||!active){Reset();return;}
  const auto& comm=s.communication;const auto& j=s.joints;
  const bool valid=s.state==AutoHomeState::HOLD_HOME&&s.feedback_ready&&s.motors_enabled&&comm.connected&&comm.enabled&&comm.feedback_received&&!comm.watchdog_tripped&&!comm.stm32_watchdog_tripped&&s.feedback_stamp&&
   now>=*s.feedback_stamp&&std::chrono::duration<double>(now-*s.feedback_stamp).count()<=ttl&&j.valid[6]&&j.motor_id[6]==7&&j.error[6]==1&&std::isfinite(j.torque[6]);
  if(!valid){Invalidate("FEEDBACK_INVALID_OR_STALE");return;}
  if(earliest_&&*s.feedback_stamp<*earliest_){Invalidate("FEEDBACK_PREDATES_CLOSE");return;}
  try{width_=GripperInnerWidth(j.position[6],cfg_);}catch(const std::exception&){Invalidate("WIDTH_INVALID");return;}
  torque_=std::abs(j.torque[6]);
  // A read/host timer is not a new measurement. Use real received frame count
  // and source time; repeated snapshots never accumulate confirmation time.
  // SDK age has millisecond precision: cached-frame reconstructed stamps can
  // jitter backwards. Only a NEW frame can establish a source-time regression.
  if(comm.valid_frames==0||comm.valid_frames<last_frame_||(comm.valid_frames>last_frame_&&last_stamp_&&*s.feedback_stamp<*last_stamp_)){Invalidate("FEEDBACK_EPOCH_OR_COUNTER_INVALID");return;}
  const bool satisfied=*torque_>cfg_.contact_torque_nm&&*width_>=cfg_.min_grasp_width_m;
  if(!satisfied){since_.reset();state_=complete?"NOT_GRASPED":"UNKNOWN";reason_="TORQUE_OR_WIDTH_NOT_SATISFIED";}
  const bool new_sample=comm.valid_frames>last_frame_&&(!last_stamp_||*s.feedback_stamp>*last_stamp_);
  if(!new_sample){
   if(last_stamp_&&std::chrono::duration<double>(now-*last_stamp_).count()>ttl){Invalidate("NO_NEW_FEEDBACK_WITHIN_TTL");return;}
   if(complete&&state_!="GRASPED"){state_="NOT_GRASPED";reason_="CONFIRMATION_NOT_REACHED";}
   return;
  }
  if(last_stamp_&&std::chrono::duration<double>(*s.feedback_stamp-*last_stamp_).count()>ttl)since_.reset();
  last_stamp_=s.feedback_stamp;last_frame_=comm.valid_frames;
  if(satisfied){
   if(!since_)since_=s.feedback_stamp;
   const bool confirmed=std::chrono::duration<double>(*s.feedback_stamp-*since_).count()>=cfg_.confirmation_duration_s;
   state_=confirmed?"GRASPED":complete?"NOT_GRASPED":"UNKNOWN";reason_=confirmed?"TORQUE_WIDTH_CONFIRMED_PRELIMINARY":"CONFIRMATION_PENDING";
  }else{since_.reset();state_=complete?"NOT_GRASPED":"UNKNOWN";reason_="TORQUE_OR_WIDTH_NOT_SATISFIED";}
 }
 const std::string& state()const{return state_;}const std::string& reason()const{return reason_;}
 std::optional<double> width()const{return width_;}std::optional<double> torque()const{return torque_;}
 double confirmation_seconds()const{return since_&&last_stamp_?std::chrono::duration<double>(*last_stamp_-*since_).count():0.;}
 private:
 void Invalidate(const char* why){since_.reset();state_="UNKNOWN";reason_=why;width_.reset();torque_.reset();}
 GraspAssessmentConfig cfg_;std::optional<ArmTime> earliest_,since_,last_stamp_;uint64_t last_frame_=0;
 std::string state_="UNKNOWN",reason_="NO_CLOSE_FEEDBACK";std::optional<double> width_,torque_;
};
} // namespace sim2real
