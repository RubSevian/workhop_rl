#pragma once
#include "grasp_assessment.hpp"
namespace sim2real {
// A sampled contact regulator, NOT a motor-side or instantaneous torque limit.
struct MeasuredGripperConfig {
 double kp=20,kd=1,close_speed_rad_s=.15,contact_torque_nm=2.2;
 double hold_gain_rad_s_per_nm=.05,hold_deadband_nm=.1,max_target_lead_rad=.15;
};
class MeasuredGripperControl {
 public:
 MeasuredGripperControl(MeasuredGripperConfig c,GraspAssessmentConfig geometry,double closed,double open,double rate,double ttl)
 :cfg_(c),guard_(Enabled(geometry)),closed_(closed),open_(open),rate_(rate),ttl_(ttl){
  for(double v:{c.kp,c.kd,c.close_speed_rad_s,c.contact_torque_nm,c.hold_gain_rad_s_per_nm,c.hold_deadband_nm,c.max_target_lead_rad,rate,ttl})
   if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid measured gripper control config");
  if(!std::isfinite(closed)||!std::isfinite(open)||closed==open||c.hold_deadband_nm>=c.contact_torque_nm)
   throw std::invalid_argument("invalid measured gripper range/deadband");
 }
 void Begin(ArmTime now,const AutoHomeStatus& s){
  guard_.Reset();active_=true;failed_=contact_=false;frame_=0;stamp_.reset();reason_.clear();
  target_=std::clamp(double(s.joints.position[6]),std::min(closed_,open_),std::max(closed_,open_));
 }
 void Stop(){active_=false;}
 void CaptureClosedStop(){target_=closed_;}
 bool Update(ArmTime now,const AutoHomeStatus& s,bool authority,bool hold){
  if(!active_||failed_)return !failed_;
  guard_.Observe(now,s,true,false,ttl_);
  if(!authority||!guard_.torque()){
   failed_=true;active_=false;reason_=authority?guard_.reason():"AUTHORITY_REVOKED";return false;
  }
  // Updating the same received frame never integrates more closing motion.
  if(s.communication.valid_frames<=frame_)return true;
  const double dt=stamp_?std::clamp(std::chrono::duration<double>(*s.feedback_stamp-*stamp_).count(),0.,1./rate_):0.;
  stamp_=s.feedback_stamp;frame_=s.communication.valid_frames;
  const double tau=*guard_.torque(),q=s.joints.position[6],direction=closed_<open_?-1.:1.;
  if(tau>=cfg_.contact_torque_nm)contact_=true;
  double speed=cfg_.close_speed_rad_s;
  if(contact_||hold){
   const double error=cfg_.contact_torque_nm-tau;
   speed=std::abs(error)<=cfg_.hold_deadband_nm?0.:std::clamp(cfg_.hold_gain_rad_s_per_nm*error,-cfg_.close_speed_rad_s,cfg_.close_speed_rad_s);
  }
  // On a threshold sample there is no further closing increment; overload
  // unwinds the target at bounded speed. Only fresh feedback may resume it.
  if(tau>=cfg_.contact_torque_nm)speed=std::min(0.,speed);
  if(speed>0&&dt>0)speed=std::min(speed,std::max(0.,cfg_.max_target_lead_rad-direction*(target_-q))/dt);
  target_=std::clamp(target_+direction*speed*dt,std::min(closed_,open_),std::max(closed_,open_));
  return true;
 }
 bool active()const{return active_;}bool failed()const{return failed_;}bool contact()const{return contact_;}
 double target()const{return target_;}const std::string& reason()const{return reason_;}
 const MeasuredGripperConfig& config()const{return cfg_;}
 private:
 static GraspAssessmentConfig Enabled(GraspAssessmentConfig c){c.enabled=true;return c;}
 MeasuredGripperConfig cfg_;GraspAssessment guard_;double closed_,open_,rate_,ttl_,target_=0;
 bool active_=false,failed_=false,contact_=false;uint64_t frame_=0;std::optional<ArmTime> stamp_;std::string reason_;
};
}
