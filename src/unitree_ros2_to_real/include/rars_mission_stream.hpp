#pragma once
#include "rars_auto_home.hpp"
#include "grasp_assessment.hpp"
#include "measured_gripper_control.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace sim2real {
// Trajectory preparation has no heartbeat TTL. Cancellation is a time fence,
// including queued HOME plans that otherwise have controlled-stop authority.
inline bool PlanTimestampAllowed(int64_t observed_ns,int64_t now_ns,int64_t cancelled_ns){
 return observed_ns>0&&observed_ns<=now_ns&&observed_ns>cancelled_ns;
}
inline bool ReadyObservePlanAllowed(bool enabled,bool initial_move_seen,const std::string& kind){
 return !enabled||kind=="home"||(kind=="move"&&!initial_move_seen);
}
inline bool PresetSearchPlanAllowed(const std::string& operation,const std::string& phase){
 return (operation=="scan_step"&&phase=="ARM_SEARCH")||(operation=="scan_return"&&phase=="RETURN_HOME");
}
inline bool ControlledStopHomePhaseAllowed(const std::string& phase){
 return phase=="ARM_RETURN_HOME"||phase=="ARM_HOME_BLOCKED"||phase=="ARM_HOME_SETTLE";
}
// An approximate open-jaw collision result is not physical clearance evidence.
// Use both producer operation and owner health/hold facts, not message metadata alone.
inline bool PhysicalRetreatPlanAllowed(bool qualified,const std::string& operation,
                                      const std::string& phase,bool holding,const std::string& kind){
 return qualified||!(operation=="retreat"||phase=="RETREAT"||(holding&&kind=="move"));
}
// No SDK, ROS, inference, IK or collision work in this bounded command sampler.
// Admission is owned by ArmOwner; every tick rechecks its authority lease.
struct MissionStreamConfig {
 std::array<double,6> lower{},upper{},velocity{};
 std::optional<std::array<double,6>> search_velocity;
 rars_arm::RarsArm::MotorValues kp{},kd{};
 double gripper_velocity=2,rate=100, start_tolerance=.02, reached_tolerance=.02, settled_velocity=.035;
 double settle_s=.2, completion_timeout_s=5.5, max_duration_s=60;
 double closed=0,open=1;
 MeasuredGripperConfig gripper_control;
 double move_kp=20,move_kd=1;
 double close_duration_s=8.,hard_stop=.01;
 GraspAssessmentConfig assessment;double feedback_ttl_s=.1;
};
class MissionStream {
 public:
 explicit MissionStream(MissionStreamConfig config):cfg_(config),assessment_(config.assessment),gripper_(config.gripper_control,config.assessment,config.closed,config.open,config.rate,config.feedback_ttl_s){
  if(config.gripper_control.close_speed_rad_s>config.gripper_velocity)throw std::invalid_argument("closing speed exceeds SDK limit");
  for(size_t j=0;j<6;++j)if(!std::isfinite(cfg_.lower[j])||!std::isfinite(cfg_.upper[j])||cfg_.lower[j]>=cfg_.upper[j]||!std::isfinite(cfg_.velocity[j])||cfg_.velocity[j]<=0)throw std::invalid_argument("invalid mission joint limits");
  for(double value:{cfg_.rate,cfg_.gripper_velocity,cfg_.start_tolerance,cfg_.reached_tolerance,cfg_.max_duration_s,cfg_.settle_s,cfg_.completion_timeout_s,cfg_.close_duration_s})if(!std::isfinite(value)||value<=0)throw std::invalid_argument("invalid mission stream setting");
  held_.kp=cfg_.kp;held_.kd=cfg_.kd;
  if(cfg_.search_velocity)for(double v:*cfg_.search_velocity)if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid search velocity");
 }
 bool Start(uint64_t id,const std::string& kind,const std::vector<std::array<float,7>>& points,
            ArmTime now,const AutoHomeStatus& status,bool search=false){
  if(search&&!cfg_.search_velocity)return false;
  if(id<=last_id_||busy_||!status.feedback_ready||!status.motors_enabled||!status.target_fresh)return false;
  if(kind!="move"&&kind!="home"&&kind!="close"&&kind!="release")return false;
  if(points.size()<2||points.size()>size_t(cfg_.rate*cfg_.max_duration_s)+1)return false;
  if(kind=="close"&&holding_)return false;
  const bool release=kind=="release";
  if(release&&(!cfg_.assessment.enabled||assessment_.state()!="NOT_GRASPED"||!holding_))return false;
  for(size_t n=0;n<points.size();++n)for(size_t j=0;j<7;++j){
   const double q=points[n][j];if(!std::isfinite(q))return false;
   if(j<6&&(q<cfg_.lower[j]||q>cfg_.upper[j]))return false;
   if(j==6&&(q<std::min(cfg_.closed,cfg_.open)||q>std::max(cfg_.closed,cfg_.open)))return false;
   if(n==0&&!(holding_&&j==6)&&std::abs(q-status.joints.position[j])>cfg_.start_tolerance)return false;
   if(n&&j<6&&std::abs(q-points[n-1][j])*cfg_.rate>(search?(*cfg_.search_velocity)[j]:cfg_.velocity[j])+1e-5)return false;
   if(n&&j==6&&std::abs(q-points[n-1][j])*cfg_.rate>cfg_.gripper_velocity+1e-5)return false;
   if(holding_&&!release&&j==6&&((n==0&&std::abs(q-held_.q[6])>cfg_.start_tolerance)||(n&&std::abs(q-points[0][6])>1e-5)))return false;
   if((kind=="close"||release)&&j<6&&std::abs(q-points[0][j])>1e-5)return false;
  }
  if(release&&(cfg_.open>cfg_.closed?1.:-1.)*(points.back()[6]-status.joints.position[6]) < -cfg_.start_tolerance)return false;
  if(kind=="home")for(size_t j=0;j<6;++j)if(std::abs(points.back()[j])>1e-6)return false;
  if(release){holding_=false;gripper_.Stop();}
  if(kind=="close"){assessment_.Begin(now);gripper_.Begin(now,status);}
  Capture(status);last_id_=id;kind_=kind;points_=points;started_=now;settled_.reset();
  next_index_=0;busy_=true;state_="EXECUTING";error_.clear();return true;
 }
 void Cancel(const AutoHomeStatus& status,const std::string& reason){
  if(status.feedback_ready)Capture(status);
  gripper_.Stop();busy_=false;points_.clear();settled_.reset();state_="CANCELLED";error_=reason;
 }
 AutoHomeController::Command Tick(ArmTime now,const AutoHomeStatus& status,bool authority){
  if(!initialized_)Capture(status);
  ObserveAssessment(now,status);
  
  if(gripper_.active()&&!gripper_.Update(now,status,authority,holding_)){
   // With untrustworthy contact feedback, remove positional closing effort.
   // Do not automatically resume this operation when feedback returns.
   if(!authority&&status.feedback_ready)Capture(status);
   holding_=false;gripper_.Stop();busy_=false;points_.clear();settled_.reset();
   error_="gripper_control:"+gripper_.reason();assessment_.Reset();
   held_.kp[6]=0;held_.kd[6]=cfg_.gripper_control.kd;held_.torque[6]=0;
   state_=authority?"FAILED":"CANCELLED";return held_;
  }
  if(busy_&&!authority)Cancel(status,"authority_revoked");
  if(gripper_.active())ApplyGripper();
  if(!busy_)return held_;
  const double elapsed=std::chrono::duration<double>(now-started_).count();
  // No burst/catch-up sends after a
  // callback delay: advance at most one checked sample, without a new 20ms
  // cancellation threshold. Existing SDK feedback/authority watchdogs remain.
  const auto index=std::min({points_.size()-1,size_t(std::max(0.,elapsed)*cfg_.rate),next_index_});
  next_index_=index+1;
  held_.q=points_[index];
  if(gripper_.active())ApplyGripper();
  if(kind_=="close"){
   const double sign=cfg_.open>cfg_.closed?1:-1;
   const bool at_stop=sign*(status.joints.position[6]-cfg_.closed)<cfg_.hard_stop;
   // Contact switches to regulation immediately, but leaves the confirmation
   // interval available before reporting a completed close to Mission.
   if(at_stop){gripper_.CaptureClosedStop();ApplyGripper();}
   if(gripper_.contact())holding_=true;
   if(at_stop||elapsed>=cfg_.close_duration_s||(holding_&&cfg_.assessment.enabled&&assessment_.state()=="GRASPED")){
    holding_=true;busy_=false;state_="SUCCEEDED";
   }
  }else if(index==points_.size()-1){
   bool reached=true;for(size_t j=0;j<((holding_||kind_=="home")?6u:7u);++j)reached=reached&&std::abs(status.joints.position[j]-held_.q[j])<=cfg_.reached_tolerance&&std::abs(status.joints.velocity[j])<=cfg_.settled_velocity;
   if(reached){if(!settled_)settled_=now;
    if(std::chrono::duration<double>(now-*settled_).count()>=cfg_.settle_s){busy_=false;state_="SUCCEEDED";}
   }else settled_.reset();
  }
  if(busy_&&elapsed>(kind_=="close"?cfg_.close_duration_s:points_.size()/cfg_.rate)+cfg_.completion_timeout_s){Cancel(status,"trajectory_feedback_timeout");state_="FAILED";}
  return held_;
 }
 bool home6(const AutoHomeStatus& status,double tolerance)const{
  if(!initialized_)return status.arm_home_ready;
  if(busy_||!status.feedback_ready||!status.motors_enabled||!status.target_fresh)return false;
  for(size_t j=0;j<6;++j)if(std::abs(status.joints.position[j])>tolerance||std::abs(held_.q[j])>tolerance)return false;
  return true;
 }
 // holding() means a gripper command is retained; it is NOT object evidence.
 bool HoldConfirmed(const AutoHomeStatus& status)const{return cfg_.assessment.enabled&&status.feedback_ready&&assessment_.state()=="GRASPED";}
 void ObserveAssessment(ArmTime now,const AutoHomeStatus& status){
  if((kind_=="close"&&(state_=="EXECUTING"||state_=="SUCCEEDED"))||(holding_&&kind_!="release"))assessment_.Observe(now,status,true,holding_,cfg_.feedback_ttl_s);
  else if(kind_!="release")assessment_.Reset();
 }
 double contact_target_nm()const{return cfg_.gripper_control.contact_torque_nm;}
 const GraspAssessment& assessment()const{return assessment_;}
 const std::string& operation()const{return kind_;}
 bool has_command()const{return initialized_;}
 bool busy()const{return busy_;}bool holding()const{return holding_;}
 uint64_t id()const{return last_id_;}const std::string& state()const{return state_;}
 const std::string& error()const{return error_;}
 const AutoHomeController::Command& command()const{return held_;}
 private:
 void Capture(const AutoHomeStatus& s){
  const float grip=held_.q[6];held_.q=s.joints.position;
  if(holding_)held_.q[6]=grip;else {held_.kp[6]=cfg_.move_kp;held_.kd[6]=cfg_.move_kd;held_.torque[6]=0;}
  initialized_=true;
 }
 void ApplyGripper(){held_.q[6]=gripper_.target();held_.kp[6]=cfg_.gripper_control.kp;held_.kd[6]=cfg_.gripper_control.kd;held_.torque[6]=0;}
 MissionStreamConfig cfg_;GraspAssessment assessment_;MeasuredGripperControl gripper_;AutoHomeController::Command held_;
 std::vector<std::array<float,7>> points_;
 ArmTime started_{};std::optional<ArmTime> settled_;
 bool initialized_=false,busy_=false,holding_=false;uint64_t last_id_=0;
 size_t next_index_=0;
 std::string kind_,state_="IDLE",error_;
};
} // namespace sim2real
