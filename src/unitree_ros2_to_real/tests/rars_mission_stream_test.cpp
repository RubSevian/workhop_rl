#include "rars_mission_stream.hpp"
#include "arm_status_snapshot.hpp"
#include <cassert>
#include <iostream>
using namespace sim2real;
ArmTime at(double s){return ArmTime(std::chrono::duration_cast<ArmClock::duration>(std::chrono::duration<double>(s)));}
// This legacy sampler suite supplies explicit healthy synthetic SDK frames.
// Invalid/stale/cached feedback is tested in measured_gripper_control_test.
class FreshStream : public MissionStream {
 public:using MissionStream::MissionStream;
 AutoHomeController::Command Tick(ArmTime now,const AutoHomeStatus& input,bool authority){
  auto s=input;s.state=AutoHomeState::HOLD_HOME;s.feedback_stamp=now;
  s.joints.valid.fill(true);s.joints.error.fill(1);for(int i=0;i<7;++i)s.joints.motor_id[i]=i+1;
  s.joints.torque[6]=2.2F;s.communication.connected=s.communication.enabled=s.communication.feedback_received=true;
  s.communication.valid_frames=++frames_;return MissionStream::Tick(now,s,authority);
 }
 private:uint64_t frames_=0;
};
int main(){
 assert(PresetSearchPlanAllowed("refine","VIEW_REFINEMENT"));
 assert(!PresetSearchPlanAllowed("refine","GRASP"));
 {
  std::vector<std::array<float,7>> points(2,std::array<float,7>{});points[1][0]=.3F;
  assert(UncheckedSearchPlanAllowed("preset_search_unchecked","refine","VIEW_REFINEMENT","move",false,points));
  assert(!UncheckedSearchPlanAllowed("preset_search_unchecked","refine","VIEW_REFINEMENT","move",true,points));
  points[1][1]=.01F;
  assert(!UncheckedSearchPlanAllowed("preset_search_unchecked","refine","VIEW_REFINEMENT","move",false,points));
 }

 ScanHomeReference scan_ref;std::array<float,7> measured{0,.5,.2,.1,0,0,.4};
 assert(!scan_ref.Matches(measured,.025));
 scan_ref.Begin("scan_pose");scan_ref.Observe("EXECUTING",measured);assert(!scan_ref.Matches(measured,.025));
 scan_ref.Observe("SUCCEEDED",measured);assert(scan_ref.Matches(measured,.025));
 scan_ref.Observe("CANCELLED",measured);assert(scan_ref.Matches(measured,.025));
 auto moved=measured;moved[1]+=.03;assert(!scan_ref.Matches(moved,.025));
 assert(UncheckedScanHomeAllowed("scan_to_home_unchecked","home","BLOCKED","home",true,"CANCELLED",true));
 assert(!UncheckedScanHomeAllowed("scan_to_home_unchecked","home","BLOCKED","home",true,"CANCELLED",false));
 scan_ref.Begin("scan_step");scan_ref.Observe("CANCELLED",measured);assert(!scan_ref.Matches(measured,.025));
 scan_ref.Begin("refine");scan_ref.Observe("SUCCEEDED",measured);assert(scan_ref.Matches(measured,.025));
 scan_ref.Begin("pregrasp");scan_ref.Observe("SUCCEEDED",measured);assert(!scan_ref.Matches(measured,.025));
 scan_ref.Begin("retreat",true);scan_ref.Observe("SUCCEEDED",measured);assert(scan_ref.Matches(measured,.025));
 scan_ref.Reset();assert(!scan_ref.Matches(measured,.025));
 assert(UncheckedScanHomeAllowed("scan_to_home_unchecked","home","RETURN_HOME","home",true,"SUCCEEDED"));
 assert(!UncheckedScanHomeAllowed("scan_to_home_unchecked","home","RETURN_HOME","home",false,"SUCCEEDED"));
 assert(!UncheckedScanHomeAllowed("scan_to_home_unchecked","home","RETURN_HOME","home",true,"EXECUTING"));
 assert(!UncheckedScanHomeAllowed("scan_to_home_unchecked","home","RETREAT","home",true,"SUCCEEDED"));
 assert(!UncheckedScanHomeAllowed("scan_to_home_unchecked","home","RETURN_HOME","move",true,"SUCCEEDED"));

 for(const auto* phase:{"ARM_RETURN_HOME","ARM_HOME_BLOCKED","ARM_HOME_SETTLE"})assert(ControlledStopHomePhaseAllowed(phase));
 for(const auto* phase:{"CANCEL_TASKS","PD_CAPTURE","PD_HOLD_CONFIRMED","ARM_DISABLE","LIE_DOWN","EMERGENCY_FAULT",""})assert(!ControlledStopHomePhaseAllowed(phase));
 assert(ReadyObservePlanAllowed(true,false,"move"));
 assert(!ReadyObservePlanAllowed(true,true,"move"));
 assert(!ReadyObservePlanAllowed(true,false,"close"));
 assert(!ReadyObservePlanAllowed(true,true,"close"));
 assert(ReadyObservePlanAllowed(true,true,"home"));
 assert(ReadyObservePlanAllowed(false,true,"close"));

 // Parsing may exceed 250ms or several seconds while the previous target holds.
 assert(PlanTimestampAllowed(1'000'000'000,1'330'000'000,0));
 assert(PlanTimestampAllowed(1'000'000'000,61'000'000'000,0));
 assert(!PlanTimestampAllowed(0,2'000'000'000,0));
 assert(!PlanTimestampAllowed(3'000'000'000,2'000'000'000,0));
 // Cancel rejects an in-flight/queued plan, including HOME. A freshly requested
 // controlled-stop HOME can pass this fence; live authority is checked by owner.
 assert(!PlanTimestampAllowed(1'000'000'000,3'000'000'000,2'000'000'000));
 assert(!PlanTimestampAllowed(2'000'000'000,3'000'000'000,2'000'000'000));
 assert(PlanTimestampAllowed(2'100'000'000,3'000'000'000,2'000'000'000));

 // Cancelling at measured HOME captures encoder quantization, not exact zero.
 {
  MissionStreamConfig c;c.lower.fill(-3);c.upper.fill(3);c.velocity.fill(1);
  MissionStream home_stream(c);AutoHomeStatus s;
  s.feedback_ready=s.motors_enabled=s.target_fresh=true;
  s.joints.position.fill(-0.000190734863F);
  home_stream.Cancel(s,"operator_cancel");
  assert(home_stream.home6(s,.02));
  s.joints.position[0]=.021F;
  assert(!home_stream.home6(s,.02));
  home_stream.Cancel(s,"operator_cancel");
  s.joints.position[0]=0; // Measured HOME cannot hide an out-of-HOME target.
  assert(!home_stream.home6(s,.02));
  s.joints.position.fill(0);home_stream.Cancel(s,"operator_cancel");
  s.feedback_ready=false;assert(!home_stream.home6(s,.02));
  s.feedback_ready=true;s.motors_enabled=false;assert(!home_stream.home6(s,.02));
  s.motors_enabled=true;s.target_fresh=false;assert(!home_stream.home6(s,.02));
 }
 MissionStreamConfig cfg;cfg.close_duration_s=.5;cfg.lower.fill(-3);cfg.upper.fill(3);cfg.velocity.fill(1);cfg.kp.fill(20);cfg.kd.fill(1);
 FreshStream stream(cfg);AutoHomeStatus feedback;feedback.feedback_ready=feedback.motors_enabled=feedback.target_fresh=true;
 feedback.joints.position.fill(0);feedback.joints.velocity.fill(0);feedback.joints.position[6]=.8;
 std::array<float,7> q=feedback.joints.position;std::vector<std::array<float,7>> points(101,q);
 for(size_t i=0;i<points.size();++i)points[i][0]=.1*i/100;
 assert(stream.Start(1,"move",points,at(1),feedback));
 for(int n=0;n<140;++n){auto command=stream.Tick(at(1+n*.01),feedback,true);feedback.joints.position=command.q;}
 assert(stream.state()=="SUCCEEDED");assert(!stream.Start(1,"move",points,at(3),feedback));
 // Stub completes a bounded close command and never certifies an object.
 q=feedback.joints.position;std::vector<std::array<float,7>> close(2,q);
 assert(stream.Start(2,"close",close,at(3),feedback));
 for(int n=0;n<70;++n){feedback.joints.position[6]=n<40?float(.8-n*.01):.4f;
  feedback.joints.velocity[6]=n<40?-.5f:0;stream.Tick(at(3+n*.01),feedback,true);}
 assert(stream.holding()&&stream.state()=="SUCCEEDED"&&!stream.HoldConfirmed(feedback));
 assert(stream.command().torque[6]==0&&stream.command().kp[6]==cfg.gripper_control.kp);
 const auto grip=stream.command().q[6];
 points.assign(101,feedback.joints.position);
 for(size_t i=0;i<points.size();++i){points[i][0]=.1*(1-i/100.);points[i][6]=grip;}
 assert(stream.Start(3,"home",points,at(4),feedback));
 for(int n=0;n<140;++n){auto command=stream.Tick(at(4+n*.01),feedback,true);feedback.joints.position=command.q;feedback.joints.position[6]=0;feedback.joints.velocity.fill(0);}
 assert(stream.home6(feedback,.02)&&stream.holding()&&stream.command().q[6]==grip);
 // Simulated loss: HOME completed with q7=0 while retaining the grip command.
 assert(!stream.HoldConfirmed(feedback));
 // Cancel during motion freezes the arm and preserves the gripper command.
 points.assign(101,feedback.joints.position);for(size_t i=0;i<points.size();++i){points[i][1]=.1*i/100;points[i][6]=grip;}
 assert(stream.Start(4,"move",points,at(6),feedback));
 stream.Tick(at(6),feedback,true);feedback.joints.position[1]=.005;
 const auto frozen=stream.Tick(at(6.01),feedback,false);
 assert(!stream.busy()&&stream.state()=="CANCELLED"&&frozen.q[1]==feedback.joints.position[1]&&frozen.q[6]==grip);
 // Neither joint NaNs, unbounded velocity nor opening a held gripper is accepted.
 points[0]=feedback.joints.position;points[1][0]=10;
 assert(!stream.Start(5,"move",points,at(7),feedback));
 points[1]=points[0];points[1][6]=grip+.1;
 assert(!stream.Start(5,"move",points,at(7),feedback));
 points[1]=points[0];points[1][0]=NAN;
 assert(!stream.Start(5,"move",points,at(7),feedback));
 // The 7th joint can be nonzero only under an explicit confirmed-hold contract.
 YAML::Node y;const auto now=at(9);const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
 y["observed_ns"]=y["frame_received_ns"]=y["accepted_ns"]=ns;y["common_feedback_age_s"]=0.;
 y["feedback_ready"]=y["static_hold"]=y["arm_home_ready"]=y["connected"]=y["enabled_local"]=y["target_valid"]=true;
 y["watchdog_tripped"]=y["stm32_watchdog_tripped"]=false;y["owner_state"]="HOLD_HOME";
 y["measured_q"]=y["measured_dq"]=y["accepted_target"]=std::vector<float>(6,0);
 y["motor_ids"]=std::vector<int>{1,2,3,4,5,6};y["valid"]=std::vector<bool>(6,true);
 y["motor_id"]=std::vector<int>{1,2,3,4,5,6,7};y["motor_status"]=std::vector<int>(7,1);y["valid7"]=std::vector<bool>(7,true);
 y["q"]=std::vector<float>{0,0,0,0,0,0,grip};y["dq"]=y["home_target"]=std::vector<float>(7,0);
 assert(!ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["gripper_hold_confirmed"]=true;y["gripper_hold_target"]=grip;
 auto parsed=ParseArmStatus(YAML::Dump(y),.15,now);assert(parsed.error.empty()&&parsed.inputs.arm_home_ready);
 // New mission HOME6 contract does not require evidence or closed jaws.
 y["gripper_hold_confirmed"]=false;y["home_joint_count"]=6;y["mission_capable"]=true;y["grasp_verification"]="stub";
 y["grasp_verification"]="torque_width";
 y["q"]=std::vector<float>{0,0,0,0,0,0,.95f};
 assert(ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["q"]=std::vector<float>{0,0,0,0,0,0,0};assert(ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["q"]=std::vector<float>{0,0,0,0,0,0,NAN};assert(!ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["q"]=std::vector<float>{.3,0,0,0,0,0,0};assert(!ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 // Empty/closed gripper finishes normally; a stalled open gripper is also
 // unverified after the timer. Neither case claims object presence.
 for(float final_grip:{0.f,.8f}){
  FreshStream stub(cfg);AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.position[6]=.8;
  std::vector<std::array<float,7>> close_points(2,f.joints.position);
  assert(stub.Start(1,"close",close_points,at(20),f));
  for(int n=0;n<70;++n){f.joints.position[6]=final_grip;f.joints.velocity.fill(0);stub.Tick(at(20+n*.01),f,true);}
  assert(stub.state()=="SUCCEEDED"&&stub.holding()&&!stub.HoldConfirmed(f));
  if(final_grip==0)assert(stub.command().torque[6]==0);
 }
 // Closing publishes a final non-grasp status immediately at either zero-band edge.
 {
  auto c=cfg;c.close_duration_s=8.;c.assessment.enabled=true;
  c.assessment.negative_tolerance=.025;c.assessment.confirmation_duration_s=2.;
  for(float q:{-.025F,0.F,.025F}){
   FreshStream empty(c);auto f=feedback;f.joints.position[6]=.8;
   assert(empty.Start(1,"close",std::vector<std::array<float,7>>(2,f.joints.position),at(200),f));
   f.joints.position[6]=q;empty.Tick(at(200.01),f,true);
   assert(empty.state()=="SUCCEEDED");assert(empty.assessment().state()=="NOT_GRASPED");
   // Full second attempt: HOME preserves holding; release must precede scan.
   f.joints.position.fill(0);f.joints.velocity.fill(0);
   auto home_points=std::vector<std::array<float,7>>(2,f.joints.position);
   for(auto& point:home_points)point[6]=empty.command().q[6];
   assert(empty.Start(2,"home",home_points,at(201),f));
   for(int n=0;n<50;++n)empty.Tick(at(201+n*.01),f,true);
   assert(empty.state()=="SUCCEEDED"&&empty.holding());
   auto release_points=std::vector<std::array<float,7>>(401,f.joints.position);
   for(size_t n=0;n<release_points.size();++n)release_points[n][6]=.8F*n/400;
   assert(!empty.Start(3,"move",release_points,at(202),f));
   assert(empty.Start(3,"release",release_points,at(202),f));
   assert(!empty.holding());
   for(int n=0;n<450;++n){f.joints.position[6]=.8F*std::min(n,400)/400;empty.Tick(at(202+n*.01),f,true);}
   assert(empty.state()=="SUCCEEDED"&&!empty.holding());
   auto scan_points=std::vector<std::array<float,7>>(2,f.joints.position);
   assert(empty.Start(4,"move",scan_points,at(207),f));

  }
  FreshStream contact(c);auto f=feedback;f.joints.position[6]=.7;
  assert(contact.Start(1,"close",std::vector<std::array<float,7>>(2,f.joints.position),at(210),f));
  for(int n=0;n<200;++n)contact.Tick(at(210+n*.01),f,true);
  assert(contact.busy());contact.Tick(at(212.01),f,true);
  assert(contact.state()=="SUCCEEDED");assert(contact.assessment().state()=="GRASPED");
 }
 // Waiting for the next plan preserves the completed trajectory target.
 {
  FreshStream waiting(cfg);AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
  std::vector<std::array<float,7>> path(2,f.joints.position);
  assert(waiting.Start(1,"move",path,at(1),f));
  for(int n=0;n<40;++n)waiting.Tick(at(1+n*.01),f,true);
  assert(waiting.state()=="SUCCEEDED");
  const auto held=waiting.command();
  for(int n=0;n<1000;++n){const auto cmd=waiting.Tick(at(2+n*.01),f,true);assert(cmd.q==held.q&&cmd.torque==held.torque);}
  path[0][0]=.5;assert(!waiting.Start(2,"move",path,at(13),f));
 }
 // A delayed callback does not reject a healthy stream or jump over samples.
 {
  FreshStream delayed(cfg);AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
  std::vector<std::array<float,7>> path(101,f.joints.position);
  for(size_t i=0;i<path.size();++i)path[i][0]=.1*i/100;
  assert(delayed.Start(1,"move",path,at(30),f));
  const auto first=delayed.Tick(at(30),f,true);
  const auto after_gap=delayed.Tick(at(30.03),f,true);
  assert(delayed.busy()&&std::abs(after_gap.q[0]-first.q[0])<=.00101);
  delayed.Tick(at(30.04),f,false);assert(delayed.state()=="CANCELLED");
  MissionStreamConfig longer=cfg;longer.max_duration_s=180;
  FreshStream long_stream(longer);path.assign(6002,f.joints.position);
  assert(long_stream.Start(1,"move",path,at(40),f));
 }
 // Unqualified open-jaw approximation never authorizes physical RETREAT.
 assert(AutomaticRetreatPlanAllowed(true,"retreat","RETREAT","move",true));
 assert(AutomaticRetreatPlanAllowed(true,"retreat","GRASP","move",true));
 assert(!AutomaticRetreatPlanAllowed(false,"retreat","RETREAT","move",true));
 assert(!AutomaticRetreatPlanAllowed(true,"retreat","RETREAT","move",false));
 assert(!AutomaticRetreatPlanAllowed(true,"retreat","WAIT_RETREAT_APPROVAL","move",true));
 assert(!AutomaticRetreatPlanAllowed(true,"approach","RETREAT","move",true));
 assert(!AutomaticRetreatPlanAllowed(true,"retreat","RETREAT","home",true));
 assert(!PhysicalRetreatPlanAllowed(false,"retreat","GRASP",false,"move"));
 assert(!PhysicalRetreatPlanAllowed(false,"","RETREAT",false,"move"));
 assert(!PhysicalRetreatPlanAllowed(false,"","GRASP",true,"move"));
 assert(PhysicalRetreatPlanAllowed(false,"home","RETURN_HOME",true,"home"));
 assert(PhysicalRetreatPlanAllowed(false,"scan_step","ARM_SEARCH",false,"move"));
 assert(PhysicalRetreatPlanAllowed(true,"retreat","RETREAT",true,"move"));
 // Unchecked preset admission must not admit object paths or payload motion.
 {
  std::vector<std::array<float,7>> path(2);path[1][0]=.1F;
  assert(UncheckedSearchPlanAllowed("preset_search_unchecked","scan_pose","ARM_SCAN_POSE","move",false,path));
  assert(UncheckedSearchPlanAllowed("preset_search_unchecked","scan_step","ARM_SEARCH","move",false,path));
  assert(UncheckedSearchPlanAllowed("preset_search_unchecked","scan_return","RETURN_HOME","move",false,path));
  assert(!UncheckedSearchPlanAllowed("preset_search_unchecked","approach","ARM_SEARCH","move",false,path));
  assert(!UncheckedSearchPlanAllowed("preset_search_unchecked","scan_step","PREGRASP","move",false,path));
  assert(!UncheckedSearchPlanAllowed("preset_search_unchecked","scan_step","ARM_SEARCH","move",true,path));
  assert(!UncheckedSearchPlanAllowed("","scan_step","ARM_SEARCH","move",false,path));
  path[1][1]=.01F;
  assert(!UncheckedSearchPlanAllowed("preset_search_unchecked","scan_step","ARM_SEARCH","move",false,path));
 }
 // The higher search speed is confined to preset search phases.
 assert(PresetSearchPlanAllowed("scan_step","ARM_SEARCH"));
 assert(PresetSearchPlanAllowed("scan_return","RETURN_HOME"));
 assert(!PresetSearchPlanAllowed("scan_step","PREGRASP"));
 assert(!PresetSearchPlanAllowed("pregrasp","ARM_SEARCH"));
 {
  MissionStreamConfig search=cfg;search.velocity.fill(.1F);search.search_velocity=std::array<double,6>{.2F,.2F,.2F,.2F,.2F,.2F};
  AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
  std::vector<std::array<float,7>> path(101,f.joints.position);
  for(size_t i=0;i<path.size();++i)path[i][0]=.15F*i/100;
  FreshStream ordinary(search),preset(search);
  assert(!ordinary.Start(1,"move",path,at(50),f));
  assert(preset.Start(1,"move",path,at(50),f,true));
  for(size_t i=0;i<path.size();++i)path[i][0]=.25F*i/100;
  MissionStream too_fast(search);assert(!too_fast.Start(1,"move",path,at(50),f,true));
 }
 // Both sides of the measured closed-zero band agree at admission.
 {
  MissionStreamConfig c=cfg;c.start_tolerance=.025;c.assessment.negative_tolerance=.025;
  for(float q:{-.025F,0.F,.025F}){
   AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.position[6]=q;
   std::vector<std::array<float,7>> path(2,f.joints.position);
   for(auto& point:path)point[6]=.02F;
   MissionStream stream(c);assert(stream.Start(1,"move",path,at(50),f));
   assert(f.joints.position[6]==q);
  }
  for(float q:{-.025001F,.025001F}){
   AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.position[6]=q;
   std::vector<std::array<float,7>> path(2,std::array<float,7>{});
   MissionStream stream(c);assert(!stream.Start(1,"move",path,at(50),f));
  }
  for(float delta:{.024999F,.025F,.025001F}){
   AutoHomeStatus f=feedback;f.joints.position.fill(0);
   std::vector<std::array<float,7>> path(2,f.joints.position);for(auto& point:path)point[0]=delta;
   MissionStream stream(c);assert(stream.Start(1,"move",path,at(50),f)==(delta<=.025F));
  }
 }
 // YAML motor speeds are confined to explicitly agreed operation/phase pairs.
 for(const auto& pair:std::vector<std::pair<std::string,std::string>>{
     {"scan_pose","ARM_SCAN_POSE"},{"pregrasp","PREGRASP"},{"approach","GRASP"},
     {"retreat","RETREAT"},{"retreat","WAIT_RETREAT_APPROVAL"},{"retreat","GRASP"},
     {"reverse_search","RETURN_SEARCH"}}){
  assert(MotorLimitPlanAllowed(pair.first,pair.second,"move"));
  assert(!MotorLimitPlanAllowed(pair.first,"ARM_SEARCH","move"));
  assert(!MotorLimitPlanAllowed(pair.first,pair.second,"home"));
 }
 for(const auto* op:{"home","refine","scan_step","scan_return","close","unknown"})
  assert(!MotorLimitPlanAllowed(op,"GRASP","move"));
 {
  MissionStreamConfig c=cfg;c.velocity.fill(.1);c.search_velocity=std::array<double,6>{};
  c.search_velocity->fill(.2);c.motor_velocity=std::array<double,6>{};c.motor_velocity->fill(1.);
  AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
  std::vector<std::array<float,7>> path(101,f.joints.position);
  for(size_t i=0;i<path.size();++i)path[i][0]=.3F*i/100;
  MissionStream motor(c),ordinary(c),search(c),ambiguous(c);
  assert(motor.Start(1,"move",path,at(50),f,false,true));
  assert(!ordinary.Start(1,"move",path,at(50),f));
  assert(!search.Start(1,"move",path,at(50),f,true));
  assert(!ambiguous.Start(1,"move",path,at(50),f,true,true));
  for(size_t i=0;i<path.size();++i)path[i][0]=1.1F*i/100;
  MissionStream too_fast(c);assert(!too_fast.Start(1,"move",path,at(50),f,false,true));
  c.motor_velocity.reset();MissionStream missing(c);
  assert(!missing.Start(1,"move",path,at(50),f,false,true));
  c.motor_velocity=std::array<double,6>{};bool invalid=false;
  try{MissionStream bad(c);}catch(const std::invalid_argument&){invalid=true;}
  assert(invalid);
 }
 // Completion tolerance and timeout diagnostics must use pre-cancel targets.
 {
  MissionStreamConfig c=cfg;c.reached_tolerance=.025;c.settle_s=.2;c.completion_timeout_s=.5;
  AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
  std::vector<std::array<float,7>> path(2,f.joints.position);
  FreshStream ok(c);assert(ok.Start(1,"move",path,at(100),f));
  f.joints.position[0]=.024F;
  ok.Tick(at(100),f,true);ok.Tick(at(100.02),f,true);ok.Tick(at(100.23),f,true);
  assert(ok.state()=="SUCCEEDED");assert(!ok.timeout_snapshot());
  for(bool velocity_only:{false,true}){
   f.joints.position.fill(0);f.joints.velocity.fill(0);
   FreshStream failed(c);assert(failed.Start(1,"move",path,at(200),f));
   if(velocity_only)f.joints.velocity[6]=.1F;else f.joints.position[6]=.026F;
   failed.Tick(at(200),f,true);failed.Tick(at(200.02),f,true);failed.Tick(at(200.6),f,true);
   assert(failed.state()=="FAILED");assert(failed.error()=="trajectory_feedback_timeout");
   const auto& d=*failed.timeout_snapshot();
   assert(d.checked_joints==7&&d.sample_index==1&&d.sample_count==2);
   assert(d.target[6]==0&&d.final_target[6]==0);
   assert(d.measured[6]==f.joints.position[6]&&d.velocity[6]==f.joints.velocity[6]);
   assert(failed.command().q[6]==f.joints.position[6]);
   failed.Cancel(f,"later_cancel");assert(failed.timeout_snapshot()->target[6]==0);
   std::vector<std::array<float,7>> next(2,f.joints.position);
   assert(failed.Start(2,"move",next,at(201),f));assert(!failed.timeout_snapshot());
  }
 }
 std::cout<<"PASS bounded close stub, unknown object, retained command, empty/lost/open HOME6, legacy HOME7, cancel and limits\n";
}
