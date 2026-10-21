#include "rars_mission_stream.hpp"
#include "arm_status_snapshot.hpp"
#include <cassert>
#include <iostream>
using namespace sim2real;
ArmTime at(double s){return ArmTime(std::chrono::duration_cast<ArmClock::duration>(std::chrono::duration<double>(s)));}
int main(){
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

 MissionStreamConfig cfg;cfg.close_duration_s=.5;cfg.lower.fill(-3);cfg.upper.fill(3);cfg.velocity.fill(1);cfg.kp.fill(20);cfg.kd.fill(1);
 MissionStream stream(cfg);AutoHomeStatus feedback;feedback.feedback_ready=feedback.motors_enabled=feedback.target_fresh=true;
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
 assert(stream.command().torque[6]<0&&stream.command().kp[6]==cfg.hold_kp);
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
 points[1]=points[0];points[1][6]=.8;
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
 y["q"]=std::vector<float>{0,0,0,0,0,0,.95f};
 assert(ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["q"]=std::vector<float>{0,0,0,0,0,0,0};assert(ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["q"]=std::vector<float>{0,0,0,0,0,0,NAN};assert(!ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 y["q"]=std::vector<float>{.3,0,0,0,0,0,0};assert(!ParseArmStatus(YAML::Dump(y),.15,now).error.empty());
 // Empty/closed gripper finishes normally; a stalled open gripper is also
 // unverified after the timer. Neither case claims object presence.
 for(float final_grip:{0.f,.8f}){
  MissionStream stub(cfg);AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.position[6]=.8;
  std::vector<std::array<float,7>> close_points(2,f.joints.position);
  assert(stub.Start(1,"close",close_points,at(20),f));
  for(int n=0;n<70;++n){f.joints.position[6]=final_grip;f.joints.velocity.fill(0);stub.Tick(at(20+n*.01),f,true);}
  assert(stub.state()=="SUCCEEDED"&&stub.holding()&&!stub.HoldConfirmed(f));
  if(final_grip==0)assert(stub.command().torque[6]==0);
 }
 // Waiting for the next plan preserves the completed trajectory target.
 {
  MissionStream waiting(cfg);AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
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
  MissionStream delayed(cfg);AutoHomeStatus f=feedback;f.joints.position.fill(0);f.joints.velocity.fill(0);
  std::vector<std::array<float,7>> path(101,f.joints.position);
  for(size_t i=0;i<path.size();++i)path[i][0]=.1*i/100;
  assert(delayed.Start(1,"move",path,at(30),f));
  const auto first=delayed.Tick(at(30),f,true);
  const auto after_gap=delayed.Tick(at(30.03),f,true);
  assert(delayed.busy()&&std::abs(after_gap.q[0]-first.q[0])<=.00101);
  delayed.Tick(at(30.04),f,false);assert(delayed.state()=="CANCELLED");
  MissionStreamConfig longer=cfg;longer.max_duration_s=180;
  MissionStream long_stream(longer);path.assign(6002,f.joints.position);
  assert(long_stream.Start(1,"move",path,at(40),f));
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
  MissionStream ordinary(search),preset(search);
  assert(!ordinary.Start(1,"move",path,at(50),f));
  assert(preset.Start(1,"move",path,at(50),f,true));
  for(size_t i=0;i<path.size();++i)path[i][0]=.25F*i/100;
  MissionStream too_fast(search);assert(!too_fast.Start(1,"move",path,at(50),f,true));
 }
 std::cout<<"PASS bounded close stub, unknown object, retained command, empty/lost/open HOME6, legacy HOME7, cancel and limits\n";
}
