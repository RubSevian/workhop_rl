// Pure synthetic source frames. No ROS/serial/device operations.
#include "grasp_assessment.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>
using namespace sim2real;
ArmTime t(double s){return ArmTime{}+std::chrono::nanoseconds(std::llround(s*1e9));}
GraspAssessmentConfig cfg(){GraspAssessmentConfig c;c.enabled=true;return c;}
AutoHomeStatus sample(double seconds,float torque=1.6F,float q=.7F,uint64_t frame=1){
 AutoHomeStatus s;s.state=AutoHomeState::HOLD_HOME;s.feedback_ready=s.motors_enabled=true;s.feedback_stamp=t(seconds);
 s.communication.connected=s.communication.enabled=s.communication.feedback_received=true;s.communication.valid_frames=frame;
 s.joints.valid[6]=true;s.joints.motor_id[6]=7;s.joints.error[6]=1;s.joints.position[6]=q;s.joints.torque[6]=torque;return s;
}
void feed(GraspAssessment& a,double start,int count,float torque=1.6F,float q=.7F){for(int i=0;i<=count;++i)a.Observe(t(start+i*.01),sample(start+i*.01,torque,q,1+uint64_t(std::llround((start+i*.01)*1000))),true,false,.1);}
int main(int argc,char** argv){
 if(argc>1&&std::string(argv[1])=="--closed-zero-grid"){
  auto c=cfg();c.negative_tolerance=.025;std::cout<<std::setprecision(17);
  for(double q:{-.025,double(-.025F),-.024,0.,.024,.025,double(.025F),.025001,.05,.7})std::cout<<q<<' '<<GripperInnerWidth(q,c)<<'\n';
  return 0;
 }
 if(argc>1){auto c=cfg();std::cout<<std::setprecision(17);for(int n=0;n<=200;++n){double q=n/200.;std::cout<<q<<' '<<GripperInnerWidth(q,c)<<'\n';}return 0;}
 {
  auto c=cfg();c.negative_tolerance=.025;
  for(double q:{-.025,0.,.025,double(-.025F),double(.025F)})assert(std::abs(GripperInnerWidth(q,c))<1e-12);
  assert(GripperInnerWidth(.025001,c)>0.);
  bool rejected=false;try{GripperInnerWidth(-.025001,c);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
 }
 assert(GripperInnerWidth(.7,cfg())>=.05);
 GraspAssessment a(cfg());feed(a,10,49);assert(a.state()=="UNKNOWN");feed(a,10.5,0);assert(a.state()=="GRASPED");
 // Two seconds must be real continuous feedback; closed-zero is never an object.
 {
  auto c=cfg();c.confirmation_duration_s=2.;c.negative_tolerance=.025;
  GraspAssessment a(c);feed(a,200,199);assert(a.state()=="UNKNOWN");
  feed(a,202,0);assert(a.state()=="GRASPED");
  for(float q:{-.025F,0.F,.025F}){
   GraspAssessment empty(c);feed(empty,210,210,2.2F,q);
   empty.Observe(t(212.11),sample(212.11,2.2F,q,300000),true,true,.1);
   assert(empty.state()=="NOT_GRASPED");
  }
 }
 // Strict absolute measured-torque boundary; commanded tau never substitutes.
 for(float torque:{1.5F,1.49F,-1.5F}){GraspAssessment x(cfg());feed(x,20,60,torque);x.Observe(t(20.61),sample(20.61,torque,.7,30000),true,true,.1);assert(x.state()=="NOT_GRASPED");}
 // Inclusive width boundary uses the exact computed width, with no new tolerance.
 auto equal=cfg();equal.min_grasp_width_m=GripperInnerWidth(.7F,equal);GraspAssessment e(equal);feed(e,30,50);assert(e.state()=="GRASPED");
 GraspAssessment negative(cfg());feed(negative,35,50,-1.6F);assert(negative.state()=="GRASPED");
 GraspAssessment narrow(cfg());feed(narrow,40,60,1.6F,.01F);narrow.Observe(t(40.61),sample(40.61,1.6F,.01F,50000),true,true,.1);assert(narrow.state()=="NOT_GRASPED");
 // Violation interrupts the continuous interval.
 GraspAssessment reset(cfg());feed(reset,50,40);reset.Observe(t(50.41),sample(50.41,1.5F,.7,50411),true,false,.1);feed(reset,50.42,49);assert(reset.state()=="UNKNOWN");feed(reset,50.92,0);assert(reset.state()=="GRASPED");
 // Missing/repeated frames cannot accumulate 500ms of source measurements.
 GraspAssessment repeated(cfg());auto old=sample(60);for(int n=0;n<60;++n)repeated.Observe(t(60+n*.01),old,true,false,.1);assert(repeated.state()=="UNKNOWN");
 for(int k=0;k<7;++k){GraspAssessment x(cfg());feed(x,70,50);auto s=sample(70.51,1.6F,.7,90000);if(k==0)s.feedback_ready=false;if(k==1)s.joints.valid[6]=false;if(k==2)s.joints.motor_id[6]=6;if(k==3)s.joints.error[6]=8;if(k==4)s.joints.torque[6]=NAN;if(k==5)s.joints.position[6]=NAN;if(k==6)s.communication.stm32_watchdog_tripped=true;x.Observe(t(70.51),s,true,true,.1);assert(x.state()=="UNKNOWN");}
 GraspAssessment cached_jitter(cfg());feed(cached_jitter,75,40);
 auto duplicate=sample(75.3995,1.6F,.7F,75401);cached_jitter.Observe(t(75.41),duplicate,true,false,.1);
 feed(cached_jitter,75.41,9);assert(cached_jitter.state()=="GRASPED");
 auto renewed=sample(75.7,1.6F,.7F,75501);cached_jitter.Observe(t(75.7),renewed,true,true,.1);assert(cached_jitter.state()=="UNKNOWN");
 GraspAssessment gap(cfg());feed(gap,80,40);feed(gap,80.6,49);assert(gap.state()=="UNKNOWN");feed(gap,81.1,0);assert(gap.state()=="GRASPED");gap.Reset();assert(gap.state()=="UNKNOWN");
 GraspAssessment preclose(cfg());preclose.Begin(t(100));preclose.Observe(t(100),sample(99.95,1.7F,.7,100000),true,false,.1);assert(preclose.state()=="UNKNOWN");feed(preclose,100,49);assert(preclose.state()=="UNKNOWN");feed(preclose,100.5,0);assert(preclose.state()=="GRASPED");
 GraspAssessment no_speed_gate(cfg());for(int n=0;n<=50;++n){auto s=sample(110+n*.01,1.7F,.7,110000+n);s.joints.velocity[6]=2.;no_speed_gate.Observe(t(110+n*.01),s,true,false,.1);}assert(no_speed_gate.state()=="GRASPED");
 GraspAssessment idle(cfg());feed(idle,90,50);idle.Observe(t(90.51),sample(90.51),false,true,.1);assert(idle.state()=="UNKNOWN");
 // Full MIT is not ff alone; feedback compensation is not a hardware cap.
 assert(std::abs(TotalMitTorque(20,.2,0,1,0,-.1,2.2)-6.3)<1e-12);
 assert(std::abs(TotalMitTorque(0,0,0,.5,0,-1,2.2)-2.7)<1e-12);
 assert(!ControllerTotalTorqueCapSupported());
 std::cout<<"PASS measured torque/width, strict/inclusive boundaries, source-time 500ms, reset/gaps/replay/stale/fault, full MIT terms; synthetic only\n";
}
