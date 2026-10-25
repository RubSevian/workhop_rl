#include "rars_mission_stream.hpp"
#include <cassert>
#include <iostream>
using namespace sim2real;
ArmTime t(double x){return ArmTime{}+std::chrono::nanoseconds(std::llround(x*1e9));}
AutoHomeStatus sample(double seconds,float q,float tau,uint64_t frame){
 AutoHomeStatus s;s.state=AutoHomeState::HOLD_HOME;s.feedback_ready=s.motors_enabled=s.target_fresh=true;s.feedback_stamp=t(seconds);
 s.joints.position.fill(0);s.joints.velocity.fill(0);s.joints.torque.fill(0);s.joints.position[6]=q;s.joints.torque[6]=tau;s.joints.valid.fill(true);s.joints.error.fill(1);
 for(int i=0;i<7;++i)s.joints.motor_id[i]=i+1;
 s.communication.connected=s.communication.enabled=s.communication.feedback_received=true;s.communication.valid_frames=frame;return s;
}
MissionStreamConfig config(){MissionStreamConfig c;c.lower.fill(-3);c.upper.fill(3);c.velocity.fill(1);c.assessment.enabled=true;return c;}
int main(){
 const auto c=config();
 {
  auto cfg=c;cfg.assessment.negative_tolerance=.025;
  MeasuredGripperControl m(cfg.gripper_control,cfg.assessment,0,1,100,.1);
  auto s=sample(1,.02F,0,1);m.Begin(t(1),s);assert(m.Update(t(1),s,true,false));
  assert(m.target()==double(s.joints.position[6])); // geometry zero must not rewrite MIT target
  assert(s.joints.position[6]==.02F);
 }

 // Close -> HOME -> terminal success retains the live regulator. Terminal
 // phase alone cannot enable it; cancellation/fault still revokes the hold.
 {
  auto cfg=c;MissionStream m(cfg);auto s=sample(10,.7F,2.2F,1);
  std::vector<std::array<float,7>> close(2,s.joints.position);
  assert(!m.CompletedHomeHoldAllowed("GRASP_PRESUMED_SUCCESS"));
  assert(m.Start(1,"close",close,t(10),s));
  for(int n=0;n<=60;++n)m.Tick(t(10+n*.01),sample(10+n*.01,.7F,2.2F,n+2),true);
  assert(m.holding()&&!m.busy());
  assert(!m.CompletedHomeHoldAllowed("GRASP_PRESUMED_SUCCESS"));
  s=sample(11,.7F,2.2F,100);
  auto point=s.joints.position;point[6]=m.command().q[6];
  std::vector<std::array<float,7>> home(2,point);
  assert(m.Start(2,"home",home,t(11),s));
  assert(!m.CompletedHomeHoldAllowed("GRASP_PRESUMED_SUCCESS"));
  for(int n=0;n<=30;++n)m.Tick(t(11+n*.01),sample(11+n*.01,.7F,2.2F,n+101),true);
  assert(m.CompletedHomeHoldAllowed("GRASP_PRESUMED_SUCCESS"));
  for(const auto* phase:{"BLOCKED","CANCELLED","IDLE"})assert(!m.CompletedHomeHoldAllowed(phase));
  assert(m.CompletedHomeHoldAllowed("TARGET_NOT_GRASPED"));
  for(int n=31;n<=130;++n){
   const auto cmd=m.Tick(t(11+n*.01),sample(11+n*.01,.7F,2.2F,n+101),m.CompletedHomeHoldAllowed("TARGET_NOT_GRASPED"));
   assert(m.holding()&&m.CompletedHomeHoldAllowed("GRASP_PRESUMED_SUCCESS"));
   assert(cmd.kp[6]==cfg.gripper_control.kp&&cmd.torque[6]==0);
  }
  // All live authority checks remain necessary even after terminal success.
  const auto cmd=m.Tick(t(12.31),sample(12.31,.7F,2.2F,232),false);
  assert(!m.holding()&&cmd.kp[6]==0);
  assert(!m.CompletedHomeHoldAllowed("GRASP_PRESUMED_SUCCESS"));
 }
 // Polarity of measured torque is deliberately irrelevant. Both closing joint
 // directions are supported without using torque sign as an extra criterion.
 for(double polarity:{-1.,1.})for(bool reverse:{false,true}){
  auto cfg=c;cfg.closed=reverse?1.:0.;cfg.open=reverse?0.:1.;
  MeasuredGripperControl m(cfg.gripper_control,cfg.assessment,cfg.closed,cfg.open,100,.1);
  auto s=sample(1,.5,0,1);m.Begin(t(1),s);assert(m.Update(t(1),s,true,false));
  const double initial=m.target();
  for(int n=1;n<=10;++n){s=sample(1+n*.01,.5,0,n+1);double prev=m.target();assert(m.Update(t(1+n*.01),s,true,false));assert(std::abs(m.target()-prev)<=.00150001);}
  assert(reverse?m.target()>initial:m.target()<initial);
  const double before=m.target();s=sample(1.11,.5,polarity*2.2,12);assert(m.Update(t(1.11),s,true,false));
  assert(m.contact()&&std::abs(before-m.target())<1e-10); // >= includes equality.
  s=sample(1.12,.5,polarity*2.8,13);assert(m.Update(t(1.12),s,true,true));
  assert(reverse?m.target()<before:m.target()>before); // unwind overload
  double last=m.target();s=sample(1.13,.5,polarity*2.15,14);m.Update(t(1.13),s,true,true);assert(m.target()==last);
  s=sample(1.14,.5,polarity*1.8,15);m.Update(t(1.14),s,true,true);assert(reverse?m.target()>last:m.target()<last);
  last=m.target();for(int n=0;n<5;++n){s.feedback_stamp=t(1.15+n*.01);m.Update(t(1.15+n*.01),s,true,true);assert(m.target()==last);}
  s.feedback_stamp=t(1.3);assert(!m.Update(t(1.3),s,true,true));assert(m.failed());
  s=sample(1.31,.5,0,16);assert(!m.Update(t(1.31),s,true,true));assert(m.target()==last); // no auto-restart
 }
 // A healthy snapshot may predate close admission slightly; it must not
 // trip a stale-feedback failure while awaiting the next SDK frame.
 {MeasuredGripperControl g(c.gripper_control,c.assessment,0,1,100,.1);
  auto s=sample(5,.7,0,1);g.Begin(t(5.005),s);assert(g.Update(t(5.005),s,true,false));
  assert(g.target()==double(s.joints.position[6]));}
 // Frozen measurements cannot accumulate arbitrary position error or cause
 // unbounded rotation. Neither delayed callbacks nor deadbands create bursts.
 {MeasuredGripperControl g(c.gripper_control,c.assessment,0,1,100,.1);
  auto s=sample(6,.7,0,1);g.Begin(t(6),s);g.Update(t(6),s,true,false);
  double prev=g.target();s=sample(6.09,.7,0,2);g.Update(t(6.09),s,true,false);assert(prev-g.target()<=.00150001);
  for(int n=1;n<1000;++n){s=sample(6.09+n*.01,.7,0,n+2);g.Update(t(6.09+n*.01),s,true,false);}
  assert(.7-g.target()<=c.gripper_control.max_target_lead_rad+1e-6);}
 // Stream uses zero FF and the agreed MIT gains, including between plans and
 // during arm motion. Feedback loss reports UNKNOWN and removes closing Kp.
 MissionStream m(c);auto s=sample(10,.7,0,1);std::vector<std::array<float,7>> p(2,s.joints.position);
 assert(m.Start(1,"close",p,t(10),s));
 for(int n=0;n<=120;++n){s=sample(10+n*.01,.7,n<20?0:-2.2,n+1);auto cmd=m.Tick(t(10+n*.01),s,true);assert(cmd.kp[6]==20&&cmd.kd[6]==1&&cmd.torque[6]==0);}
 assert(m.state()=="SUCCEEDED"&&m.assessment().state()=="GRASPED");
 s=sample(11.21,.7,-2.8,122);auto target=m.command().q[6];m.Tick(t(11.21),s,true);assert(m.command().q[6]>target);
 p.assign(101,s.joints.position);for(auto& q:p)q[6]=m.command().q[6];for(int i=0;i<=100;++i)p[i][0]=i*.001;
 assert(m.Start(2,"move",p,t(11.21),s));m.Tick(t(11.22),sample(11.22,.7,-1.8,123),true);
 s=sample(11.23,.7,-1.8,124);s.feedback_ready=false;
 auto cmd=m.Tick(t(11.23),s,true);assert(m.state()=="FAILED"&&m.assessment().state()=="UNKNOWN"&&cmd.kp[6]==0&&cmd.torque[6]==0);
 cmd=m.Tick(t(11.24),sample(11.24,.7,0,125),true);assert(cmd.kp[6]==0&&m.assessment().state()=="UNKNOWN");
 // At the measured closed stop, do not keep an obsolete open target that
 // would reopen an empty gripper. This is not evidence of grasp success.
 {MissionStream empty(c);auto a=sample(15,.7,0,1);std::vector<std::array<float,7>> p(2,a.joints.position);
  assert(empty.Start(1,"close",p,t(15),a));empty.Tick(t(15),a,true);
  auto cmd=empty.Tick(t(15.01),sample(15.01,0,0,2),true);
  assert(cmd.q[6]==0&&cmd.torque[6]==0&&empty.state()=="SUCCEEDED");
  empty.ObserveAssessment(t(15.01),sample(15.01,0,0,2));assert(empty.assessment().state()=="NOT_GRASPED");}
 // Invalid motor/error/torque/epoch/watchdog/authority must never advance qdes.
 for(int failure=0;failure<7;++failure){
  MeasuredGripperControl g(c.gripper_control,c.assessment,0,1,100,.1);auto a=sample(20,.7,0,1);g.Begin(t(20),a);g.Update(t(20),a,true,false);
  auto b=sample(20.01,.7,0,2);if(failure==0)b.joints.torque[6]=NAN;if(failure==1)b.joints.valid[6]=false;
  if(failure==2)b.joints.error[6]=2;if(failure==3)b.communication.watchdog_tripped=true;
  if(failure==4)b.feedback_stamp=t(19.99);if(failure==5)b.state=AutoHomeState::FAULT_LATCHED;
  double prev=g.target();assert(!g.Update(t(20.01),b,failure!=6,false));assert(g.target()==prev);
 }
 // Synthetic rigid-contact plant with delayed measured torque. This is an
 // overshoot demonstration, NOT a model of certified physical motor dynamics.
 for(int delay:{0,2,5,10}){
  MeasuredGripperControl g(c.gripper_control,c.assessment,0,1,100,.1);
  std::vector<double> torque(1,0);double peak=0;auto a=sample(30,.7,0,1);g.Begin(t(30),a);
  for(int n=0;n<600;++n){double tau=torque[std::max(0,int(torque.size())-1-delay)];
   a=sample(30+(n-delay)*.01,.7,-tau,n+1);assert(g.Update(t(30+n*.01),a,true,g.contact()));
   double actual=20*(.7-g.target());torque.push_back(actual);peak=std::max(peak,actual);
  }
  assert(peak>=2.2&&peak<=2.2+20*.15*(delay+1)*.01+1e-5);
  std::cout<<"synthetic_delay_ms="<<delay*10<<" peak_nm="<<peak<<" final_nm="<<torque.back()<<"\n";
 }
 std::cout<<"PASS measured-contact controller (synthetic only)\n";
}
