#include "rars_mission_stream.hpp"
#include <cassert>
#include <iostream>
using namespace sim2real;
ArmTime t(double x){return ArmTime{}+std::chrono::nanoseconds(std::llround(x*1e9));}
AutoHomeStatus status(double seconds,float q,float tau){AutoHomeStatus s;s.state=AutoHomeState::HOLD_HOME;s.feedback_ready=s.motors_enabled=s.target_fresh=true;s.feedback_stamp=t(seconds);s.joints.position.fill(0);s.joints.velocity.fill(0);s.joints.position[6]=q;s.joints.torque[6]=tau;s.joints.valid.fill(true);s.joints.error.fill(1);for(int i=0;i<7;++i)s.joints.motor_id[i]=i+1;s.communication.connected=s.communication.enabled=s.communication.feedback_received=true;s.communication.valid_frames=uint64_t(std::llround(seconds*1000))+1;return s;}
MissionStreamConfig config(){MissionStreamConfig c;c.lower.fill(-3);c.upper.fill(3);c.velocity.fill(1);c.assessment.enabled=true;c.close_duration_s=2.;return c;}
void close(MissionStream& m,double now,float tau){auto s=status(now,.7F,tau);std::vector<std::array<float,7>> p(2,s.joints.position);assert(m.Start(1,"close",p,t(now),s));for(int i=0;i<=201&&m.busy();++i){double v=now+i*.01;s=status(v,.7F,tau);m.Tick(t(v),s,true);m.ObserveAssessment(t(v),s);}assert(!m.busy()&&m.state()=="SUCCEEDED");}
int main(){
 MissionStream success(config());close(success,10,-1.7F);assert(success.assessment().state()=="GRASPED"&&success.HoldConfirmed(status(10.5,.7,-1.7)));
 auto s=status(10.51,.7,-1.7);std::vector<std::array<float,7>> open(101,s.joints.position);for(int n=0;n<=100;++n)open[n][6]=.7F+.3F*n/100;
 assert(!success.Start(2,"release",open,t(10.51),s)); // A confirmed grasp never auto-opens.
 MissionStream failed(config());close(failed,20,1.5F);assert(failed.assessment().state()=="NOT_GRASPED");
 s=status(22.01,.7,1.5);assert(failed.RetreatReady(t(22.01),s));assert(!failed.HoldConfirmed(s));
 auto stale=s;assert(!failed.RetreatReady(t(23),stale));
 s=status(23.01,.7,1.5);assert(failed.RetreatReady(t(23.01),s));
 s.target_fresh=false;assert(!failed.RetreatReady(t(23.01),s));
 // Independent stream preserves the release scenario's original time sequence.
 MissionStream ready(config());close(ready,30,1.8F);
 assert(ready.RetreatReady(t(32.02),status(32.02,.7,1.8F)));
 ready.Cancel(status(32.03,.7,1.8F),"test");assert(!ready.RetreatReady(t(32.03),status(32.03,.7,1.8F)));
 s=status(23.02,.7,1.5);assert(failed.Start(2,"release",open,t(23.02),s));assert(!failed.holding());
 for(int i=0;i<=130;++i){double v=23.02+i*.01;s=status(v,open[std::min(i,100)][6],0);failed.Tick(t(v),s,true);failed.ObserveAssessment(t(v),s);}
 assert(failed.state()=="SUCCEEDED");std::vector<std::array<float,7>> p(2,s.joints.position);
 assert(failed.Start(3,"close",p,t(24.33),s));assert(failed.assessment().state()=="UNKNOWN");
 for(int i=0;i<=51;++i){double v=24.33+i*.01;s=status(v,1.,1.8);failed.Tick(t(v),s,true);failed.ObserveAssessment(t(v),s);}
 assert(failed.assessment().state()=="GRASPED");
 s.state=AutoHomeState::FAULT_LATCHED;failed.ObserveAssessment(t(24.85),s);assert(failed.assessment().state()=="UNKNOWN");
 std::cout<<"PASS stream measured assessment, negative torque, NOT_GRASPED-only release, second close reset, fault invalidation; synthetic only\n";
}
