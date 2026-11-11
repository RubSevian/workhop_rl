#include "latest_lowstate_channel.hpp"
#include <cassert>
#include <limits>
using namespace sim2real;
SafetyTime at(double s){return SafetyTime(std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s)));}
unitree_go::msg::LowState frame(uint16_t buttons=0,float q=1) {
 unitree_go::msg::LowState m;m.imu_state.quaternion[0]=1;
 for(auto& motor:m.motor_state){motor.q=q;motor.dq=2*q;}
 m.wireless_remote[2]=buttons&255;m.wireless_remote[3]=buttons>>8;return m;
}
LatestLowStateChannel channel(){return LatestLowStateChannel(.5,{"L1","L2","X"},{"L1","L2","B"});}
void hold(LatestLowStateChannel& c,uint16_t buttons,double start=1) {
 for(int i=0;i<5;++i)c.Receive(frame(buttons),at(start+i*.2));
}
int main() {
 LatestLowStateChannel::Sample sample;
 for(uint16_t buttons:{0x422,0x222,0x722}) {
  auto c=channel();hold(c,buttons);c.Receive(frame(),at(1.81));
  assert(c.Read(sample));
  assert(c.TakeDecision(sample,at(3))==(buttons==0x422?R3RemoteEvent::CONTROLLED_ABORT:R3RemoteEvent::EMERGENCY));
  assert(c.TakeDecision(sample,at(3))==R3RemoteEvent::NONE);
 }
 for(int scenario=0;scenario<4;++scenario) {
  auto c=channel();hold(c,0x122);
  if(scenario==1)c.Receive(frame(),at(1.81));
  // Fresh handset frames do not extend the already-issued request's TTL.
  if(scenario==2)for(double t:{2.,2.2})c.Receive(frame(0x122),at(t));
  assert(c.Read(sample));
  if(scenario==3)assert(c.TakeDecision(sample,at(1.79))==R3RemoteEvent::NONE);
  assert(c.TakeDecision(sample,at(scenario==2?2.21:1.82))==((scenario==0||scenario==3)?R3RemoteEvent::TAKEOVER:R3RemoteEvent::NONE));
  assert(c.TakeDecision(sample,at(2.22))==R3RemoteEvent::NONE);
 }
 auto c=channel();LatestLowStateChannel::Rejection error;
 assert(!c.TakeRejection(error));
 c.Receive(frame(0,std::numeric_limits<float>::quiet_NaN()),at(1));
 auto bad=frame();bad.imu_state.quaternion.fill(0);c.Receive(bad,at(1.1));
 c.Receive(frame(),at(1.2));assert(c.Read(sample)&&sample.state.valid);
 assert(c.TakeRejection(error));assert(error.reason==LowStateRejection::NONFINITE_MOTOR&&error.first_time==at(1));
 assert(!c.TakeRejection(error));
 c.Receive(bad,at(1.3));assert(c.TakeRejection(error)&&error.reason==LowStateRejection::ZERO_QUATERNION);
 // A stop invalidates an older unconsumed request even if its state snapshot
 // is retained by a paused consumer across the stop acknowledgement.
 auto race=channel();hold(race,0x122);assert(race.Read(sample));
 LatestLowStateChannel::Sample old=sample,empty;
 hold(race,0x222,2);
 assert(race.TakeDecision(empty,at(3))==R3RemoteEvent::EMERGENCY);
 assert(race.TakeDecision(old,at(1.9))==R3RemoteEvent::NONE);
 // No overflow regardless of how long the consumer is paused.
 auto many=channel();for(int i=1;i<=10000;++i)many.Receive(frame(0,float(i)),at(i*.002));
 assert(many.Read(sample)&&sample.state.motor_q[11]==10000&&sample.state.received==at(20));
}
