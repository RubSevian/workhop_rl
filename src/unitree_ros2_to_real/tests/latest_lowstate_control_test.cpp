#include "latest_lowstate_control.hpp"
#include <cassert>
#include <limits>
#include <cstring>
using namespace sim2real;
SafetyTime at(double s){return SafetyTime(std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s)));}
unitree_go::msg::LowState frame(float q=1,uint16_t mask=0) {
 unitree_go::msg::LowState msg;msg.imu_state.quaternion[0]=1;
 for(auto& motor:msg.motor_state){motor.q=q;motor.dq=q*2;}
 msg.wireless_remote[2]=mask&255;msg.wireless_remote[3]=mask>>8;
 float stick=.3F;std::memcpy(msg.wireless_remote.data()+4,&stick,4);return msg;
}
R3Profile profile(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);
 p.stand.fill(.2F);p.gate0_verified=p.mapping_verified=p.emergency_validated=p.remote_chords_verified=p.robot_supported=p.policy_timing_reviewed=true;
 std::array<float,12> damping;damping.fill(1.5F);p.emergency_kd=damping;p.emergency_evidence="OFFLINE_MOCK_ONLY";return p;}
R3Inputs ready(){R3Inputs in;in.ready={true,true,true,true,true,true,true,true,false,true};
 in.measured_q.fill(.4F);in.arm_static_hold=in.arm_home_ready=in.arm_control_ready=in.own_output_healthy=true;
 in.sport=SportMode::RELEASED;in.sport_stamp=in.lowstate_stamp=in.remote_stamp=in.arm_stamp=in.target_stamp=at(1);return in;}
struct Fixture {
 LatestLowStateChannel channel{.5,{"L1","L2","X"},{"L1","L2","B"}};
 LatestLowStateControl control{.25};LowStateReader reader{.5};R3Supervisor supervisor{profile()};R3Inputs inputs{ready()};
 void apply(double now){control.Apply(channel,reader,inputs,supervisor,at(now));supervisor.Observe(inputs,at(now));}
 void arm(){channel.Receive(frame(),at(1));apply(1);assert(supervisor.Takeover(at(1)).success);apply(1);assert(supervisor.EnableOutput(true,at(1)).success);}
};
int main() {
 // The first bad frame survives a newer good one while output is enabled.
 {Fixture f;f.arm();
  f.channel.Receive(frame(std::numeric_limits<float>::quiet_NaN()),at(1.01));
  f.channel.Receive(frame(2),at(1.02));f.apply(1.03);
  assert(f.supervisor.fault_latched()&&f.supervisor.last_fault()=="invalid_lowstate");
  assert(f.inputs.measured_q[11]==2&&f.inputs.lowstate_stamp==at(1.02));
  assert(f.control.rejection()->first_time==at(1.01));
 }
 // A recovered pre-arm error must not replace the later fault's reason.
 {Fixture f;
  auto zero=frame();zero.imu_state.quaternion.fill(0);
  f.channel.Receive(zero,at(.5));f.apply(.5);f.arm();
  f.channel.Receive(frame(std::numeric_limits<float>::quiet_NaN()),at(1.01));f.apply(1.01);
  assert(f.control.rejection()->reason==LowStateRejection::NONFINITE_MOTOR);
  assert(f.control.rejection()->first_time==at(1.01));
  f.channel.Receive(zero,at(1.02));f.apply(1.02);
  assert(f.control.rejection()->first_time==at(1.01));
 }
 // Disarmed invalid data blocks admission without a permanent fault.
 {Fixture f;f.channel.Receive(frame(),at(1));f.apply(1);assert(f.supervisor.Takeover(at(1)).success);f.apply(1);
  f.channel.Receive(frame(std::numeric_limits<float>::quiet_NaN()),at(1.01));f.apply(1.01);
  assert(!f.inputs.ready.motor_state_valid&&!f.supervisor.fault_latched());
  assert(!f.supervisor.EnableOutput(true,at(1.01)).success);
  f.channel.Receive(frame(2),at(1.02));f.apply(1.02);
  assert(f.inputs.ready.motor_state_valid&&!f.supervisor.fault_latched());
  assert(f.supervisor.EnableOutput(true,at(1.02)).success);
 }
 // Future snapshots are deferred; the error barrier still blocks older good data.
 {Fixture f;f.channel.Receive(frame(),at(1));f.apply(1);
  f.channel.Receive(frame(std::numeric_limits<float>::quiet_NaN()),at(1.1));f.apply(1.05);
  assert(f.inputs.lowstate_stamp==at(1)&&!f.inputs.ready.motor_state_valid&&!f.supervisor.fault_latched());
  f.channel.Receive(frame(2),at(1.2));f.apply(1.15);
  assert(!f.inputs.ready.motor_state_valid);f.apply(1.2);assert(f.inputs.ready.motor_state_valid);
  assert(f.reader.snapshot().received==at(1.2));
  assert(f.control.status().lx==.3F);f.apply(1.5);assert(!f.control.status().remote_valid);
 }
 // A short invalid excursion before arm recovers with the newer good frame.
 {Fixture f;f.channel.Receive(frame(std::numeric_limits<float>::quiet_NaN()),at(1));f.channel.Receive(frame(),at(1.01));f.apply(1.01);
  assert(!f.supervisor.fault_latched()&&f.inputs.ready.motor_state_valid);
 }
 // More than the former queue capacity is bounded work and keeps the last stamp.
 {Fixture f;for(int i=0;i<512;++i)f.channel.Receive(frame(2),at(1)+std::chrono::milliseconds(2*i));f.apply(2.023);
  assert(f.reader.snapshot().received==at(1)+std::chrono::milliseconds(1022)&&f.inputs.measured_q[0]==2&&!f.supervisor.fault_latched());
 }
 // Once a stop is processed, later neutral snapshots never synthesize takeover.
 {Fixture f;for(int i=0;i<5;++i)f.channel.Receive(frame(1,0x722),at(1+i*.2));
  f.channel.Receive(frame(),at(1.81));f.apply(1.82);
  assert(f.control.TakeDecision(f.channel,at(1.82))==R3RemoteEvent::EMERGENCY);
  assert(f.control.TakeDecision(f.channel,at(1.82))==R3RemoteEvent::NONE);
 }
}
