#include "lowstate_inbox.hpp"
#include <cassert>
#include <limits>
#include <thread>
#include <mutex>
#include <future>
using namespace sim2real;
SafetyTime at(double s){return SafetyTime(std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s)));}
unitree_go::msg::LowState frame(float q=1.,uint16_t mask=0){
 unitree_go::msg::LowState msg;msg.imu_state.quaternion[0]=1.;
 for(auto& m:msg.motor_state){m.q=q;m.dq=q*2;}
 msg.wireless_remote[0]=0xFE;msg.wireless_remote[1]=0xEF;
 msg.wireless_remote[2]=mask&255;msg.wireless_remote[3]=mask>>8;
 return msg;
}
R3Profile profile(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);
 p.stand.fill(.2F);std::array<float,12> lie;lie.fill(-.3F);p.lie_down=lie;
 std::array<float,12> damping;damping.fill(1.5F);p.emergency_kd=damping;p.lie_down_validated=true;
 p.gate0_verified=p.mapping_verified=p.emergency_validated=p.remote_chords_verified=p.robot_supported=p.policy_timing_reviewed=true;
 p.emergency_evidence="OFFLINE_MOCK_ONLY";return p;}
void enable(R3Supervisor& s){
 R3Inputs in;in.ready={true,true,true,true,true,true,true,true,false,true};
 in.measured_q.fill(.4F);in.arm_static_hold=true;in.arm_home_ready=true;in.arm_control_ready=true;in.own_output_healthy=true;
 in.sport_stamp=in.lowstate_stamp=in.remote_stamp=in.arm_stamp=in.target_stamp=at(1);
 in.sport=SportMode::ACTIVE;s.Observe(in,at(1));assert(s.Takeover(at(1)).success);
 in.sport=SportMode::RELEASED;s.Observe(in,at(1));assert(s.EnableOutput(true,at(1)).success);
}
int main(){
 // Receipt never waits for a stalled controller mutex. Original age survives.
 LowStateInbox<> inbox(.5);std::mutex control;
 std::unique_lock held(control);
 auto producer=std::async(std::launch::async,[&]{return inbox.Receive(frame(),at(1));});
 assert(producer.wait_for(std::chrono::seconds(1))==std::future_status::ready);assert(producer.get());
 held.unlock();LowStateInbox<>::Sample s;
 assert(!inbox.Pop(s,at(.9)));assert(inbox.Pop(s,at(1.1)));
 assert(s.stamp==at(1));assert(s.state.snapshot().received==at(1));
 assert(!s.state.Fresh(at(1.501)));assert(!inbox.Pop(s,at(2)));
 // Bad sample followed by a good sample cannot erase a fault while enabled.
 R3Supervisor supervisor(profile());enable(supervisor);
 R3Inputs inputs;LowStateReader reader(.5);R3RemoteCommands remote({"L1","L2","X"},{"L1","L2","B"});
 inbox.Receive(frame(std::numeric_limits<float>::quiet_NaN()),at(1.01));inbox.Receive(frame(2.),at(1.02));
 ApplyLowStateInbox(inbox,reader,remote,inputs,supervisor,at(1.03));
 assert(supervisor.fault_latched());assert(supervisor.last_fault()=="invalid_lowstate");
 assert(inputs.measured_q[0]==2.);assert(inputs.lowstate_stamp==at(1.02));
 // Release between two held button frames must reset the hold duration.
 LowStateInbox<> buttons(.5);R3Supervisor idle(profile());R3RemoteCommands handset({"L1","L2","X"},{"L1","L2","B"});
 buttons.Receive(frame(1.,0x222),at(2)); // L1/L2/B
 ApplyLowStateInbox(buttons,reader,handset,inputs,idle,at(2));
 assert(handset.Poll(at(2))==R3RemoteEvent::NONE);
 buttons.Receive(frame(1.,0x222),at(2.2));
 buttons.Receive(frame(1.,0),at(2.3));
 buttons.Receive(frame(1.,0x222),at(2.4));
 ApplyLowStateInbox(buttons,reader,handset,inputs,idle,at(2.5));
 assert(handset.Poll(at(2.5))==R3RemoteEvent::NONE);
 for(double t:{2.6,2.8,3.,3.2}){
  buttons.Receive(frame(1.,0x222),at(t));ApplyLowStateInbox(buttons,reader,handset,inputs,idle,at(t));
 }
 assert(handset.Poll(at(3.2))==R3RemoteEvent::EMERGENCY);
 // Completed stop chords survive a release in the same drain, even if stale
 // by dispatch time. Emergency wins; consumed edges never replay.
 for(uint16_t mask:{0x422,0x222,0x722}) {
  LowStateInbox<> backlog(.5);R3RemoteCommands commands({"L1","L2","X"},{"L1","L2","B"});
  for(double t:{4.,4.2,4.4,4.6,4.8})assert(backlog.Receive(frame(1.,mask),at(t)));
  assert(backlog.Receive(frame(),at(4.81)));
  ApplyLowStateInbox(backlog,reader,commands,inputs,idle,at(5.5));
  assert(commands.Poll(at(5.5))==(mask==0x422?R3RemoteEvent::CONTROLLED_ABORT:R3RemoteEvent::EMERGENCY));
  assert(commands.Poll(at(5.5))==R3RemoteEvent::NONE);
 }
 // A buffered takeover is admitted only while the latest chord is fresh and
 // still held. An old completed hold must never start motion after release.
 for(int scenario=0;scenario<3;++scenario) {
  LowStateInbox<> backlog(.5);R3RemoteCommands commands({"L1","L2","X"},{"L1","L2","B"});
  for(double t:{6.,6.2,6.4,6.6,6.8})assert(backlog.Receive(frame(1.,0x122),at(t)));
  if(scenario==0)assert(backlog.Receive(frame(),at(6.81)));
  const auto now=at(scenario==1?7.5:6.82);
  ApplyLowStateInbox(backlog,reader,commands,inputs,idle,now);
  assert(commands.Poll(now)==(scenario==2?R3RemoteEvent::TAKEOVER:R3RemoteEvent::NONE));
 }
 // Overflow is persistent and blocks admission; nothing silently overwrites.
 LowStateInbox<2> small(.5);assert(small.Receive(frame(1.),at(1)));assert(small.Receive(frame(2.),at(1.01)));
 assert(!small.Receive(frame(3.),at(1.02)));
 ApplyLowStateInbox(small,reader,handset,inputs,idle,at(1.1));
 assert(small.overflowed());assert(idle.fault_latched());assert(idle.last_fault()=="lowstate_inbox_overflow");
 assert(inputs.measured_q[0]==2.);
 // Concurrent slot reuse: no torn snapshots, no reordered accepted samples.
 LowStateInbox<64> concurrent(.5);constexpr int count=20000;
 std::atomic<int> consumed{0};std::thread writer([&]{
  for(int i=1;i<=count;++i){
   while(i-consumed.load(std::memory_order_acquire)>32)std::this_thread::yield();
   assert(concurrent.Receive(frame(float(i)),at(i*.002)));
  }
 });
 for(int i=1;i<=count;++i){
  LowStateInbox<64>::Sample value;
  while(!concurrent.Pop(value,at(100)))std::this_thread::yield();
  assert(value.state.snapshot().motor_q[0]==float(i));
  assert(value.state.snapshot().motor_dq[11]==float(i*2));assert(value.stamp==at(i*.002));
  consumed.store(i,std::memory_order_release);
 }
 writer.join();assert(!concurrent.overflowed());
}
