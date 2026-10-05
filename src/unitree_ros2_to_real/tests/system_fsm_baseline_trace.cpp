#ifdef FROZEN_BASELINE
#include "baseline_r3_commissioning.hpp"
namespace controller=sim2real::baseline;
#else
#include "r3_commissioning.hpp"
namespace controller=sim2real;
#endif
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace sim2real;
SafetyTime at(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
template<class T> T observation(double s){T in;in.ready={true,true,true,true,true,true,true,true,false,true};in.sport=s<.1?SportMode::ACTIVE:SportMode::RELEASED;
 in.sport_stamp=in.lowstate_stamp=in.remote_stamp=in.arm_stamp=in.target_stamp=at(s);in.arm_static_hold=in.arm_home_ready=true;
 if constexpr(requires{in.arm_control_ready;})in.arm_control_ready=true;
 for(int i=0;i<12;++i)in.measured_q[i]=.3F+i*.01F+.002F*std::sin(s);return in;}
int main(int argc,char** argv){if(argc!=2)return 2;auto p=controller::LoadR3Profile(YAML::LoadFile(argv[1]));
 p.gate0_verified=p.mapping_verified=p.emergency_validated=p.remote_chords_verified=p.robot_supported=p.policy_timing_reviewed=true;p.emergency_evidence="OFFLINE_FIXTURE";
 controller::R3Supervisor f(p);f.Observe(observation<controller::R3Inputs>(0),at(0));if(!f.StartRemoteSequence(at(0)).success)return 3;
 std::optional<controller::R3State> previous_state;
 std::optional<controller::PolicyTicket> pending;int complete=0,release=0;
 for(int i=0;i<5600;++i){double s=i*.002;auto now=at(s);f.Observe(observation<controller::R3Inputs>(s),now);
  if(pending&&i==complete){auto q=p.stand;for(auto& x:q)x+=.01F;if(!f.PolicyResult(*pending,q,8,now))return 4;pending.reset();std::cout<<"accepted "<<i<<"\n";}
  if(auto packet=f.Tick(now)){std::cout<<i<<" ";for(auto b:SerializeLowCmd(*packet))std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);std::cout<<std::dec<<"\n";}
  if(!previous_state||*previous_state!=f.state()){std::cout<<"transition "<<controller::R3StateName(f.state())<<" "<<i<<"\n";previous_state=f.state();}
  if(i%10==0){auto action=f.RemoteSequenceNext(now);
   if(action==controller::R3SequenceAction::RELEASE_SPORT)++release;
   if(action==controller::R3SequenceAction::ENABLE_OUTPUT&&s>=.6&&!f.EnableOutput(true,now).success)return 5;
   if(action==controller::R3SequenceAction::STAND&&!f.RequestStand(now).success)return 6;
   if(action==controller::R3SequenceAction::RL&&!f.RequestRl(now).success)return 7;
   if(f.ConsumePolicyReset())std::cout<<"reset "<<i<<"\n";
   if(f.NeedsPolicy()){f.RemoteTestCommand({},now);pending=f.BeginPolicy(now);if(pending)complete=i+4;}
  }
 }std::cout<<"release_count "<<release<<"\n";return release==1?0:8;}
