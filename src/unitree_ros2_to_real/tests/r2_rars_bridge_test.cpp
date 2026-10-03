#include "rars_bridge.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <filesystem>
#include <unistd.h>
using namespace sim2real;
SafetyTime t(double s){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(s));}
RarsFrame frame(){RarsFrame f;f.communication.connected=true;f.communication.feedback_received=true;
 for(int i=0;i<7;++i){f.joints.motor_id[i]=i+1;f.joints.valid[i]=true;f.joints.position[i]=.1F*(i+1);f.joints.velocity[i]=-.2F*(i+1);}
 return f;
}
int main(){
 rars_arm::ArmConfiguration config;
 for(int i=0;i<7;++i){config.motors[i].zero_offset=.123F*i;config.motors[i].direction=i%2?-1:1;}
 MockRarsTransport mock(config);RarsBridge bridge(mock);
 assert(!bridge.Poll(t(0)).feedback_ready);assert(!bridge.snapshot().target_ready);assert(std::isnan(bridge.snapshot().q[0]));
 auto good=frame();mock.frames.push_back(good);
 const auto& state=bridge.Poll(t(0));assert(state.feedback_ready&&!state.target_ready);
 for(int i=0;i<6;++i){assert(state.q[i]==good.joints.position[i]);assert(state.dq[i]==good.joints.velocity[i]);}
 assert(state.q.size()==6&&!state.per_joint_freshness_proven&&state.zero_calibration_operator_verified);
 assert(state.zero_calibration_source=="SDK_GUI_OPERATOR_SAVED");
 assert(bridge.CalibrationDiagnostic().find("physical_pose_validated=false")!=std::string::npos);
 // Target differs from measured position; nothing substitutes it for feedback.
 AcceptedArmTarget target;target.q.fill(1.5F);target.valid=true;target.accepted=t(0);mock.target=target;
 assert(bridge.Poll(t(.1)).target_ready);assert(bridge.snapshot().q[0]==good.joints.position[0]);
 assert(bridge.Poll(t(.25)).feedback_ready);assert(!bridge.Poll(t(.251)).feedback_ready);assert(!bridge.snapshot().target_ready);
 for(int i=0;i<6;++i){
  auto bad=good;bad.joints.motor_id[i]=0;mock.frames.push_back(bad);assert(!bridge.Poll(t(0)).feedback_ready);
  bad=good;bad.joints.valid[i]=false;mock.frames.push_back(bad);assert(!bridge.Poll(t(0)).feedback_ready);
  bad=good;bad.joints.position[i]=std::numeric_limits<float>::quiet_NaN();mock.frames.push_back(bad);assert(!bridge.Poll(t(0)).feedback_ready);
  bad=good;bad.joints.velocity[i]=std::numeric_limits<float>::infinity();mock.frames.push_back(bad);assert(!bridge.Poll(t(0)).feedback_ready);
  bad=good;bad.joints.error[i]=8;mock.frames.push_back(bad);assert(!bridge.Poll(t(0)).feedback_ready);
 }
 auto gripper=good;gripper.joints.valid[6]=false;gripper.joints.position[6]=std::numeric_limits<float>::quiet_NaN();mock.frames.push_back(gripper);assert(bridge.Poll(t(0)).feedback_ready);
 auto aged=good;aged.communication.feedback_age=std::chrono::milliseconds(251);mock.frames.push_back(aged);assert(!bridge.Poll(t(0)).feedback_ready);
 auto disconnected=good;disconnected.communication.connected=false;mock.frames.push_back(disconnected);assert(!bridge.Poll(t(0)).feedback_ready);
 auto future=good;future.received=t(1);mock.frames.push_back(future);assert(!bridge.Poll(t(0)).feedback_ready);
 target.valid=false;mock.target=target;assert(!bridge.Poll(t(0)).target_ready);
 target.valid=true;target.q[5]=std::numeric_limits<float>::infinity();mock.target=target;assert(!bridge.Poll(t(0)).target_ready);
 mock.target.reset();assert(!bridge.Poll(t(0)).target_ready);
 for(int i=0;i<7;++i){assert(mock.Configuration().motors[i].direction==config.motors[i].direction);assert(mock.Configuration().motors[i].zero_offset==config.motors[i].zero_offset);}
 auto path=std::filesystem::temp_directory_path()/("r2-lease-"+std::to_string(getpid()));std::filesystem::create_directory(path);
 {SerialOwnerLease first(path.string(),"/dev/ttyACM0");assert(first.acquired());SerialOwnerLease second(path.string(),"/dev/ttyACM0");assert(!second.acquired());SerialOwnerLease other(path.string(),"/dev/ttyACM1");assert(other.acquired());}
 const auto device=path/"mock-device";const auto alias=path/"alias-device";
 { FILE* f=fopen(device.c_str(),"w");assert(f);fclose(f);std::filesystem::create_symlink(device,alias);
   SerialOwnerLease real(path.string(),device.string());assert(real.acquired());
   SerialOwnerLease same(path.string(),alias.string());assert(!same.acquired()); }
 {SerialOwnerLease again(path.string(),"/dev/ttyACM0");assert(again.acquired());}
 std::filesystem::remove_all(path);
 std::cout<<"PASS RARS SDK config unchanged, measured six joints/IDs/finite/common age, gripper excluded, accepted targets, mock only, serial lease without opening device\n";
}
