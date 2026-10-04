#include "rars_auto_home.hpp"
#include <cassert>
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <unistd.h>
using namespace sim2real;
ArmTime t(double value){return ArmTime{}+std::chrono::duration_cast<ArmClock::duration>(std::chrono::duration<double>(value));}
struct Journal:AutoHomeJournal {
 int attempts=0,faults=0;bool writable=true;std::string blocked;
 std::string BlockReason() const override{return blocked;}
 bool RecordEnableAttempt() override{++attempts;return writable;}
 bool RecordFault(const std::string&) override{++faults;return writable;}
};
struct Transport:AutoHomeTransport {
 rars_arm::JointState joints;rars_arm::CommunicationStatus comm;
 int connects=0,enables=0,sends=0;bool connect_ok=true,enable_ok=true,send_ok=true,read_ok=true,reset_on_enable=false;
 std::vector<rars_arm::RarsArm::MotorValues> targets;
 Transport(){comm.connected=comm.feedback_received=true;for(size_t i=0;i<7;++i){joints.valid[i]=true;joints.motor_id[i]=i+1;joints.error[i]=0;joints.position[i]=.02F;}}
 bool Connect() override{++connects;comm.connected=connect_ok;return connect_ok;}
 bool Enable() override{++enables;comm.enabled=enable_ok;if(reset_on_enable){read_ok=false;comm.feedback_received=false;}if(enable_ok)joints.error.fill(1);return enable_ok;}
 bool Send(const rars_arm::RarsArm::MotorValues& target) override{++sends;targets.push_back(target);return send_ok;}
 bool Read(rars_arm::JointState& out) override{if(!read_ok)return false;out=joints;return true;}
 rars_arm::CommunicationStatus Status() const override{return comm;}
 std::string Error() const override{return "mock_error";}
};
AutoHomeConfig config(){AutoHomeConfig c;c.enabled=true;return c;}
void start(AutoHomeController& c){c.Tick(t(0));c.Tick(t(9.999));assert(c.status().state==AutoHomeState::STARTUP_DELAY);c.Tick(t(10));c.Tick(t(10.01));assert(c.status().arm_home_ready);}
void startup(){
 Transport transport;Journal journal;AutoHomeController c(config(),transport,journal);
 c.Tick(t(0));c.Tick(t(9.999));assert(transport.enables==0&&transport.sends==0);
 c.Tick(t(10));assert(transport.enables==1&&journal.attempts==1&&transport.sends==0);
 for(int i=1;i<=100;++i)c.Tick(t(10+i*.01));
 assert(transport.enables==1);assert(transport.sends>=99&&transport.sends<=100);
 for(const auto& target:transport.targets)for(float value:target)assert(value==0); // includes gripper
 assert(c.status().arm_home_ready&&c.status().joints.position[6]==.02F);
 const auto accepted=c.status().accepted_stamp;c.Tick(t(11.001));assert(c.status().accepted_stamp==accepted);
 Transport delayed;delayed.comm.connected=false;delayed.connect_ok=false;Journal j;AutoHomeController waiting(config(),delayed,j);
 waiting.Tick(t(0));waiting.Tick(t(.2));assert(delayed.connects==1&&delayed.enables==0);
 delayed.connect_ok=true;waiting.Tick(t(1));waiting.Tick(t(1.01));waiting.Tick(t(11.0));assert(delayed.enables==0);
 waiting.Tick(t(11.01));assert(delayed.enables==1);
 Transport reset;Journal jr;AutoHomeController countdown(config(),reset,jr);countdown.Tick(t(0));
 reset.comm.connected=false;reset.connect_ok=false;countdown.Tick(t(9));reset.comm.connected=true;countdown.Tick(t(10));
 countdown.Tick(t(19.999));assert(reset.enables==0);countdown.Tick(t(20));assert(reset.enables==1);
 Transport ro;Journal jro;auto disabled=config();disabled.enabled=false;AutoHomeController readonly(disabled,ro,jro);
 readonly.Tick(t(0));readonly.Tick(t(100));assert(ro.enables==0&&ro.sends==0&&!readonly.status().arm_home_ready);
}
void readiness(){
 for(int bad=0;bad<10;++bad){Transport tr;Journal j;AutoHomeController c(config(),tr,j);start(c);
  switch(bad){case 0:tr.read_ok=false;break;case 1:tr.joints.error[6]=0;break;
   case 2:tr.joints.error[6]=8;break;case 3:tr.joints.motor_id[6]=6;break;
   case 4:tr.joints.position[6]=std::numeric_limits<float>::quiet_NaN();break;
   case 5:tr.joints.velocity[6]=std::numeric_limits<float>::infinity();break;
   case 6:tr.comm.watchdog_tripped=true;break;case 7:tr.comm.stm32_watchdog_tripped=true;break;
   case 8:tr.comm.connected=false;break;case 9:tr.comm.enabled=false;break;}
  c.Tick(t(11.1));assert(!c.status().arm_home_ready&&c.status().state==AutoHomeState::FAULT_LATCHED);
  const int sent=tr.sends;c.Tick(t(12));c.Tick(t(50));assert(tr.enables==1&&tr.sends==sent);
 }
 Transport far;Journal j;AutoHomeController c(config(),far,j);start(c);
 far.joints.position[6]=.16F;c.Tick(t(10.02));assert(!c.status().arm_home_ready&&c.status().state==AutoHomeState::HOLD_HOME);
 far.joints.position[6]=.05F;c.Tick(t(10.03));assert(c.status().arm_home_ready);
 Transport stale;Journal js;AutoHomeController st(config(),stale,js);start(st);st.Tick(t(10.3));
 assert(st.status().state==AutoHomeState::FAULT_LATCHED&&!st.status().target_fresh);
 Transport failure;Journal jf;AutoHomeController failed(config(),failure,jf);start(failed);
 auto last=failed.status().accepted_stamp;failure.send_ok=false;failed.Tick(t(10.02));
 assert(!failed.status().target_fresh&&!failed.status().arm_home_ready&&failed.status().accepted_stamp==last);
 Transport enablefail;enablefail.enable_ok=false;Journal je;AutoHomeController ef(config(),enablefail,je);
 ef.Tick(t(0));ef.Tick(t(10));ef.Tick(t(20));assert(enablefail.enables==1&&ef.status().state==AutoHomeState::FAULT_LATCHED);
 Transport unwritable;Journal jw;jw.writable=false;AutoHomeController uw(config(),unwritable,jw);uw.Tick(t(0));uw.Tick(t(10));assert(unwritable.enables==0);
 Transport postenable;postenable.reset_on_enable=true;Journal jp;AutoHomeController pe(config(),postenable,jp);
 pe.Tick(t(0));pe.Tick(t(10));pe.Tick(t(10.2));assert(pe.status().state==AutoHomeState::HOLD_HOME&&!pe.status().arm_home_ready&&postenable.sends==1);
 postenable.read_ok=true;postenable.comm.feedback_received=true;pe.Tick(t(10.3));assert(pe.status().arm_home_ready&&postenable.enables==1);
 Transport grace;grace.reset_on_enable=true;Journal jg;AutoHomeController g(config(),grace,jg);
 g.Tick(t(0));g.Tick(t(10));g.Tick(t(11.1));assert(g.status().state==AutoHomeState::FAULT_LATCHED&&grace.sends==0);
 Transport fifty;Journal j50;auto c50=config();c50.command_rate_hz=50;AutoHomeController hz50(c50,fifty,j50);start(hz50);
 hz50.Tick(t(10.015));assert(fifty.sends==1);hz50.Tick(t(10.02));assert(fifty.sends==2);
 Transport invalid;invalid.joints.valid[6]=false;Journal ji;AutoHomeController iv(config(),invalid,ji);iv.Tick(t(0));iv.Tick(t(10));iv.Tick(t(10.01));assert(invalid.enables==1&&iv.status().state==AutoHomeState::FAULT_LATCHED);
 // Real STM supplies no feedback until enable/stream: countdown must proceed.
 Transport missing;missing.read_ok=false;Journal jm;AutoHomeController miss(config(),missing,jm);miss.Tick(t(0));miss.Tick(t(9.999));assert(missing.enables==0);
 miss.Tick(t(10));assert(missing.enables==1);miss.Tick(t(10.01));assert(missing.sends==1&&!miss.status().arm_home_ready);
 miss.Tick(t(10.02));assert(missing.sends==2&&!miss.status().arm_home_ready);
 missing.read_ok=true;missing.comm.feedback_received=true;miss.Tick(t(10.03));assert(miss.status().arm_home_ready&&missing.enables==1);
 Transport silent;silent.read_ok=false;Journal jsi;AutoHomeController si(config(),silent,jsi);
 si.Tick(t(0));si.Tick(t(10));for(int i=1;i<=110;++i)si.Tick(t(10+i*.01));
 assert(si.status().state==AutoHomeState::FAULT_LATCHED&&!si.status().arm_home_ready&&silent.enables==1);
 const int silent_sends=silent.sends;si.Tick(t(50));assert(silent.sends==silent_sends&&silent.enables==1);
}
void journal(){
 auto directory=std::filesystem::temp_directory_path()/("rars-journal-"+std::to_string(getpid()));std::filesystem::create_directory(directory);
 const auto path=(directory/"state").string();
 {FileAutoHomeJournal a(path,"boot1");assert(a.BlockReason().empty());assert(a.RecordEnableAttempt());}
 {FileAutoHomeJournal b(path,"boot1");assert(!b.BlockReason().empty());}
 {FileAutoHomeJournal fresh(path,"boot2");assert(fresh.BlockReason().empty());assert(fresh.RecordFault("hardware_fault"));}
 {FileAutoHomeJournal fault(path,"boot3");assert(!fault.BlockReason().empty());}
 std::filesystem::remove_all(directory);
}
int main(){startup();readiness();journal();std::cout<<"PASS AUTO HOME mock: delay/reconnect, enable once, seven zeros, continuous stream, measured readiness, stale/send/disabled/ID/NaN/watchdogs, persistent fault; no hardware\n";}
