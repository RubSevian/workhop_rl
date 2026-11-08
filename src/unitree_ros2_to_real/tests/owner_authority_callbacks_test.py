"""Compile actual owner callback bodies; no ROS node or serial device is opened."""
from pathlib import Path
import subprocess
import tempfile


def test_replayed_status_preserves_lease_but_invalid_new_status_revokes():
    source = (Path(__file__).parents[1]/'src/rars_r3_owner.cpp').read_text()
    callbacks = []
    for name in ('r3', 'nav', 'health'):
        start = source.index(name+'_sub_=create_subscription')
        body = source[source.index('[this]', start):].split('\n  });', 1)[0].split('{', 1)[1]
        callbacks.append('auto '+name+'=[&](std::shared_ptr<Message> msg){'+body+'\n};')
    authority = source.split('bool Authority(ArmTime now,bool home_request=false,bool retain_completed_hold=false){',1)[1].split('\n }',1)[0]
    code = r'''
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>
#include <memory>
using ArmClock=std::chrono::steady_clock;
using ArmTime=ArmClock::time_point;
long long Ns(ArmTime t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
bool ControlledStopHomePhaseAllowed(const std::string&){return false;}
struct Status{double feedback_age_s=.01;};
struct Home{Status status(){return {};}};
struct Mission{void Cancel(Status,const char*){} bool CompletedHomeHoldAllowed(const std::string&){return false;}};
struct Scan{void Reset(){}};
struct Message{std::string data;};
int main(){
Home home;auto home_=&home;Mission mission;auto mission_=&mission;Scan scan_home_reference_;
bool r3_authority_=false,r3_planned_disable_ready_=false,r3_active_=false,r3_stopping_=false,r3_home_phase_allowed_=false;
bool nav_ready_=false,health_ready_=false,health_manipulating_=false;
ArmTime r3_stamp_{},nav_stamp_{},health_stamp_{},health_callback_stamp_{};
double health_callback_gap_s_=0,health_producer_gap_s_=0,health_age_at_callback_s_=0,health_decode_s_=0;
std::string r3_nav_session_,r3_session_,mission_session_,retired_session_,nav_session_,health_nav_session_,health_phase_;
unsigned long long health_goal_revision_=0;
'''+ '\n'.join(callbacks) + '\nauto count_publishers=[](const char*){return size_t(1);};\ndouble mission_ttl_=.25,mission_feedback_ttl_=.25;\nauto authority=[&](ArmTime now,bool home_request=false,bool retain_completed_hold=false){'+authority+'\n};' + r'''
auto send=[](auto& callback,const std::string& s){callback(std::make_shared<Message>(Message{s}));};
const std::string r3_fields=R"(, navigation_session_id: n, controller_session_id: c, state: ACTIVE, phase: RL_ZERO, motion_command: [0,0,0], operation_profile: full_mission, capabilities: {arm_motion: true}, fault_latched: false})";
const std::string nav_fields=R"(, session_id: n, navigation_ready: true})";
const std::string health_fields=R"(, session_id: m, navigation_session_id: n, ready: true, goal_revision: 1, phase: ARM_SEARCH})";
auto check=[&](auto& callback,auto& stamp,auto& ready,const auto& fields){
 send(callback,"{observed_ns: 100"+fields);assert(ready&&Ns(stamp)==100);
 // Neither duplicates nor delayed packets clear authority or extend its lease.
 send(callback,"{observed_ns: 100}");assert(ready&&Ns(stamp)==100);
 send(callback,"{observed_ns: 99}");assert(ready&&Ns(stamp)==100);
 send(callback,"{observed_ns: 101}");assert(!ready); // New malformed status fails closed.
 send(callback,"{observed_ns: 102"+fields);assert(ready&&Ns(stamp)==102);
 send(callback,"not: [valid");assert(!ready);
};
check(r3,r3_stamp_,r3_authority_,r3_fields);
check(nav,nav_stamp_,nav_ready_,nav_fields);
check(health,health_stamp_,health_ready_,health_fields);
send(r3,"{observed_ns: 1000000000"+r3_fields);
send(nav,"{observed_ns: 1000000000"+nav_fields);
send(health,"{observed_ns: 1000000000"+health_fields);
assert(authority(ArmTime(std::chrono::nanoseconds(1250000000))));
// A delayed duplicate cannot extend the 250 ms lease.
send(health,"{observed_ns: 1000000000}");
assert(!authority(ArmTime(std::chrono::nanoseconds(1250000001))));
assert(!authority(ArmTime(std::chrono::nanoseconds(1320000000))));
// The RL owner has no MissionStream. Changing controller sessions must be safe.
mission_=nullptr;
const std::string lying=R"({observed_ns: 2000000000, navigation_session_id: '', controller_session_id: remote,
 state: SYSTEM_HOLD, phase: LIE_DOWN_HOLD, fault_latched: false,
 output_enabled: false, output_stop_confirmed: true, lowcmd_lease_present: false,
 passive_sequence_complete: true, motion_command: [0,0,0], operation_profile: remote_test})";
send(r3,lying);assert(r3_planned_disable_ready_&&!r3_authority_);
auto ns=2000000000LL;
for(const auto& field:{"output_enabled","output_stop_confirmed","lowcmd_lease_present","passive_sequence_complete","fault_latched"}){
 auto bad=YAML::Load(lying);bad["observed_ns"]=++ns;bad[field]=!bad[field].as<bool>();
 send(r3,YAML::Dump(bad));assert(!r3_planned_disable_ready_);
 bad.remove(field);bad["observed_ns"]=++ns;
 send(r3,YAML::Dump(bad));assert(!r3_planned_disable_ready_);
}
auto pd=YAML::Load(lying);pd["observed_ns"]=++ns;pd["state"]="CONTROLLED_STOP";
pd["phase"]="ARM_DISABLE";pd["pd_hold_confirmed"]=true;pd["output_enabled"]=true;
send(r3,YAML::Dump(pd));assert(r3_planned_disable_ready_); // Existing failed-arm path retained.
pd["observed_ns"]=++ns;pd["pd_hold_confirmed"]=false;
send(r3,YAML::Dump(pd));assert(!r3_planned_disable_ready_);

}
'''
    with tempfile.TemporaryDirectory() as directory:
        cpp=Path(directory)/'callbacks.cpp';binary=Path(directory)/'callbacks'
        cpp.write_text(code)
        subprocess.run(['g++','-std=c++17',str(cpp),'-lyaml-cpp','-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)


def test_planned_disable_service_rechecks_freshness_home_and_unique_controller():
    source = (Path(__file__).parents[1]/'src/rars_r3_owner.cpp').read_text()
    start = source.index('  planned_disable_=create_service')
    body = source[source.index('[this]', start):].split('\n  });', 1)[0].split('{', 1)[1]
    code = r'''
#include <chrono>
#include <string>
#include <memory>
#include <cassert>
using ArmClock=std::chrono::steady_clock;
using ArmTime=ArmClock::time_point;
long long Ns(ArmTime t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
struct Response{bool success=false;std::string message;};
struct Home{
 bool ready=true,attempted=false,send_ok=true;int disables=0;
 bool planned_disable_attempted(){return attempted;}
 bool ReturnHome(ArmTime){return ready;}
 int status(){return 0;}
 bool PlannedDisable(bool,ArmTime){++disables;attempted=true;return send_ok;}
};
struct Mission{void Cancel(int,const char*){}};
int main(){
 Home home;auto home_=&home;Mission* mission_=nullptr;
 auto response=std::make_shared<Response>();
 bool r3_planned_disable_ready_=true,r3_stopping_=false,home_requested_=true;
 ArmTime r3_stamp_=ArmClock::now();double mission_ttl_=.25;
 long long plan_cancelled_ns_=0;std::string mission_session_,retired_session_;
 int publishers=1;auto count_publishers=[&](const char*){return publishers;};
 auto RevokeRetreatTrial=[](const char*){};
''' + 'auto call=[&](){' + body + r'''
};
 r3_planned_disable_ready_=false;call();assert(!response->success&&home.disables==0);
 r3_planned_disable_ready_=true;r3_stamp_=ArmClock::now()-std::chrono::seconds(1);
 call();assert(!response->success&&home.disables==0);
 r3_stamp_=ArmClock::now()+std::chrono::seconds(1);call();assert(!response->success&&home.disables==0);
 r3_stamp_=ArmClock::now();publishers=2;call();assert(!response->success&&home.disables==0);
 publishers=1;home.ready=false;call();assert(!response->success&&home.disables==0);
 home.ready=true;call();assert(response->success&&home.disables==1&&!home_requested_);
 // Failed-arm PD stop is deliberately allowed without measured HOME.
 home={};home.ready=false;r3_stopping_=true;call();assert(response->success&&home.disables==1);
 home={};home.send_ok=false;call();assert(!response->success&&home.disables==1);
}
'''
    with tempfile.TemporaryDirectory() as directory:
        cpp=Path(directory)/'service.cpp';binary=Path(directory)/'service'
        cpp.write_text(code)
        subprocess.run(['g++','-std=c++17',str(cpp),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)


if __name__ == '__main__':
    test_replayed_status_preserves_lease_but_invalid_new_status_revokes()
    test_planned_disable_service_rechecks_freshness_home_and_unique_controller()
    print('PASS owner callbacks: authority, remote finish, freshness, HOME and disable guards; no hardware')
