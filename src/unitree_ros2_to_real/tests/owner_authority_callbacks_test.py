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
using ArmTime=std::chrono::steady_clock::time_point;
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
ArmTime r3_stamp_{},nav_stamp_{},health_stamp_{};
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
}
'''
    with tempfile.TemporaryDirectory() as directory:
        cpp=Path(directory)/'callbacks.cpp';binary=Path(directory)/'callbacks'
        cpp.write_text(code)
        subprocess.run(['g++','-std=c++17',str(cpp),'-lyaml-cpp','-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
