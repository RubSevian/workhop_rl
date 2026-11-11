"""Compile actual infrastructure polling body with deterministic ROS fakes."""
from pathlib import Path
import subprocess
import tempfile


def test_discovery_does_not_hold_control_lock_or_renew_old_graph():
    source=(Path(__file__).parents[1]/'src/go2_r3_commissioning.cpp').read_text()
    poll=source.split(' void PollInfrastructure() {',1)[1].split('\n void PortTick()',1)[0]
    policy=source.split(' void PolicyTick(',1)[1].split('\n ControlDiagnostics CaptureStatus(',1)[0]
    assert 'count_publishers(' not in policy and 'service_is_ready()' not in policy
    assert 'SdkTick(' not in policy and 'SequenceTick(' not in policy
    code=r'''
#include <chrono>
#include <cassert>
#include <stdexcept>
#include <string>
#include <cstdint>
using SafetyTime=std::chrono::steady_clock::time_point;
struct SafetyClock {static SafetyTime now(){return current;} static inline SafetyTime current{};};
double Age(SafetyTime a,SafetyTime b){return std::chrono::duration<double>(a-b).count();}
namespace rclcpp {bool ok(){return true;} namespace exceptions {struct RCLError:std::runtime_error{using std::runtime_error::runtime_error;};}}
struct FakeLock{bool held=false;};
struct Trace{Trace* get(){return this;}};
struct ControlLock{FakeLock& m;ControlLock(FakeLock& m,Trace*,int):m(m){assert(!m.held);m.held=true;}~ControlLock(){m.held=false;}};
struct Client {FakeLock& lock;bool ready=true;bool service_is_ready(){assert(!lock.held);return ready;}};
struct Node {
 FakeLock mutex_;Trace control_latency_;Client nav{mutex_},manip{mutex_};
 Client* nav_cancel_client_=&nav;Client* manipulation_cancel_client_=&manip;
 SafetyTime last_infrastructure_poll_{},graph_stamp_{};int64_t policy_period_ns_=20000000;
 const char* navigation_readiness_topic_="/go2/mission/readiness";
 size_t cached_publisher_count_=0;bool nav_input_unique_=false,nav_cancel_available_=false,manipulation_cancel_available_=false;
 double infrastructure_poll_s_=0;int queries=0,refreshes=0;bool fail=false;int delay_ms=0;
 size_t count_publishers(const char*) {assert(!mutex_.held);++queries;if(fail)throw rclcpp::exceptions::RCLError("discovery failed");SafetyClock::current+=std::chrono::milliseconds(delay_ms);return 1;}
 void SdkTick(SafetyTime){assert(mutex_.held);}
 void Refresh(SafetyTime){assert(mutex_.held);++refreshes;}
 // Polling consumes remote decisions through HandleRemote, which refreshes inputs.
 bool HandleRemote(SafetyTime now){Refresh(now);return false;}
 void SequenceTick(SafetyTime){assert(mutex_.held);}
 void PollInfrastructure(){
'''+poll+r'''
};
int main(){
 Node node;SafetyClock::current=SafetyTime(std::chrono::seconds(1));
 node.PollInfrastructure();assert(node.queries==2&&node.refreshes==1);
 const auto original=node.graph_stamp_;
 SafetyClock::current+=std::chrono::milliseconds(10);node.PollInfrastructure();assert(node.queries==2);
 SafetyClock::current+=std::chrono::milliseconds(10);node.delay_ms=30;node.PollInfrastructure();
 assert(node.queries==4&&Age(SafetyClock::now(),node.graph_stamp_)>=.06);
 // Discovery latency must be counted against the graph lease, not hidden.
 assert(node.graph_stamp_==original+std::chrono::milliseconds(20));
 auto prior=node.graph_stamp_;node.fail=true;
 try{node.PollInfrastructure();assert(false);}catch(const rclcpp::exceptions::RCLError&){}
 assert(node.graph_stamp_==prior&&!node.mutex_.held);
}
'''
    with tempfile.TemporaryDirectory() as d:
        cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(code)
        subprocess.run(['g++','-std=c++17',str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
