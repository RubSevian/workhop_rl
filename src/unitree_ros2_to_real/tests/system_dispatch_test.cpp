#include "r3_commissioning.hpp"
#include <cassert>
using namespace sim2real;
int main(){R3Profile p;p.kp.fill(40);p.kd.fill(1);p.rl_kp.fill(25);p.rl_kd.fill(1);
 R3Supervisor s(p);assert(s.system_state()==SystemState::STANDBY);
 s.Dispatch(SystemEvent::ADVANCE_PHASE,{},SystemPhase::SPORT_RELEASE_REQUIRED);assert(s.system_state()==SystemState::TAKEOVER);
 s.Dispatch(SystemEvent::ADVANCE_PHASE,{},SystemPhase::RL_ZERO);assert(s.system_state()==SystemState::ACTIVE);
 s.Dispatch(SystemEvent::ADVANCE_PHASE,{},SystemPhase::ARM_RETURN_HOME);assert(s.system_state()==SystemState::CONTROLLED_STOP);
 s.Dispatch(SystemEvent::ADVANCE_PHASE,{},SystemPhase::LIE_DOWN_HOLD);assert(s.system_state()==SystemState::SYSTEM_HOLD);
 s.Fault("test",{});assert(s.system_state()==SystemState::EMERGENCY_FAULT);
 assert(!s.Dispatch(SystemEvent::REQUEST_A,{}).success);
}
