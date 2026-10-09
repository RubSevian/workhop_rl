"""Source wiring contract; gate/session/lifecycle behavior has numeric C++ tests."""
from pathlib import Path
s=(Path(__file__).resolve().parents[1]/'src/go2_r3_commissioning.cpp').read_text()
a=s.index('  mission_readiness_sub_=');b=s.index('  arm_home_client_=',a)
body=s[a:b]
for required in ('rclcpp::QoS(1)', 'size()>4096', 'navigation_gate_->Observe', 'navigation_gate_->Accept(session', 'command_age_s', 'c.size()!=3', 'SystemState::CONTROLLED_STOP'):
    assert required in body,required
assert 'nav_sub_=create_subscription' not in s
io=s[s.index('  io_timer_='):s.index('  policy_group_=')]
assert 'navigation_gate_->Get(now)' in io and 'inputs_.navigation_ready?' in io
assert io.index('supervisor_->NavigationCommand') < io.index('supervisor_->Tick(now)')
refresh=s[s.index(' void Refresh('):s.index(' void StopPublisher(')]
assert 'navigation_gate_->Ready(now)&&nav_cancel_available_&&nav_input_unique_' in refresh
assert 'count_publishers(' not in refresh and 'service_is_ready()' not in refresh
assert 'navigation_gate_->Invalidate();nav_cancel_pending_=true' in s
assert 'PolicyStateTransaction policy_state(a)' in s and 'if(accepted)policy_state.Commit()' in s
print('PASS atomic session/command envelope, IO zero fallback, cancel, worker transaction')
