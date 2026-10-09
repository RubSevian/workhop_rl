"""Check actual ROS NAV gate wiring; numeric contracts are tested in C++."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/go2_r3_commissioning.cpp').read_text()
a=s.index('  nav_sub_=create_subscription<geometry_msgs::msg::TwistStamped>')
b=s.index('  io_group_=',a)
body=s[a:b]
for required in ('rclcpp::QoS(1)', 'std::isfinite', 'm->header.frame_id!="vehicle"', 'm->header.stamp.nanosec>=1000000000u', 'this->now()-rclcpp::Time(m->header.stamp)', 'navigation_gate_->Accept(requested,message_age', 'SystemState::CONTROLLED_STOP'):
    assert required in body,required
io=s[s.index('  io_timer_='):s.index('  policy_group_=')]
assert 'navigation_gate_->Get(now)' in io and 'inputs_.navigation_ready?' in io
assert io.index('supervisor_->NavigationCommand') < io.index('supervisor_->Tick(now)')
refresh=s[s.index(' void Refresh('):s.index(' void StopPublisher(')]
assert 'navigation_gate_->Ready(now)&&nav_cancel_available_' in refresh
assert 'count_publishers(' not in refresh and 'service_is_ready()' not in refresh
assert 'navigation_gate_->Invalidate();nav_cancel_pending_=true' in s
print('PASS actual NAV stamped/frame/finite gate, IO zero fallback, cancel invalidation and nonblocking Refresh')
