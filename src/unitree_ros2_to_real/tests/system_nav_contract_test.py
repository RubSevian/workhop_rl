"""Differential source oracle for the actual NAV receive body: no ROS/robot."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/go2_r3_commissioning.cpp').read_text()
a=s.index('   const std::array<double,3> requested{m->twist')
b=s.index('   navigation_stamp_=SafetyClock::now();',a)+len('   navigation_stamp_=SafetyClock::now();')
assert s[a:b]+'\n' == (root/'tests/fixtures/r3_baseline/nav_handler_body.cpp.txt').read_text()
assert 'rclcpp::QoS(1)' in s and 'NavigationFresh(now)' in s
assert 'age<=limit' in (root/'src/r3_commissioning.cpp').read_text()
print('PASS actual NAV finite/header/freshness/clamp receive body unchanged from 39b78c9')

refresh=s[s.index(' void Refresh('):s.index(' void StopPublisher(')]
assert 'count_publishers(' not in refresh and 'service_is_ready()' not in refresh
print('PASS 500Hz Refresh contains no blocking ROS graph/service queries')
