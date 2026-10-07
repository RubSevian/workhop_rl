"""Check actual node transport wiring; never creates a ROS or LowCmd publisher."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/go2_r3_commissioning.cpp').read_text()
stop=s[s.index(' void StopPublisher()'):s.index(' void PolicyTick()')]
assert stop.index('SystemEvent::DISABLE_OUTPUT') < stop.index('output_.reset()') < stop.index('lease_.reset()') < stop.index('SystemEvent::OUTPUT_STOPPED')
assert 'if(!output_&&!lease_)' in stop
set_output=s[s.index(' R3Reply SetOutput('):s.index(' void SdkTick(')]
assert set_output.index('count_publishers("/lowcmd")!=0') < set_output.index('std::make_unique<OutputLease>') < set_output.index('SystemEvent::ENABLE_OUTPUT') < set_output.index('create_publisher<unitree_go::msg::LowCmd>')
assert 'StopPublisher();' in set_output
assert 'if(output_&&!supervisor_->output_enabled())StopPublisher();' in s
assert 'declare_parameter<bool>("controlled_stop_lie_down_trial",profile.controlled_stop_lie_down_trial,immutable)' in s
assert 'generation!=supervisor_->orchestration_generation()' in s
assert 'motor_power_off_confirmed' in s and 'lowcmd_lease_present' in s
print('PASS node stop/publisher/lease/ack and foreign-owner/acquire/create order, immutable trial/status/generation guards')

import yaml
for name in ('config/go2_rars01_real.yaml', 'deployment/workspace_runtime/r3_first_rl_zero.yaml'):
    r=yaml.safe_load((root/name).read_text())['real_deployment']['r3_commissioning']
    assert r['controlled_stop']['lie_down_trial'] is False
    assert r['lie_down']['operator_validated'] is False
    assert r['lie_down']['motor_q']==[.01,1.3,-2.7,-.01,1.3,-2.7,-.3,1.3,-2.7,.3,1.3,-2.7]
print('PASS production and operator config fail-closed trial defaults and exact approved target')

assert s.index('output_->publish(*packet);++sent_;') < s.index('supervisor_->NotifyPacketPublished(*packet,SafetyClock::now());')
assert 'passive_command_sent' in s and 'last_commanded_leg_mode' in s
print('PASS passive publication acknowledgement follows actual publish return')

policy=s[s.index('  if(infer) {'):s.index('  std::lock_guard lock(mutex_);const auto now=SafetyClock::now();',s.index('  if(infer) {'))]
assert policy.index('torch::InferenceMode inference;') < policy.index('a.ResetPolicyState()') < policy.index('a.Act()')
warm=s[s.index('// Warm TorchScript'):s.index('  home_tolerance_=')]
assert 'torch::InferenceMode inference;' in warm
print('PASS inference guard in both startup and worker reset/history/Act path')
