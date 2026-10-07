"""Resolve launch parameters only; never launches nodes, ROS graph or SDK."""
import importlib.util
import os
import tempfile
launch_logs = tempfile.TemporaryDirectory(prefix="fsm-launch-test-")
os.environ["ROS_LOG_DIR"] = launch_logs.name
from pathlib import Path
from launch import LaunchContext
p = Path(__file__).resolve().parents[1] / 'launch/go2_rars01_r3_commissioning.launch.py'
spec = importlib.util.spec_from_file_location('system_launch', p)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
def context(profile=''):
    c = LaunchContext()
    c.launch_configurations.update(config_path='/offline/config', model_path='/offline/policy',
        network_interface='', operation_profile=profile, remote_auto_sequence='true',
        remote_test_mode='', control_mode='', motion_commands_enabled='', read_only='', controlled_stop_lie_down_trial='',
        motion_diagnostics_enabled='false', motion_diagnostics_duration_s='120.0', motion_diagnostics_path='/offline/trace')
    return c
for profile in m.PROFILES:
    actions = m._controller(context(profile))
    assert len(actions) == 1
for value in ('bad', 'REMOTE_TEST'):
    try: m._controller(context(value))
    except ValueError: pass
    else: raise AssertionError('unknown profile did not fail before node creation')
assert m._boolean('true') and not m._boolean('false')
try: m._boolean('maybe')
except ValueError: pass
else: raise AssertionError('invalid bool')
# Parameters of the explicit profile do not get overridden by compatibility defaults.
c=context('rl_zero_test')
actions=m._controller(c)
assert len(actions)==1
print('PASS launch resolves seven profiles and rejects unknown before Node construction')

captured = []
m.Node = lambda **kwargs: captured.append(kwargs) or kwargs
m._controller(context('rl_zero_test'))
params = captured[-1]['parameters'][0]
assert params['operation_profile'] == 'rl_zero_test'
assert not any(k in params for k in ('read_only', 'remote_test_mode', 'motion_commands_enabled', 'control_mode'))
c=context();c.launch_configurations.update(read_only='false', remote_test_mode='true')
m._controller(c)
assert captured[-1]['parameters'][0]['remote_test_mode'] is True

c=context('remote_test');c.launch_configurations['controlled_stop_lie_down_trial']='true'
m._controller(c)
assert captured[-1]['parameters'][0]['controlled_stop_lie_down_trial'] is True
assert 'controlled_stop_lie_down_trial' not in params
assert params['motion_diagnostics_enabled'] is False and params['motion_diagnostics_duration_s']==120.0
c=context('remote_test');c.launch_configurations.update(motion_diagnostics_enabled='true',motion_diagnostics_duration_s='30.0')
m._controller(c)
assert captured[-1]['parameters'][0]['motion_diagnostics_enabled'] is True
assert captured[-1]['parameters'][0]['motion_diagnostics_duration_s']==30.0
