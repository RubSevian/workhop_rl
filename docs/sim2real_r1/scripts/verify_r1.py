"""Offline identity/isolation check; imports no robot or ROS runtime."""
import hashlib
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[1]
manifest = yaml.safe_load((root / 'weights/POLICY_MANIFEST.yaml').read_text())
policy = root / 'weights' / manifest['policy']['file']
if hashlib.sha256(policy.read_bytes()).hexdigest() != manifest['policy']['sha256']:
    raise RuntimeError('Policy identity mismatch')
w = root / 'repos/workhop_rl'
base = w / 'src/unitree_ros2_to_real'
real = yaml.safe_load((base / 'config/go2_rars01_real.yaml').read_text())
sim = yaml.safe_load((base / 'config/go2_rars01_unified.yaml').read_text())
if real['go2_rars01'] != sim['go2_rars01']:
    raise RuntimeError('Unified RL configuration changed')
if real['real_deployment']['enable_actuator_output'] is not False:
    raise RuntimeError('Output must be disabled')
for part in ['src/ros2_rl_go2.cpp', 'include/ros2_rl_go2.hpp',
             'src/real_controller_core.cpp', 'include/real_controller_core.hpp']:
    content = (base / part).read_text()
    for forbidden in ['create_publisher<unitree_go::msg::LowCmd', 'cmd_puber',
                      'ChannelFactory::', 'ServiceSwitch(', 'send_command(',
                      'rars_arm_py', 'SerialPort']:
        if forbidden in content:
            raise RuntimeError(f'Actuator path in R1: {part}: {forbidden}')
for part in ['src/unitree_ros2', 'src/unitree_mujoco']:
    if not (w / part / 'COLCON_IGNORE').is_file():
        raise RuntimeError(f'Old/sim package discovery enabled: {part}')
print('verify_r1: PASS policy SHA, unified config parity, default output disabled, source transport isolation')
