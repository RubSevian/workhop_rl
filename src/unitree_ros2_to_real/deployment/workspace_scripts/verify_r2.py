"""Offline R2 defaults and IO isolation checks. No ROS node or hardware imports."""
from pathlib import Path
import subprocess
import yaml
root = Path(__file__).resolve().parents[1]
subprocess.run(['python3', str(root/'scripts/verify_r1.py')], check=True)
p = root/'repos/workhop_rl/src/unitree_ros2_to_real'
d = yaml.safe_load((p/'config/go2_rars01_real.yaml').read_text())['real_deployment']
assert d['safety'] == {'startup_disarmed': True, 'actuator_output_default': False}
assert d['remote']['takeover_chord'] == ['L1', 'L2', 'A']
assert d['remote']['emergency_chord'] == []
assert d['sport_mode']['require_release_verified'] is True
assert d['rars01']['per_joint_freshness_proven'] is False
assert d['rars01']['zero_calibration_operator_verified'] is True
assert d['rars01']['zero_calibration_source'] == 'sdk_gui_operator_saved'
for section, keys in [('remote', ['takeover_hold_s', 'stale_timeout_s']),
                      ('lowstate', ['stale_timeout_s']),
                      ('rars01', ['feedback_timeout_s', 'target_timeout_s']),
                      ('sport_mode', ['observation_timeout_s'])]:
    for key in keys:
        assert 0 < d[section][key] < float('inf')
assert d['low_state_timeout_sec'] == d['lowstate']['stale_timeout_s']
assert d['arm_state_timeout_sec'] == d['rars01']['feedback_timeout_s']
for file in ['src/rars_sdk_readonly.cpp', 'src/rars_bridge.cpp']:
    code = (p/file).read_text()
    for forbidden in ['.connect(', '.enable(', '.disable(', '.setZero(', '.sendMit(',
                      '.sendConfigured(', '.sendPositionTargets(', 'SerialPort(']:
        assert forbidden not in code, (file, forbidden)
assert '#include "gamepad.hpp"' in (p/'src/safety_io.cpp').read_text()
assert 'r.btn.components.L1 && r.btn.components.L2 && r.btn.components.A' in (p/'src/safety_io.cpp').read_text()
print('verify_r2: PASS safe defaults, SDK remote, read-only RARS adapters; static check only')
