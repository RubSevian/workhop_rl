"""Offline config/isolation checks; does not authorize physical gates."""
from pathlib import Path
import subprocess
import yaml
import math
root=Path(__file__).resolve().parents[1]
subprocess.run(['python3',str(root/'scripts/verify_r2.py')],check=True)
p=root/'repos/workhop_rl/src/unitree_ros2_to_real'
y=yaml.safe_load((p/'config/go2_rars01_real.yaml').read_text())
r=y['real_deployment']['r3_commissioning']
assert r['remote']['controlled_abort_chord']==['L1','L2','X']
assert r['remote']['emergency_chord']==['L1','L2','B']
assert r['manual']['max_vx']<=.20
assert r['manual']['max_duration_s']<=1
if r['emergency']['operator_validated']:
    assert r['emergency']['evidence']
    assert len(r['emergency']['motor_kd'])==12
    assert all(math.isfinite(x) and x>0 for x in r['emergency']['motor_kd'])
if r['lie_down']['operator_validated']:
    assert len(r['lie_down']['motor_q'])==12
    assert all(math.isfinite(x) and abs(x)<=3.5 for x in r['lie_down']['motor_q'])
launch=(p/'launch/go2_rars01_r3_commissioning.launch.py').read_text()
assert "default_value='true'" in launch
assert "enable_actuator_output=False" in launch
assert "Unknown operation_profile" in launch and "OpaqueFunction" in launch
monitor=(p/'scripts/r3_sport_monitor.py').read_text()
assert "'--status'" in monitor
assert '--release-sport-mode' not in monitor and '--enable-sport-mode' not in monitor
owner=(p/'src/rars_r3_owner.cpp').read_text()
assert 'setZero(' not in owner
assert '"connect_serial",false' in owner and '"read_only",true' in owner
print('verify_r3: PASS startup/chords/bounds/isolation; operator confirmations are not inferred')
