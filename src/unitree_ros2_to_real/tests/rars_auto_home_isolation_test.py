"""Source/deployment invariants; no serial, ROS runtime or SDK calls."""
from pathlib import Path
import yaml
p=Path(__file__).resolve().parents[1]
owner=(p/'src/rars_r3_owner.cpp').read_text()
core=(p/'src/rars_auto_home.cpp').read_text()
leg=(p/'src/go2_r3_commissioning.cpp').read_text()
assert 'setZero(' not in owner+core+leg
assert '"connect_serial",false' in owner and '"read_only",true' in owner
assert 'hold_request' not in leg and 'sendPositionTargets' not in leg
assert 'sdk_->enable' not in leg and 'arm_home_ready' in leg
assert 'config.motors[i].direction=directions[i]' in owner
assert 'config.motors[i].zero_offset=z[i]' in owner
assert 'sendPositionTargets(q)' in owner
assert 'configuration_' not in core
cfg=yaml.safe_load((p/'config/go2_rars01_real.yaml').read_text())['real_deployment']['rars01']
assert cfg['require_home_ready_for_leg_takeover'] is True
assert cfg['auto_home']['home_target']==[0]*7
assert cfg['auto_home']['startup_delay_s']==10
assert cfg['auto_home']['enable_once_on_boot'] is True
unit=(p/'deployment/rars01-owner.service').read_text()
assert 'Restart=no' in unit and 'StateDirectory=rars01-owner' in unit
print('PASS AUTO HOME source/config/service isolation')
