"""Check native owner linkage math against the existing Python geometry."""
from pathlib import Path
import subprocess,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'repos/rars01_graspnet'))
from rars01_graspnet.config import load_config
from rars01_graspnet.gripper_geometry import RarsGripperGeometry
cfg=load_config(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml')
g=cfg['robot']['gripper']['rars01']
keys=('linkage_radius_m','connecting_rod_length_m','carriage_width_m','jaw_center_End_link_m','jaw_depth_m','jaw_height_m','R_grasp_End_link')
geometry=RarsGripperGeometry(**{key:g[key] for key in keys},maximum_width_m=cfg['robot']['rars01']['max_grasp_width_m'],motor_angle_limit_rad=abs(g['angle_open']))
rows=np.array([[float(v) for v in line.split()] for line in subprocess.check_output([sys.argv[1],'--width-grid'],text=True).splitlines()])
error=max(abs(geometry.opening_width(q)-width) for q,width in rows)
assert error<1e-12,(len(rows),error)
print(f'PASS native/Python measured-angle linkage: {len(rows)} samples, max error {error:.3g}m')

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.gripper_measurement import closed_zero
rows=np.array([[float(v) for v in line.split()] for line in subprocess.check_output([sys.argv[1],'--closed-zero-grid'],text=True).splitlines()])
tolerance=cfg['go2_mission']['collision']['gripper_closed_measurement_tolerance_rad']
assert tolerance==.025
error=max(abs(geometry.opening_width(closed_zero(q,tolerance))-width) for q,width in rows)
assert error<1e-12,(len(rows),error)
print(f'PASS symmetric closed-zero Python/C++ parity: {len(rows)} samples')
