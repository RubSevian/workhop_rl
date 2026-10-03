"""Compare robot dynamics with the pre-MVP MJCF; no policy/controller changes."""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np

repo=Path(__file__).resolve().parents[4]
relative='src/unitree_mujoco/unitree_robots/go2_rars01/go2_rars01.xml'
path=repo/relative
baseline_ref='cd5bd7c26d5b37284689b7475768087e890d296c'
baseline=subprocess.check_output(['git','-C',str(repo),'show',baseline_ref+':'+relative], text=True)
def load(text):
    root=ET.fromstring(text)
    root.find('compiler').set('meshdir', str(path.parent))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
def run(text):
    model=load(text); data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0)
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'virtual_payload')
    if body>=0: model.body_gravcomp[body]=1
    trajectory=[]
    for step in range(1000):
        mujoco.mj_step(model,data)
        trajectory.append(np.r_[data.qpos[:27],data.qvel[:26]])
    return np.array(trajectory), (model.nu,model.nsensor)
original, abi_original=run(baseline)
updated, abi_updated=run(path.read_text())
max_difference=float(np.max(np.abs(original-updated)))
assert max_difference < 1e-9, max_difference
assert abi_original==abi_updated and abi_original[0]==20
print('PASS disabled/pre-attach regression: 1000 steps, max robot qpos/qvel difference',max_difference)
