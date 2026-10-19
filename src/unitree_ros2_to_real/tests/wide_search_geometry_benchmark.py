#!/usr/bin/env python3
"""Offline actual planning/geometry sweep. No ROS, camera or hardware initialization."""
from pathlib import Path
import copy,sys,time,json
import numpy as np
root=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(root/'repos/workhop_rl/src/unitree_ros2_to_real/scripts'))
from mission.config import load
from mission.planning import Planning
from mission.scene import BodyPose,SceneSnapshot
cfg=load(root/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',root)
assert not cfg['physical_enabled']
cfg=copy.deepcopy(cfg)
t0=time.monotonic();p=Planning(cfg,initialize_perception=False)
print(json.dumps(dict(stage='initialized',elapsed_s=time.monotonic()-t0,angles=p.angles.tolist())),flush=True)
legs=[-.1,.8,-1.5,.1,.8,-1.5,-.1,.8,-1.5,.1,.8,-1.5];q7=np.zeros(7)
def context(*args):
 now=time.monotonic_ns();body=BodyPose(time.time_ns(),now,'bench-nav','bench-tf',np.eye(4).tolist(),p.frames.body(np.eye(4)).tolist())
 scene=SceneSnapshot(body,tuple(q7),tuple(legs),now,now,'bench-owner')
 return dict(capture_ns=now,capture_ros_ns=time.time_ns(),feedback_ns=now,q=q7[:6].tolist(),q7=q7.tolist(),legs=legs,legs_ns=now,
   owner_session_id='bench-owner',odom_mono_ns=now,odom_vehicle=body.map_vehicle,body=body,scene_snapshot=scene.data())
rows=[]
operations=[('scan_pose',0)]+[('scan_step',i) for i in range(1,len(p.angles))]+[('scan_return',0),('home',0)]
for op,index in operations:
 request=dict(operation=op,scan_index=index,owner={'gripper_hold_confirmed':False},owner_session_id='bench-owner',navigation_session_id='bench-nav',goal_revision=1)
 start=time.monotonic()
 try:
  result=p.execute(request,context);points=np.asarray(result['trajectory']['points']);q7=points[-1].copy()
  row=dict(operation=op,index=index,target_rad=float(p.angles[index]),ok=True,points=len(points),duration_s=(len(points)-1)/100,
   peak_joint1_velocity_rad_s=float(np.max(np.abs(np.diff(points[:,0])))*100),elapsed_s=time.monotonic()-start,stats=p.collision.last_stats.copy())
 except Exception as error:
  row=dict(operation=op,index=index,target_rad=float(p.angles[index]),ok=False,error=str(error),elapsed_s=time.monotonic()-start,stats=p.collision.last_stats.copy(),failure=p.collision.last_failure.copy())
 rows.append(row);print(json.dumps(row),flush=True)
 if not row['ok']:break
Path('/tmp/wide_search_geometry_20261018.json').write_text(json.dumps(dict(rows=rows,all_pass=len(rows)==len(operations) and all(r['ok'] for r in rows),physical_commands=0),indent=2))
