#!/usr/bin/env python3
from pathlib import Path
import mujoco,numpy as np
p=Path(__file__).resolve().parents[1]/'unitree_robots/go2_rars01/scene.xml';m=mujoco.MjModel.from_xml_path(str(p));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0)
legs=('FR_hip_joint','FR_thigh_joint','FR_calf_joint','FL_hip_joint','FL_thigh_joint','FL_calf_joint','RR_hip_joint','RR_thigh_joint','RR_calf_joint','RL_hip_joint','RL_thigh_joint','RL_calf_joint'); targets=np.array([-.1,.8,-1.5,.1,.8,-1.5,-.1,.8,-1.5,.1,.8,-1.5])
arm=tuple('joint%d'%i for i in range(1,7))+('gripper_left_joint','gripper_right_joint');kp=np.array([20,20,20,6,6,6,20,20]);kd=np.array([1,1,1,.4,.4,.4,.2,.2]);lim=np.array([27,27,27,7,7,7,3,3]); maxarm=maxvel=maxtau=0.;z=[]
for _ in range(800):
 for i,n in enumerate(legs):
  j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n);q=d.qpos[m.jnt_qposadr[j]];v=d.qvel[m.jnt_dofadr[j]];d.ctrl[i]=np.clip(25*(targets[i]-q)-v,-35.55 if 'calf' in n else -23.7,35.55 if 'calf' in n else 23.7)
 for i,n in enumerate(arm):
  j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n);q=d.qpos[m.jnt_qposadr[j]];v=d.qvel[m.jnt_dofadr[j]];d.ctrl[12+i]=np.clip(-kp[i]*q-kd[i]*v,-lim[i],lim[i])
 mujoco.mj_step(m,d); z.append(d.qpos[2]); maxarm=max(maxarm,max(abs(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)]]) for n in arm));maxvel=max(maxvel,np.max(abs(d.qvel)));maxtau=max(maxtau,np.max(abs(d.ctrl[12:20])))
assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and maxarm<0.1
print('HOME HOLD: PASS arm_error=%.5f max_qvel=%.3f max_arm_tau=%.3f base_z=[%.3f,%.3f]'%(maxarm,maxvel,maxtau,min(z),max(z)))
