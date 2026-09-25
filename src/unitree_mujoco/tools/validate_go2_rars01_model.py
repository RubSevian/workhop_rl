#!/usr/bin/env python3
import argparse, math
from pathlib import Path
import mujoco, numpy as np
LEG=('FR_hip_joint','FR_thigh_joint','FR_calf_joint','FL_hip_joint','FL_thigh_joint','FL_calf_joint','RR_hip_joint','RR_thigh_joint','RR_calf_joint','RL_hip_joint','RL_thigh_joint','RL_calf_joint')
ARM=tuple('joint%d'%i for i in range(1,7)); GRIP=('gripper_left_joint','gripper_right_joint')
def main():
 p=Path(__file__).resolve().parents[1]/'unitree_robots/go2_rars01/scene.xml'; m=mujoco.MjModel.from_xml_path(str(p)); d=mujoco.MjData(m)
 assert m.nu==20 and abs(m.opt.timestep-.005)<1e-12
 for n,t in [(n,mujoco.mjtJoint.mjJNT_HINGE) for n in LEG+ARM]+[(n,mujoco.mjtJoint.mjJNT_SLIDE) for n in GRIP]:
  i=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n); assert i>=0 and m.jnt_type[i]==t,n
 for i,n in enumerate(LEG+ARM+GRIP):
  aid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,n.replace('_joint','_motor')); assert aid==i,(n,aid,i)
 mujoco.mj_resetDataKeyframe(m,d,0); mujoco.mj_forward(m,d)
 for _ in range(20): mujoco.mj_step(m,d)
 assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and np.isfinite(d.sensordata).all()
 print('MODEL VALIDATION: PASS nq=%d nv=%d nu=%d timestep=%.3f'%(m.nq,m.nv,m.nu,m.opt.timestep))
if __name__=='__main__': main()
