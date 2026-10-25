#!/usr/bin/env python3
"""Read-only, cached URDF stick model and planning snapshot renderer."""
from functools import lru_cache
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


@lru_cache(maxsize=4)
def arm_chain(path):
    joints = ET.parse(path).getroot().findall('joint')
    parents = {j.find('child').get('link'): j for j in joints}
    chain = []; link = 'End_link'
    while link != 'base_link':
        j = parents[link]; origin = j.find('origin'); axis = j.find('axis')
        xyz = np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
        rpy = np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
        vector = np.fromstring(axis.get('xyz','1 0 0') if axis is not None else '1 0 0',sep=' ')
        chain.append((j.get('name'),j.get('type'),xyz,rpy,vector))
        link = j.find('parent').get('link')
        if len(chain)>100:raise ValueError('Invalid URDF chain')
    return tuple(reversed(chain))


def arm_fk(path,q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('Invalid arm joints')
    angles={f'joint{i+1}':q[i] for i in range(6)}
    t=np.eye(4);points=[np.zeros(3)]
    for name,kind,xyz,rpy,axis in arm_chain(str(path)):
        origin=np.eye(4);origin[:3,3]=xyz;origin[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix()
        t=t@origin
        if kind!='fixed':
            points.append(t[:3,3].copy())
            if kind not in ('revolute','continuous'):raise ValueError('Unsupported arm joint')
            turn=np.eye(4);turn[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*angles[name]).as_matrix()
            t=t@turn
    points.append(t[:3,3].copy())
    return np.array(points),t


def pose_matrix(pose):
    t=np.eye(4);t[:3,3]=pose[:3];t[:3,:3]=Rotation.from_euler('xyz',pose[3:]).as_matrix()
    return t


def load_arm_snapshot(path):
    data=json.loads(path.read_text())
    if data.get('frame')!='base_link':raise ValueError('Unsupported display frame')
    return data


def draw_arm_snapshot(ax,data,candidate_name):
    view=(ax.elev,ax.azim);ax.clear();extent=[]
    def arm(q,color,label,style='-'):
        pts,t=arm_fk(data['urdf'],q);extent.extend(pts)
        ax.plot(*pts.T,linestyle=style,marker='o',ms=3,lw=2,color=color,label=label)
        return t
    arm(data['current_q'],'gray','Measured at planning start')
    ax.scatter(0,0,0,c='black',s=55,label='Arm base / robot marker')
    candidate=next((c for c in data['candidates'] if c['candidate']==candidate_name),None)
    if candidate is not None:
        pre=pose_matrix(candidate['pregrasp_End_link_pose6d'])
        target=pose_matrix(candidate['grasp_End_link_pose6d'])
        camera=np.asarray(data['camera_to_arm'])
        obj=(camera@np.r_[candidate['translation_camera_m'],1])[:3]
        ax.scatter(*obj,c='orange',s=80,label='Object: GraspNet point (shape unknown)')
        extent.extend([pre[:3,3],target[:3,3],obj])
        ax.plot(*np.array([pre[:3,3],target[:3,3]]).T,'k--',label=f'Approach {np.linalg.norm(pre[:3,3]-target[:3,3])*100:.1f} cm')
        ax.scatter(*target[:3,3],c='red',marker='x',s=50,label='Requested TCP')
        ax.quiver(*target[:3,3],*(target[:3,0]*.06),color='red',label='Requested local +X')
        # Symbolic jaws are drawn in the recorded GraspNet convention.
        rotation=camera[:3,:3]@np.asarray(candidate['rotation_camera'])
        width=candidate['width_m'];depth=candidate['depth_m']
        jaws=np.array([[depth,width/2,0],[-.04,width/2,0],[-.04,-width/2,0],[depth,-width/2,0]])@rotation.T+obj
        ax.plot(*jaws.T,color='red',lw=2,label='Requested gripper (schematic)')
        calls=[c for c in data['ik'] if c.get('candidate')==candidate_name]
        for stage,color in [('pregrasp','royalblue'),('approach','seagreen'),('retreat','purple')]:
            valid=[c for c in calls if c.get('stage')==stage and c.get('success')]
            if valid:arm(valid[-1]['joints'],color,stage+' IK (not execution)')
        failed=[c for c in calls if not c.get('success') and 'joints' in c]
        if failed and np.isfinite(failed[-1]['joints']).all():
            arm(failed[-1]['joints'],'crimson','FAILED IK — not executable',':')
        path=[arm_fk(data['urdf'],c['joints'])[1][:3,3] for c in calls
              if c.get('stage')=='approach' and c.get('success')]
        if path:ax.plot(*np.array(path).T,color='seagreen',lw=1,label='Accepted approach TCP samples')
    pts=np.array(extent);center=(pts.min(0)+pts.max(0))/2;r=max(np.ptp(pts,axis=0).max()*.6,.15)
    for setter,c in zip((ax.set_xlim,ax.set_ylim,ax.set_zlim),center):setter(c-r,c+r)
    ax.set_box_aspect((1,1,1));ax.view_init(*view)
    ax.set(xlabel='base_link X (m)',ylabel='Y (m)',zlabel='Z (m)')
    selected=data.get('selected')==candidate_name and candidate is not None
    ax.set_title(f'{candidate_name or "Waiting for candidates"}\n'+
                 ('SELECTED; configured collision checks passed' if selected else 'Diagnostic only; candidate not approved'))
    ax.legend(fontsize=7,loc='upper left')
