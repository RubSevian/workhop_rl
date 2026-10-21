#!/usr/bin/env python3
"""Offline candidate jaw envelopes. No ROS, SDK, motor or planning admission.

For a direct prismatic child of End_link, each mesh point moves affinely.
The convex hull of its two endpoint meshes therefore contains the full sweep.
An AABB of these endpoints is a second, more conservative enclosure. This
analysis deliberately uses signed Coal primitive/convex queries and does not
turn a negative result into an allowed STL zero contact.
"""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def sweep_points(vertices, lower_pose, upper_pose):
    vertices=np.asarray(vertices,dtype=float)
    from mission.transforms import rigid
    lower_pose,upper_pose=rigid(lower_pose),rigid(upper_pose)
    if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError('SWEEP_INVALID_VERTICES')
    if not np.allclose(lower_pose[:3,:3],upper_pose[:3,:3],atol=1e-12,rtol=0):
        raise ValueError('SWEEP_NOT_TRANSLATIONAL')
    return np.vstack([vertices@pose[:3,:3].T+pose[:3,3] for pose in (lower_pose,upper_pose)])


def enclosing_box(points):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('SWEEP_INVALID_POINTS')
    low=np.nextafter(points.min(axis=0),-np.inf)
    high=np.nextafter(points.max(axis=0),np.inf)
    center=(low+high)/2
    half=np.nextafter(np.maximum(center-low,high-center),np.inf)
    if not np.isfinite([center,half]).all() or np.any(half<=0) or np.any(np.abs(points-center)>half):
        raise ValueError('SWEEP_ENCLOSURE_UNPROVEN')
    return center,half


class SweepAnalysis:
    """Shapes are constructed once per analysis, then moved only by End_link."""
    def __init__(self,cfg):
        import coal
        import pinocchio as pin
        self.coal,self.pin,self.cfg=coal,pin,cfg
        self.model=pin.buildModelFromUrdf(cfg['paths']['full_urdf'])
        self.geom=pin.buildGeomFromUrdf(self.model,cfg['paths']['full_urdf'],pin.GeometryType.COLLISION,
                    package_dirs=[str(Path(cfg['paths']['description_root']).parent)])
        self.data,self.gdata=self.model.createData(),self.geom.createData()
        self.links=[self.model.frames[g.parentFrame].name for g in self.geom.geometryObjects]
        self.distance_request=coal.DistanceRequest();self.collision_request=coal.CollisionRequest()
        root=ET.parse(cfg['paths']['full_urdf']).getroot()
        self.base_q=pin.neutral(self.model);self.volumes=[];self.metadata=[]
        for side in ('left','right'):
            name='gripper_'+side;joint=root.find("joint[@name='%s_joint']"%name)
            if (joint is None or joint.get('type')!='prismatic' or joint.find('mimic') is not None
                    or joint.find('parent').get('link')!='End_link' or joint.find('child').get('link')!=name):
                raise ValueError('SWEEP_JOINT_CHAIN_UNSUPPORTED:'+name)
            jid=self.model.getJointId(name+'_joint')
            if jid>=self.model.njoints or self.model.nqs[jid]!=1:raise ValueError('SWEEP_JOINT_MAPPING:'+name)
            k=self.model.idx_qs[jid]
            low,high=self.model.lowerPositionLimit[k],self.model.upperPositionLimit[k]
            if not np.isfinite([low,high]).all() or not low<high:raise ValueError('SWEEP_JOINT_RANGE:'+name)
            endpoints=[]
            for value in (low,high):
                q=self.base_q.copy();q[k]=value;poses,end=self.placements(q)
                endpoints.append([np.linalg.inv(end)@pose for pose in poses])
            indices=[i for i,link in enumerate(self.links) if link==name]
            if not indices:raise ValueError('SWEEP_JAW_GEOMETRY_MISSING:'+name)
            for i in indices:
                obj=self.geom.geometryObjects[i]
                if not hasattr(obj.geometry,'vertices'):raise ValueError('SWEEP_NONMESH_JAW:'+name)
                points=sweep_points(obj.geometry.vertices(),endpoints[0][i],endpoints[1][i])
                center,half=enclosing_box(points);local=np.eye(4);local[:3,3]=center
                self.volumes.append((name,'box',coal.Box(*(2*half)),local))
                vector=coal.StdVec_Vec3s()
                for point in points:vector.append(point)
                hull=coal.Convex.convexHull(vector,True,'Qt')
                self.volumes.append((name,'convex',hull,np.eye(4)))
                self.metadata.append(dict(jaw=name,geometry=obj.name,range_m=[float(low),float(high)],
                    lower_End_link_m=(center-half).tolist(),upper_End_link_m=(center+half).tolist(),
                    endpoint_vertex_count=len(points),mesh_sha256=hashlib.sha256(Path(obj.meshPath).read_bytes()).hexdigest()))

    def placements(self,q):
        p=self.pin;p.forwardKinematics(self.model,self.data,q);p.updateFramePlacements(self.model,self.data)
        p.updateGeometryPlacements(self.model,self.data,self.geom,self.gdata,q)
        return [t.homogeneous.copy() for t in self.gdata.oMg],self.data.oMf[self.model.getFrameId('End_link')].homogeneous.copy()

    def query(self,shape,pose,other,other_pose):
        c=self.coal;a,b=(c.Transform3s(t[:3,:3],t[:3,3]) for t in (pose,other_pose))
        result=c.DistanceResult();distance=float(c.distance(shape,a,other,b,self.distance_request,result))
        contact=c.CollisionResult();hit=bool(c.collide(shape,a,other,b,self.collision_request,contact) or contact.isCollision())
        if not np.isfinite(distance):raise ValueError('SWEEP_QUERY_NONFINITE')
        return distance,hit

    def attachment_conflicts(self,arm):
        q=self.base_q.copy()
        for i,value in enumerate(arm,1):q[self.model.idx_qs[self.model.getJointId('joint'+str(i))]]=value
        poses,end=self.placements(q);rows=[]
        for name,kind,shape,local in self.volumes:
            for i,link in enumerate(self.links):
                if link!='End_link':continue
                distance,hit=self.query(shape,end@local,self.geom.geometryObjects[i].geometry,poses[i])
                rows.append(dict(jaw=name,volume=kind,other=link,distance_m=distance,collision_reported=hit,
                                 blocked=distance<0 or hit))
        return rows


def analyze(cfg):
    analysis=SweepAnalysis(cfg)
    cases={'HOME':[0.]*6,'READY':[-.0011,.972,1.3737,-1.1017,-.0009,-.0007]}
    rows=[dict(state=state,**row) for state,arm in cases.items() for row in analysis.attachment_conflicts(arm)]
    return dict(status='BLOCKED' if any(r['blocked'] for r in rows) else 'UNQUALIFIED',
        backend_version=str(analysis.coal.__version__),solver_tolerances=dict(
            distance_gjk=analysis.distance_request.gjk_tolerance,distance_epa=analysis.distance_request.epa_tolerance,
            collision_gjk=analysis.collision_request.gjk_tolerance,collision_epa=analysis.collision_request.epa_tolerance),urdf_sha256=hashlib.sha256(Path(cfg['paths']['full_urdf']).read_bytes()).hexdigest(),
        candidates=analysis.metadata,attachment_checks=rows,cache='volume objects constructed once per analysis instance',
        coverage='Analytical prismatic translation; convex endpoint hull and outward-rounded endpoint AABB cover full URDF stroke.',
        trajectory_admission=False,full_approach_retreat_checked=False,physical_commands=0,
        note='An intrinsic attachment conflict is independent of arm pose. No activation, new collision exclusions, penetration tolerance or kinematics changes.')


if __name__=='__main__':
    from mission.config import load
    cfg=load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)
    try:result=analyze(cfg)
    except (ValueError,RuntimeError) as error:result=dict(status='BLOCKED',error=str(error),trajectory_admission=False,physical_commands=0)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
