"""Attempt snapshot and approved path reuse; fake geometry, no hardware IO."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture
from mission.planning import Planning
from mission.scene import BodyPose, WorldGraspAnchor


class PlanningContracts(unittest.TestCase):
    def setup_planner(self):
        f=Fixture();f.feed();scene=copy.deepcopy(f.m.inputs['scene'])
        body=BodyPose(**scene['body'])
        sample=dict(q=list(scene['q7'][:6]),q7=list(scene['q7']),legs=list(scene['legs']),
                    legs_ns=scene['legs_ns'],feedback_ns=scene['feedback_ns'],
                    owner_session_id='arm',body=body,odom_vehicle=body.map_vehicle,
                    odom_mono_ns=body.mono_ns,scene_snapshot=scene,capture_ns=body.mono_ns)
        p=Planning.__new__(Planning);p.cfg=copy.deepcopy(f.m.cfg);p.frames=f.frames
        p.grasp={'robot':{'motion':{'pregrasp_duration_s':1.,'grasp_duration_s':1.,'retreat_duration_s':1.},
                         'rars01':{'command_rate_hz':100.,'position_velocity_limits_rad_s':[1.]*7},
                         'gripper':{'rars01':{'closed_position_rad':0.}}}}
        p.collision=NS(validated_scene=lambda *a,**k:None,
                       check=lambda *a,**k:(_ for _ in ()).throw(AssertionError('geometry rerun')))
        return p,sample

    def test_cached_path_survives_leg_motion_without_ik_or_collision(self):
        p,sample=self.setup_planner();old=p.collision_scene(sample)
        points=np.zeros((2,7));points[-1,0]=.01
        trajectory=dict(points=points.tolist(),collision_context=copy.deepcopy(old),
                        scene_snapshot=copy.deepcopy(sample['scene_snapshot']),collision_checked=True,payload_checked=False)
        world=WorldGraspAnchor(np.eye(4).tolist(),sample['body'].mono_ns,sample['body'].ros_ns,'nav','synthetic-tf-r1',1,.9)
        plan=dict(approved_segments={'pregrasp':trajectory},world_grasp_anchor=world.data())
        request=dict(operation='pregrasp',owner={},owner_session_id='arm',navigation_session_id='nav',goal_revision=1,grasp_plan=plan)
        changed=copy.deepcopy(sample);changed['legs'][0]+=.01
        changed['scene_snapshot']['legs']=tuple(changed['legs'])
        with patch('mission.planning.time.monotonic_ns',return_value=sample['body'].mono_ns):
            result=p.execute(request,lambda *a:changed)
            p.validate_result(result,lambda *a:changed,request)
        self.assertEqual(result['trajectory']['points'],points.tolist())
        self.assertTrue(result['trajectory']['reused_approval'])
        self.assertEqual(result['trajectory']['approval_collision_context'],old)
        self.assertEqual(plan['approved_segments']['pregrasp'],trajectory)
        changed['q7'][0]=changed['q'][0]=.03
        with patch('mission.planning.time.monotonic_ns',return_value=sample['body'].mono_ns):
            with self.assertRaisesRegex(ValueError,'START_CHANGED'):p.execute(request,lambda *a:changed)
        changed['q7'][0]=changed['q'][0]=0.
        changed['owner_session_id']='new-owner'
        with self.assertRaisesRegex(ValueError,'SESSION_CHANGED'):p.execute(request,lambda *a:changed)

    def test_frozen_geometry_reference_cannot_renew_live_admission(self):
        from mission.collision import Collision
        p,sample=self.setup_planner();scene=p.collision_scene(sample);original=copy.deepcopy(scene)
        collision=Collision.__new__(Collision);collision.cfg=p.cfg;collision.environment_enabled=False
        acquisition=sample['body'].mono_ns
        with patch('mission.collision.time.monotonic_ns',return_value=acquisition+1_000_000_000):
            collision.validated_scene(scene,False,reference_ns=acquisition)
            with self.assertRaisesRegex(ValueError,'STALE'):collision.validated_scene(scene,False)
        self.assertEqual(scene,original)

    def test_candidates_share_one_snapshot_and_next_attempt_acquires_another(self):
        p,sample=self.setup_planner();p.resume_camera=lambda:None;p.compute_metrics={}
        p.eye=NS(chain=lambda *a:(np.eye(4),np.eye(4)));p.geometry=object();p.ik=object()
        at=copy.deepcopy(sample);at.update(capture_mono_ns=sample['body'].mono_ns,
                    capture_ros_ns=sample['body'].ros_ns,odom_arm=np.eye(4),arm_camera=np.eye(4))
        p.detect=lambda *a:(None,None,NS(conf=.9,bbox_xyxy=[0,0,1,1]),at,np.array([.1,.2,.3]))
        p.worker=NS(infer=lambda *a,**k:dict(grasps=[1],elapsed_s=.1));p.camera=NS(K=np.eye(3))
        p.grasp['graspnet']=dict(num_point=1,min_depth=.1,max_depth=2.,collision_thresh=.01,
                                voxel_size=.01,target_margin_px=0,target_expand_ratio=1.,worker_timeout_s=1.)
        p.grasp['robot']['rars01'].update(max_grasp_width_m=.1,max_grasp_depth_m=.1)
        p.grasp['grasp_pipeline']={'grasp':dict(pregrasp_offset_m=.1,insertion_depth_m=0.,min_base_z_m=0.,
                min_jaw_z_m=0.,position_compensation_base_m=dict(x=0,y=0,z=0),ik_candidate_limit=2,cartesian_ik={},candidate_selection={})}
        p.samples=lambda start,targets,*a,**k:np.vstack([start,np.r_[targets[-1],start[6]]])
        passes=[];reads=[];acquisitions=[]
        def context(*a):
            value=copy.deepcopy(sample);value['legs'][0]+=.001*len(reads)
            value['scene_snapshot']['legs']=tuple(value['legs']);reads.append(value)
            return value
        def checked(points,scene,payload,kind='move',**options):
            passes.append((copy.deepcopy(scene),options['reference_ns']))
            return dict(points=np.asarray(points).tolist(),scene_snapshot=copy.deepcopy(scene['scene_snapshot']),
                        collision_context=p.collision_scene(scene),collision_checked=True,payload_checked=False,kind=kind)
        p.checked=checked
        p.collision.validated_scene=lambda scene,payload,**k:acquisitions.append(copy.deepcopy(scene))
        candidates=[NS(pregrasp_joints=np.zeros(6),grasp_joints=np.zeros(6),retreat_joints=np.zeros(6),
                       approach_joint_waypoints=[np.zeros(6)],retreat_joint_waypoints=[np.zeros(6)],
                       pregrasp6d=[0.]*6,grasp6d=[0.]*6,retreat6d=[0.]*6,cost=i) for i in [1.,2.]]
        selection=ModuleType('rars01_graspnet.real_grasp_selection')
        def select(*a,**kw):
            for candidate in candidates:self.assertTrue(kw['path_validator'](candidate))
            return candidates[0]
        selection._select_executable_grasp=select
        api=ModuleType('graspnetAPI');api.GraspGroup=lambda value:value
        utils=ModuleType('utils.graspnet_utils')
        request=dict(operation='grasp_plan',owner={},owner_session_id='arm',navigation_session_id='nav',goal_revision=1)
        with patch.dict(sys.modules,{'graspnetAPI':api,'utils.graspnet_utils':utils,'rars01_graspnet.real_grasp_selection':selection}),patch('mission.planning.time.monotonic_ns',return_value=sample['body'].mono_ns):
            first=p.execute(request,context);second=p.execute(request,context)
        self.assertEqual(len(passes),20)
        for begin in (0,10):
            for scene,reference in passes[begin:begin+10]:
                self.assertEqual(scene['legs'],passes[begin][0]['legs'])
                self.assertEqual(reference,sample['body'].mono_ns)
        self.assertNotEqual(passes[0][0]['legs'],passes[10][0]['legs'])
        self.assertEqual(len(acquisitions),2)
        self.assertEqual(len(reads),6) # initial, one attempt snapshot, final live admission per attempt
        self.assertEqual(set(first['grasp_plan']['approved_segments']),{'pregrasp','approach','retreat','close'})
        self.assertNotEqual(first['grasp_plan']['planning_scene_snapshot']['legs'],second['grasp_plan']['planning_scene_snapshot']['legs'])


if __name__=='__main__':unittest.main()
