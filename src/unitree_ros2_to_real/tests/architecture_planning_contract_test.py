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
    def test_stage_profiler_preserves_timeout_and_records_inclusive_work(self):
        from mission.collision import timed_collision_stage, CollisionComputeTimeout
        class Probe:
            _profiling=True
            def __init__(self):self.last_stats={}
            @timed_collision_stage
            def outer(self):return self.inner()
            @timed_collision_stage
            def inner(self):raise CollisionComputeTimeout('test timeout')
        p=Probe()
        with self.assertRaisesRegex(CollisionComputeTimeout,'test timeout'):p.outer()
        self.assertEqual(p.last_stats['exception_stage'],'inner')
        self.assertEqual(p.last_stats['inner']['calls'],1)
        self.assertEqual(p.last_stats['outer']['calls'],1)
        self.assertGreaterEqual(p.last_stats['outer']['wall_s'],p.last_stats['inner']['wall_s'])
        self.assertIsNone(p.last_stats['active_stage'])

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

    def test_scan_pose_keeps_acquisition_reference_and_live_admission(self):
        from mission.scene import validate_scene_data
        p,sample=self.setup_planner();p.compute_metrics={};p.resume_camera=lambda:None
        acquired=sample['body'].mono_ns;clock=[acquired]
        p.grasp['robot']['ready_pose']=dict(x=0,y=0,z=0,roll=0,pitch=0,duration=1.)
        p.grasp['robot']['rars01']['max_grasp_width_m']=.1
        p.geometry=NS(motor_angle_for_width=lambda width:0.)
        p.angles=[0.];p.samples=lambda start,*a,**k:np.vstack([start,start])
        def solve(*a,**k):
            clock[0]+=1_000_000_000
            return NS(success=True,joints=np.zeros(6))
        p.ik=NS(solve=solve)
        def validate(scene,payload,reference_ns=None):
            validate_scene_data(scene['scene_snapshot'],p.cfg,
                clock[0] if reference_ns is None else reference_ns,'nav','arm')
        def check(points,scene,payload,**options):
            validate(scene,payload,**options);return True
        p.collision=NS(validated_scene=validate,check=check)
        request=dict(id=1,operation='scan_pose',owner={},owner_session_id='arm',navigation_session_id='nav')
        with patch('mission.planning.time.monotonic_ns',side_effect=lambda:clock[0]):
            result=p.execute(request,lambda *a:copy.deepcopy(sample))
            self.assertEqual(p.compute_metrics['planning_reference_ns'],acquired)
            self.assertEqual(p.compute_metrics['scan_pose_ik_elapsed_s'],1.)
            self.assertGreater(p.compute_metrics['scene_body_age_at_collision_s'],p.cfg['limits']['sensor_age_s'])
            # Successful planning is not permission to admit an old live scene.
            with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:body'):
                p.validate_result(result,lambda *a:copy.deepcopy(sample),request)
            fresh=copy.deepcopy(sample)
            fresh['scene_snapshot']['body']['mono_ns']=clock[0]
            fresh['scene_snapshot']['live_feedback_ns']=clock[0]
            fresh['scene_snapshot']['live_legs_ns']=clock[0]
            fresh['scene_snapshot']['feedback_ns']=clock[0]
            fresh['scene_snapshot']['legs_ns']=clock[0]
            p.validate_result(result,lambda *a:fresh,request)
            fresh['scene_snapshot']['live_legs_ns']=acquired
            with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:live_legs_ns'):
                p.validate_result(result,lambda *a:fresh,request)
            # An already stale acquisition must fail before IK/collision work.
            with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:body'):
                p.execute(request,lambda *a:copy.deepcopy(sample))
        self.assertEqual(result['trajectory']['approval_collision_context']['legs'],sample['legs'])

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

    def test_open_jaw_collision_reference_preserves_command_and_original_result(self):
        p,sample=self.setup_planner();checked=[]
        p.collision.check=lambda points,*a,**k:checked.append(np.asarray(points).copy()) or True
        points=np.zeros((2,7));points[:,6]=.4;points[-1,0]=.01
        reference=dict(model='open_gripper_approximation',opening_width_m=.1,motor_angle_rad=.9)
        result=p.checked(points,sample,False,gripper_reference=reference)
        np.testing.assert_array_equal(checked[0][:,6],[.9,.9])
        self.assertEqual(result['points'],points.tolist())
        self.assertEqual(result['collision_points'],checked[0].tolist())
        self.assertFalse(result['physical_geometry_qualified'])
        reference['opening_width_m']=0.;self.assertEqual(result['collision_gripper_reference']['opening_width_m'],.1)

    def test_retreat_reuses_arm_path_after_close_without_rechecking_q7_or_legs(self):
        p,sample=self.setup_planner();points=np.zeros((2,7));points[:,6]=.9;points[-1,0]=.01
        trajectory=dict(points=points.tolist(),collision_context=p.collision_scene(sample),collision_checked=True,
            scene_snapshot=copy.deepcopy(sample['scene_snapshot']),payload_checked=False,
            collision_points=points.tolist(),collision_gripper_reference=dict(model='open_gripper_approximation',opening_width_m=.1,motor_angle_rad=.9))
        original=copy.deepcopy(trajectory)
        world=WorldGraspAnchor(np.eye(4).tolist(),sample['body'].mono_ns,sample['body'].ros_ns,'nav','synthetic-tf-r1',1,.9)
        request=dict(operation='retreat',owner=dict(gripper_command_active=True,gripper_hold_target=.4),
            owner_session_id='arm',navigation_session_id='nav',goal_revision=1,
            grasp_plan=dict(approved_segments={'retreat':trajectory},world_grasp_anchor=world.data()))
        sample['q7'][6]=.4;sample['scene_snapshot']['q7']=tuple(sample['q7'])
        sample['legs'][0]+=.01;sample['scene_snapshot']['legs']=tuple(sample['legs'])
        p.grasp['robot']['rars01']['max_grasp_width_m']=.1
        p.geometry=NS(opening_width=lambda angle:angle/9)
        with patch('mission.planning.time.monotonic_ns',return_value=sample['body'].mono_ns):
            result=p.execute(request,lambda *a:sample);p.validate_result(result,lambda *a:sample,request)
        actual=result['trajectory']
        np.testing.assert_array_equal(np.array(actual['points'])[:,:6],points[:,:6])
        np.testing.assert_array_equal(np.array(actual['points'])[:,6],[.4,.4])
        self.assertEqual(actual['collision_points'],original['collision_points'])
        self.assertEqual(trajectory,original);self.assertTrue(actual['reused_approval'])
        self.assertFalse(actual['physical_geometry_qualified']);self.assertEqual(actual['operation'],'retreat')
        sample['q7'][0]=sample['q'][0]=.03
        with patch('mission.planning.time.monotonic_ns',return_value=sample['body'].mono_ns):
            with self.assertRaisesRegex(ValueError,'START_CHANGED'):p.execute(request,lambda *a:sample)

    def test_physical_retreat_requires_clearance_evidence_even_if_torque_qualified(self):
        from mission.config import blockers,physical_retreat_qualified
        p,_=self.setup_planner();cfg=p.cfg;cfg['test_mode']='full'
        cfg['grasp_verification'].update(motor_torque_cap_validated=True,motor_torque_cap_evidence='synthetic fixture')
        self.assertIn('physical_retreat_clearance_unqualified',blockers(cfg))
        cfg['collision']['retreat_physical_clearance_validated']=True
        for evidence in ('','   ',False,None):
            cfg['collision']['retreat_physical_clearance_evidence']=evidence
            self.assertFalse(physical_retreat_qualified(cfg['collision']))
        cfg['collision']['retreat_physical_clearance_evidence']='synthetic qualification fixture; NOT hardware evidence'
        self.assertTrue(physical_retreat_qualified(cfg['collision']))

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
        p.eye=NS(chain=lambda *a:(np.eye(4),np.eye(4)));p.geometry=NS(motor_angle_for_width=lambda width:.9);p.ik=object()
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
            passes.append((copy.deepcopy(scene),options['reference_ns'],copy.deepcopy(options.get('gripper_reference'))))
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
        self.assertEqual(sum(reference is not None for _,_,reference in passes),8)
        self.assertTrue(all(reference['opening_width_m']==.1 for _,_,reference in passes if reference is not None))
        for begin in (0,10):
            for scene,reference,_ in passes[begin:begin+10]:
                self.assertEqual(scene['legs'],passes[begin][0]['legs'])
                self.assertEqual(reference,sample['body'].mono_ns)
        self.assertNotEqual(passes[0][0]['legs'],passes[10][0]['legs'])
        self.assertEqual(len(acquisitions),2)
        self.assertEqual(len(reads),6) # initial, one attempt snapshot, final live admission per attempt
        self.assertEqual(set(first['grasp_plan']['approved_segments']),{'pregrasp','approach','retreat','close'})
        self.assertNotEqual(first['grasp_plan']['planning_scene_snapshot']['legs'],second['grasp_plan']['planning_scene_snapshot']['legs'])


if __name__=='__main__':unittest.main()
