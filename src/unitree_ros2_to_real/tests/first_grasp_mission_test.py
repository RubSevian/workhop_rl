#!/usr/bin/env python3
"""Synthetic integration: no camera, SDK, ROS goal or physical qualification."""
import copy
import json
from pathlib import Path
import sys
import unittest
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'repos/rars01_graspnet'))
NAV=ROOT/'repos/autonomy_nav_go2/src/utilities/go2_navigation_bridge/scripts'
sys.path.insert(0,str(NAV))
from mission.config import load,blockers,search_angles
from mission.core import Mission,TERMINAL
from mission.transforms import EyeInHand,rigid,pose_matrix,fixed_joint,xyzrpy
from mission_health import MissionHealth
from mission.scene import FrameChain, BodyPose, SceneSnapshot, BodyPoseProvider, WorldGraspAnchor, matrix_pose
from mission.scene import SceneBuffer
from scipy.spatial.transform import Rotation

CFG=load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)

class Fixture:
    def __init__(self, branch='success'):
        self.now=100.;cfg=copy.deepcopy(CFG);cfg['test_mode']='full'
        cfg['frames']['vehicle_to_mount']=copy.deepcopy(cfg['frames']['vehicle_to_mount_candidate'])
        self.m=Mission(cfg,'synthetic-coordinator');self.frames=FrameChain(cfg)
        self.branch=branch;self.effects=[];self.commands=0
        self.r3=dict(state='ACTIVE',phase='RL_ZERO',operation_profile='full_mission',capabilities={'arm_motion':True},fault_latched=False,controller_session_id='r3',navigation_session_id='nav',motion_command=[0,0,0])
        self.nav=dict(session_id='nav',navigation_ready=True,goal=dict(revision=0,stamp_ns=0,state='IDLE'))
        self.owner=dict(owner_session_id='arm',mission_capable=True,arm_emergency_validated=False,q=[0.]*7,feedback_ready=True,target_valid=True,arm_home_ready=True,gripper_hold_confirmed=False,gripper_command_active=False,grasp_verification="stub",mission_id=0,mission_state='IDLE')
        self.perception=dict(ready=True)
        self.legs=dict(q=[0,.8,-1.5]*4)
        self.base=dict(pose=[0,0,0,0,0,0,1],tilt_rad=0.)
        self.tf_revision='synthetic-tf-r1'
    def feed(self):
        self.now+=.02
        stamp=int(self.now*1e9);self.owner['frame_received_ns']=stamp
        for name in ('r3','nav','owner','perception','base','legs'):
            self.m.observe(name,dict(getattr(self,name),observed_ns=stamp),self.now)
        body_matrix=pose_matrix(self.base['pose'])
        vehicle=body_matrix@self.frames.body_mount@np.linalg.inv(xyzrpy(self.m.cfg['frames']['vehicle_to_mount']))
        body=BodyPose(stamp,stamp,self.nav['session_id'],self.tf_revision,vehicle.tolist(),body_matrix.tolist())
        scene=SceneSnapshot(body,tuple(self.owner['q']),tuple(self.legs['q']),stamp,stamp,self.owner['owner_session_id'])
        self.m.observe('scene',dict(scene.data(),observed_ns=stamp),self.now)
    def tick(self,process=True):
        self.feed();effects=self.m.step(self.now,int(self.now*1e9));self.effects+=effects
        if process:
            for kind,data in effects:self.effect(kind,data)
        return effects
    def effect(self,kind,data):
        if kind=='compute':self.m.result(self.compute(data),self.now)
        elif kind=='trajectory':
            self.commands+=1;self.owner['q']=data['points'][-1].copy()
            # Synthetic owner outcome only; no physical torque qualification.
            if data['kind']=='close':self.owner.update(q=self.owner['q'][:6]+[.4],gripper_command_active=True,gripper_hold_target=.4,grasp_assessment='NOT_GRASPED' if self.branch=='not_grasped' else 'GRASPED')
            if data['kind']=='release':self.owner.update(gripper_command_active=False,grasp_assessment='NOT_GRASPED')
            self.owner.update(mission_id=data['id'],mission_state='SUCCEEDED',arm_home_ready=all(abs(x)<1e-6 for x in self.owner['q'][:6]))
        elif kind=='cancel':self.owner['mission_state']='CANCELLED'
    def compute(self,data):
        out={k:data[k] for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')}
        out.update(ok=True,observed_ns=int(self.now*1e9))
        op=data['operation']
        if op=='reverse_search':
            out['trajectory']=copy.deepcopy(data['reverse_segment'])
            out['trajectory']['points']=out['trajectory']['points'][::-1]
            return out
        if op=='detect':out['found']=self.branch!='not_found'
        elif op=='grasp_plan':
            if self.branch=='ik_fail':out.update(ok=False,reason='IK_FAIL')
            else:out['grasp_plan']={'fixture':'synthetic-IK','scan_q':data['owner']['q'][:6]}
        else:
            start=data['owner']['q'];end=start.copy()
            if op=='retry_home_open':end[6]=.8
            elif op=='home':end[:6]=[0.]*6
            elif op=='scan_pose':end[:6]=[0.,.2,.2,.1,0,0];end[6]=.8
            elif op=='refine':end[1]+=.01;end[2]+=.01
            elif op in ('scan_step','scan_return'):end[0]=0. if op=='scan_return' else search_angles(CFG['search'])[data['scan_index']]
            elif op=='retreat':end[:6]=data['grasp_plan']['scan_q']
            elif op!='close':end[1]+=.01
            sys.path.insert(0,str(ROOT/'repos/rars01_graspnet'))
            from rars01_graspnet.real_grasp_trajectory import joint_motion_samples
            points,_=joint_motion_samples(start,end,.1,.01,np.ones(7))
            out['trajectory']=dict(kind='release' if op=='retry_home_open' else 'home' if op=='home' else 'close' if op=='close' else 'move',points=[q.tolist() for q in points],collision_checked=True,payload_checked=False)
        return out
    def start(self):
        self.tick();assert self.m.state=='WAIT_GOAL'
        self.nav['goal']=dict(revision=1,stamp_ns=int((self.now+.01)*1e9),state='REQUESTED')
        self.tick();assert self.m.state=='NAVIGATING'
        self.nav['goal']['state']='SUCCEEDED';self.tick()
    def run(self):
        self.start()
        for _ in range(4000):
            self.tick()
            if self.m.state in TERMINAL:return self.m.state
        raise AssertionError(self.m.status(self.now))

class Integration(unittest.TestCase):
    def test_disabled_deadlines_allow_long_healthy_wait_but_cancel_still_works(self):
        for phase in ('NAVIGATING','ARM_SEARCH','PLAN_GRASP'):
            with self.subTest(phase=phase):
                f,request=self.observation_fixture()
                f.m.transition(phase,f.now)
                f.m.pending=dict(id=999,operation='grasp_plan',kind='compute',at=f.now)
                f.nav['goal']['state']='ACTIVE'
                f.now+=10000.;f.tick(False)
                self.assertEqual(f.m.state,phase)
                f.m.cancel(f.now,'OPERATOR_CANCEL')
                self.assertEqual(f.m.state,'CANCELLED')
                self.assertIsNone(f.m.pending)

    def test_home_requires_confirmed_return_to_saved_scan_pose(self):
        for error in (0.,.1):
            f=Fixture();f.feed();f.m.close_outcome='GRASP_PRESUMED_SUCCESS'
            f.m.grasp_plan={'scan_q':[error,0,0,0,0,0]}
            f.m.done_motion('retreat',f.now)
            if error:
                self.assertEqual(f.m.state,'BLOCKED');self.assertEqual(f.m.reason,'SCAN_RETURN_UNCONFIRMED')
            else:
                self.assertEqual(f.m.state,'RETURN_HOME')
                self.assertTrue(f.m.effects[-1][1]['home_from_scan'])

    def test_failed_close_continues_retreat_home_without_claiming_grasp(self):
        for trial in (False,True):
            f=Fixture();f.m.cfg['operator_trial']=trial;f.m.cfg['execution']['automatic_retreat']=False
            f.owner['grasp_assessment']='NOT_GRASPED';f.feed()
            f.m.done_motion('close',f.now)
            self.assertEqual(f.m.reason,'TARGET_NOT_GRASPED_CONTINUING')
            self.assertEqual(f.m.state,'WAIT_RETREAT_APPROVAL' if trial else 'RETREAT')
            self.assertEqual(f.m.pending['operation'],'retreat')
            f.m.grasp_plan={'scan_q':f.owner['q'][:6]}
            f.m.done_motion('retreat',f.now)
            self.assertEqual(f.m.pending['operation'],'home')
            f.m.done_motion('home',f.now)
            self.assertEqual(f.m.state,'ARM_SCAN_POSE')
            self.assertEqual(f.m.full_attempt,2)
            self.assertEqual(f.m.attempt_history[0]['result'],'TARGET_NOT_GRASPED')
        f=Fixture();f.owner.update(grasp_assessment='UNKNOWN',grasp_assessment_reason='FEEDBACK_INVALID_OR_STALE');f.feed()
        f.m.done_motion('close',f.now)
        self.assertEqual(f.m.state,'BLOCKED')

    def observation_fixture(self):
        f=Fixture();f.m.cfg["refinement"]["enabled"]=False;f.start()
        for _ in range(400):
            effects=f.tick(False)
            for kind,data in effects:
                if kind=='compute' and data['operation']=='detect':return f,data
                f.effect(kind,data)
        self.fail('No detect request')

    def test_home_release_uses_stationary_arm_speed_limits_and_collision_check(self):
        from mission.planning import Planning
        from types import SimpleNamespace
        from unittest.mock import Mock
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
        planner.grasp={'robot':{'rars01':{'position_velocity_limits_rad_s':[1.]*6+[.1],
            'command_rate_hz':100,'max_grasp_width_m':.08},'ready_pose':{'duration':1.}}}
        planner.geometry=SimpleNamespace(motor_angle_for_width=lambda width:.8)
        planner.collision_scene=lambda sample: {'fixture':'scene'}
        planner.collision=SimpleNamespace(check=Mock(return_value=True),last_reason='BLOCKED',last_failure='BLOCKED')
        owner=dict(arm_home_ready=True,gripper_command_active=True,grasp_assessment='NOT_GRASPED',gripper_hold_target=0.)
        sample=dict(q7=[0.]*7,scene_snapshot={'fixture':'fresh'})
        trajectory=planner.retry_home_open(sample,owner)['trajectory'];points=np.array(trajectory['points'])
        np.testing.assert_allclose(points[:,:6],0.,atol=1e-12)
        self.assertAlmostEqual(points[-1,6],.8)
        self.assertLessEqual(np.max(np.abs(np.diff(points[:,6])))*100,.1+1e-6)
        self.assertEqual(trajectory['kind'],'release');self.assertTrue(trajectory['collision_checked'])
        self.assertEqual(trajectory['payload_checked'],bool(CFG['collision'].get('payload_enabled',True)))
        planner.collision.check.assert_called_once()
        for assessment in ('UNKNOWN','GRASPED'):
            with self.assertRaisesRegex(ValueError,'HOLD_UNKNOWN'):
                planner.retry_home_open(sample,owner|dict(grasp_assessment=assessment))
        with self.assertRaisesRegex(ValueError,'HOME_UNCONFIRMED'):
            planner.retry_home_open(sample|dict(q7=[.1]+[0.]*6),owner)
        planner.collision.check.return_value=False
        with self.assertRaisesRegex(ValueError,'BLOCKED'):planner.retry_home_open(sample,owner)

    def test_loss_during_retreat_or_home_retries_after_confirmed_home(self):
        for phase in ('RETREAT','RETURN_HOME'):
            with self.subTest(phase=phase):
                f=Fixture();f.start();lost=False
                for _ in range(1800):
                    if f.m.full_attempt==1 and f.m.state==phase:
                        f.owner.update(grasp_assessment='NOT_GRASPED',q=f.owner['q'][:6]+[0.],gripper_hold_target=0.)
                        lost=True
                    f.tick()
                    if f.m.state in TERMINAL:break
                self.assertTrue(lost);self.assertEqual(f.m.state,'GRASP_PRESUMED_SUCCESS')
                self.assertEqual(f.m.full_attempt,2)
                first,second=f.m.attempt_history
                self.assertEqual(first['close_outcome'],'GRASP_PRESUMED_SUCCESS')
                self.assertEqual(first['home_grasp_assessment'],'NOT_GRASPED')
                self.assertEqual(first['result'],'TARGET_NOT_GRASPED')
                self.assertEqual(first['reason'],'OBJECT_LOST_BEFORE_HOME')
                self.assertEqual(second['home_grasp_assessment'],'GRASPED')
                ops=[d['operation'] for k,d in f.effects if k=='compute']
                self.assertEqual(ops.count('home'),2);self.assertEqual(ops.count('retry_home_open'),1)

    def test_home_with_unknown_assessment_blocks_without_release_or_retry(self):
        f=Fixture();f.start()
        for _ in range(1000):
            if f.m.state=='RETURN_HOME':f.owner.update(grasp_assessment='UNKNOWN',grasp_assessment_reason='FEEDBACK_INVALID_OR_STALE')
            f.tick()
            if f.m.state in TERMINAL:break
        self.assertEqual(f.m.state,'BLOCKED')
        self.assertTrue(f.m.reason.startswith('GRASP_ASSESSMENT_UNKNOWN_AT_HOME:'))
        self.assertEqual(f.m.full_attempt,1);self.assertTrue(f.owner['arm_home_ready'])
        self.assertIsNotNone(f.m.base_motion_window['home_completed_ns'])
        self.assertFalse(any(k=='compute' and d['operation']=='retry_home_open' for k,d in f.effects))

    def test_loss_on_second_home_finishes_not_grasped_without_third_attempt(self):
        f=Fixture();f.start()
        for _ in range(2000):
            if f.m.state=='RETURN_HOME':f.owner.update(grasp_assessment='NOT_GRASPED',gripper_hold_target=0.)
            f.tick()
            if f.m.state in TERMINAL:break
        self.assertEqual(f.m.state,'TARGET_NOT_GRASPED');self.assertEqual(f.m.full_attempt,2)
        self.assertEqual([r['reason'] for r in f.m.attempt_history],['OBJECT_LOST_BEFORE_HOME']*2)
        self.assertEqual(sum(k=='compute' and d['operation']=='home' for k,d in f.effects),2)

    def test_full_retry_success_after_each_routine_failure(self):
        for branch in ('not_found','ik_fail','not_grasped'):
            with self.subTest(branch=branch):
                f=Fixture(branch);f.start();start=copy.deepcopy(f.m.return_navigation.start)
                deadline=f.m.started;goal=(f.m.revision,f.m.goal_stamp);original=None
                for _ in range(4000):
                    if f.m.full_attempt==2:f.branch='success'
                    f.tick()
                    if original is None and f.m.original_scene_anchor is not None:original=copy.deepcopy(f.m.original_scene_anchor)
                    if f.m.state in TERMINAL:break
                self.assertEqual(f.m.state,'GRASP_PRESUMED_SUCCESS')
                self.assertEqual(f.m.full_attempt,2);self.assertEqual(len(f.m.attempt_history),2)
                self.assertEqual(f.m.return_navigation.start,start);self.assertEqual(f.m.started,deadline)
                self.assertEqual((f.m.revision,f.m.goal_stamp),goal)
                self.assertEqual(f.m.original_scene_anchor,original)
                self.assertEqual(sum(e['state']=='NAVIGATING' for e in f.m.events),1)
                computes=[d for k,d in f.effects if k=='compute'];ops=[d['operation'] for d in computes]
                self.assertEqual(ops.count('scan_pose'),2);self.assertEqual(ops.count('home'),2)
                self.assertEqual(ops.count('retry_home_open'),int(branch=='not_grasped'))
                second=next(d for d in computes if d['full_attempt']==2 and d['operation']=='scan_pose')
                for key in ('object_anchor','grasp_plan','q_search'):self.assertIsNone(second[key])
                self.assertEqual(second['scan_index'],0)
                self.assertLess(f.m.attempt_history[0]['home_completed_ns'],f.m.attempt_history[1]['scan_started_ns']+1)
                self.assertEqual(f.m.base_motion_window['scan_started_ns'],f.m.attempt_history[0]['scan_started_ns'])
                self.assertEqual(f.m.base_motion_window['home_completed_ns'],f.m.attempt_history[1]['home_completed_ns'])

    def test_full_retry_stops_after_two_failed_grasps(self):
        f=Fixture('not_grasped');self.assertEqual(f.run(),'TARGET_NOT_GRASPED')
        self.assertEqual([r['result'] for r in f.m.attempt_history],['TARGET_NOT_GRASPED']*2)
        ops=[d['operation'] for k,d in f.effects if k=='compute']
        self.assertEqual(ops.count('home'),2);self.assertEqual(ops.count('retry_home_open'),1)
        count=f.commands
        for _ in range(20):f.tick()
        self.assertEqual(f.commands,count)

    def test_full_retry_candidate_failures_use_existing_home(self):
        for reason in ('NO_CANDIDATES','IK_FAIL','NO_TARGET_BOUND_GRASPS','CANDIDATE_GEOMETRY_REJECTED','NO_COLLISION_FREE_CANDIDATE: blocked'):
            with self.subTest(reason=reason):
                f=Fixture('ik_fail');compute=f.compute
                def replacement(data):
                    result=compute(data)
                    if data['operation']=='grasp_plan':result.update(ok=False,reason=reason)
                    return result
                f.compute=replacement
                self.assertEqual(f.run(),'GRASP_UNREACHABLE')
                self.assertEqual([r['reason'] for r in f.m.attempt_history],[reason]*2)
                self.assertEqual(sum(d['operation']=='scan_pose' for k,d in f.effects if k=='compute'),2)

    def test_full_retry_never_bypasses_home_authority_or_unknown_hold(self):
        for fault in ('home','authority','hold','scene'):
            with self.subTest(fault=fault):
                f=Fixture('not_grasped');f.start()
                for _ in range(1000):
                    f.tick()
                    if (f.m.state=='RETURN_HOME' and f.m.pending and f.m.pending['kind']=='motion'
                            and f.owner['mission_id']==f.m.pending['id']):break
                self.assertEqual(f.m.state,'RETURN_HOME')
                if fault=='home':f.owner['arm_home_ready']=False
                f.tick(False)
                if fault=='home':
                    self.assertEqual(f.m.reason,'HOME_UNCONFIRMED');continue
                self.assertEqual(f.m.full_attempt,2)
                if fault=='authority':f.r3['state']='CONTROLLED_STOP'
                if fault=='hold':f.owner['grasp_assessment']='UNKNOWN'
                if fault=='scene':f.tf_revision='changed'
                effects=f.tick(False)
                self.assertIn(f.m.state,('BLOCKED','CANCELLED'))
                self.assertTrue(all(k=='cancel' for k,d in effects))

    def test_search_anchor_reacquires_and_discards_old_detection(self):
        f,request=self.observation_fixture()
        original=copy.deepcopy(f.m.scene_anchor);window=copy.deepcopy(f.m.base_motion_window)
        commands=f.commands;f.base['pose'][0]=.26
        self.assertEqual(f.tick(False),[])
        self.assertTrue(f.m.anchor_recovery['active'])
        self.assertFalse(f.m.result(f.compute(request),f.now))
        self.assertIsNone(f.m.pending);self.assertEqual(f.m.state,'ARM_SEARCH')
        for _ in range(260):f.tick(False)
        self.assertFalse(f.m.anchor_recovery['active'])
        self.assertEqual(f.m.original_scene_anchor,original)
        self.assertEqual(f.m.base_motion_window,window)
        self.assertEqual(f.m.base_anchor[0],.26)
        diagnostic=f.m.status(f.now)['anchor_recovery']
        self.assertEqual(diagnostic['original']['translation_m'],.26)
        self.assertEqual(diagnostic['working']['translation_m'],0.)
        self.assertEqual(f.commands,commands)
        self.assertEqual(f.m.pending['operation'],'detect')
        self.assertNotEqual(f.m.pending['id'],request['id'])
        self.assertFalse(f.m.result(f.compute(request),f.now))
        f.base['pose'][0]=.52;f.tick(False)
        self.assertEqual(f.m.reason,'SCENE_ANCHOR_DISCONTINUITY')

    def test_search_anchor_gradual_drift_then_fresh_grasp_reaches_home(self):
        f,request=self.observation_fixture()
        for _ in range(26):
            f.base['pose'][0]+=.01;f.tick(False)
        self.assertTrue(f.m.anchor_recovery['active'])
        f.m.result(f.compute(request),f.now)
        for _ in range(1000):
            f.tick()
            if f.m.state in TERMINAL:break
        self.assertEqual(f.m.state,'GRASP_PRESUMED_SUCCESS')
        self.assertIsNotNone(f.m.base_motion_window['home_completed_ns'])
        self.assertAlmostEqual(f.m.anchor_diagnostic()['original']['translation_m'],.26)

    def test_search_anchor_result_callback_cannot_race_drift_guard(self):
        f,request=self.observation_fixture();f.base['pose'][0]=.26;f.feed()
        self.assertFalse(f.m.result(f.compute(request),f.now))
        self.assertEqual(f.m.state,'ARM_SEARCH')
        self.assertTrue(f.m.anchor_recovery['active']);self.assertIsNone(f.m.pending)
        self.assertEqual(f.m.drain(),[])

    def test_search_anchor_keeps_motion_tilt_epoch_authority_guards(self):
        for failure in ('motion','tilt','epoch','authority','command','arm_moved','grasp_phase'):
            with self.subTest(failure=failure):
                f,request=self.observation_fixture();f.base['pose'][0]=.26
                if failure=='motion':f.owner['mission_state']='EXECUTING'
                if failure=='tilt':f.base['tilt_rad']=f.m.cfg['settle']['max_tilt_rad']+.01
                if failure=='epoch':f.tf_revision='changed'
                if failure=='authority':f.r3['state']='CONTROLLED_STOP'
                if failure=='command':f.r3['motion_command']=[.01,0,0]
                if failure=='arm_moved':f.owner['q'][0]+=.03
                if failure=='grasp_phase':f.m.state='PREGRASP'
                effects=f.tick(False)
                self.assertIn(f.m.state,('BLOCKED','CANCELLED'))
                self.assertTrue(all(kind=='cancel' for kind,_ in effects))
                self.assertEqual(f.m.anchor_recovery['attempts'],0)

    def test_search_anchor_timeout_including_stale_scene_and_pending_compute(self):
        for mode in ('drifting','missing_scene','pending_compute'):
            with self.subTest(mode=mode):
                f,request=self.observation_fixture();f.base['pose'][0]=.26;f.tick(False)
                if mode!='pending_compute':f.m.result(f.compute(request),f.now)
                for i in range(770):
                    if mode=='drifting':f.base['pose'][0]+=.001
                    f.feed()
                    if mode=='missing_scene':f.m.received.pop('scene',None)
                    effects=f.m.step(f.now,int(f.now*1e9))
                    self.assertTrue(all(kind=='cancel' for kind,_ in effects))
                    if f.m.state in TERMINAL:break
                self.assertEqual(f.m.reason,'SEARCH_ANCHOR_RECOVERY_TIMEOUT')

    def test_search_anchor_recovery_still_blocks_on_live_faults(self):
        for failure in ('tilt','command','epoch','arm_moved','odometry','authority'):
            with self.subTest(failure=failure):
                f,request=self.observation_fixture();f.base['pose'][0]=.26;f.tick(False)
                f.m.result(f.compute(request),f.now)
                if failure=='tilt':f.base['tilt_rad']=.5
                if failure=='command':f.r3['motion_command']=[.01,0,0]
                if failure=='epoch':f.tf_revision='changed'
                if failure=='arm_moved':f.owner['q'][0]+=.03
                if failure=='odometry':f.perception=dict(ready=False,reason='ODOMETRY_DISCONTINUITY')
                if failure=='authority':f.r3['state']='CONTROLLED_STOP'
                f.tick(False)
                self.assertIn(f.m.state,('BLOCKED','CANCELLED'))
                self.assertEqual(f.m.base_anchor[0],0.)

    def test_search_anchor_stability_window_restarts_and_does_not_use_repeated_scene(self):
        f,request=self.observation_fixture();f.base['pose'][0]=.26;f.tick(False)
        f.m.result(f.compute(request),f.now)
        for _ in range(150):f.tick(False)
        since=f.m.anchor_recovery['stable_since']
        f.base['pose'][0]+=.021;f.tick(False)
        self.assertGreater(f.m.anchor_recovery['stable_since'],since)
        for _ in range(150):f.tick(False)
        self.assertTrue(f.m.anchor_recovery['active'])
        old=copy.deepcopy(f.m.inputs['scene']);received=f.m.received['scene']
        for _ in range(260):
            f.feed();f.m.inputs['scene']=copy.deepcopy(old);f.m.received['scene']=received
            f.m.step(f.now,int(f.now*1e9))
        self.assertTrue(f.m.anchor_recovery['active']);self.assertIsNone(f.m.pending)
        self.assertEqual(f.m.base_anchor[0],0.)

    def test_transient_observation_retries_same_pose_and_new_identity(self):
        f,request=self.observation_fixture();index=f.m.scan_index;commands=f.commands
        failed=f.compute(request);failed.update(ok=False,reason='RGBD_STALE_OR_REPEATED')
        self.assertTrue(f.m.result(failed,f.now));self.assertNotIn(f.m.state,TERMINAL)
        self.assertFalse(f.m.result(failed,f.now)) # old result cannot advance retry
        retry=None
        for _ in range(30):
            for kind,data in f.tick(False):
                self.assertNotEqual(kind,'trajectory')
                if kind=='compute':retry=data
            if retry:break
        self.assertIsNotNone(retry);self.assertGreater(retry['id'],request['id'])
        self.assertEqual(retry['operation'],'detect');self.assertEqual(f.m.scan_index,index)
        self.assertEqual(f.commands,commands)
        self.assertTrue(f.m.result(f.compute(retry),f.now))
        self.assertEqual(f.m.state,'PLAN_GRASP');self.assertIsNone(f.m.recovery_since)

    def test_refinement_toggle_keeps_fresh_grasp_plan_operation(self):
        for enabled,expected in ((False,'PLAN_GRASP'),(True,'VIEW_REFINEMENT')):
            f,request=self.observation_fixture()
            f.m.cfg['refinement']['enabled']=enabled
            self.assertTrue(f.m.result(f.compute(request),f.now))
            self.assertEqual(f.m.state,expected)
            self.assertEqual(f.m.pending['operation'],'refine' if enabled else 'grasp_plan')

    def test_scene_lag_waits_then_recovers_without_cancel(self):
        f,request=self.observation_fixture()
        f.perception.update(ready=False,reason='SCENE_SNAPSHOT_STALE:body')
        for _ in range(30):self.assertEqual(f.tick(False),[])
        self.assertEqual(f.m.state,'ARM_SEARCH')
        self.assertTrue(f.m.perception_service_ready(f.now))
        self.assertFalse(f.m.inputs['perception']['ready'])
        self.assertFalse(f.m.perception_service_ready(f.now+f.m.cfg['execution']['perception_status_ttl_s']+.01))
        f.perception.update(ready=True,reason='READY');f.tick(False)
        self.assertTrue(f.m.result(f.compute(request),f.now));self.assertEqual(f.m.state,'PLAN_GRASP')

    def test_recovered_health_does_not_timeout_ongoing_grasp_plan(self):
        f,detect=self.observation_fixture()
        self.assertTrue(f.m.result(f.compute(detect),f.now))
        self.assertEqual(f.m.state,'PLAN_GRASP')
        request=next(data for kind,data in f.m.drain() if kind=='compute')
        pending_id=f.m.pending['id'];commands=f.commands
        f.perception.update(ready=False,reason='SCENE_SNAPSHOT_STALE:live_arm')
        f.tick(False);since=f.m.recovery_since
        self.assertIsNotNone(since)
        f.now+=.2;f.perception.update(ready=True,reason='READY');f.tick(False)
        self.assertIsNone(f.m.recovery_since)
        # Same original computation continues beyond the former recovery deadline.
        f.now=since+31.;f.tick(False)
        self.assertEqual(f.m.state,'PLAN_GRASP')
        self.assertEqual(f.m.pending['id'],pending_id)
        self.assertEqual(f.commands,commands)
        self.assertTrue(f.m.result(f.compute(request),f.now))
        self.assertEqual(f.m.state,'PREGRASP')

    def test_real_retry_in_flight_keeps_deadline_despite_ready_status(self):
        f,request=self.observation_fixture()
        failed=f.compute(request);failed.update(ok=False,reason='RGBD_STALE_OR_REPEATED')
        f.m.result(failed,f.now);since=f.m.recovery_since
        for _ in range(30):
            effects=f.tick(False)
            if any(kind=='compute' for kind,_ in effects):break
        self.assertIsNotNone(f.m.pending)
        self.assertTrue(f.m.recovery_retry_active)
        self.assertTrue(f.perception['ready'])
        self.assertEqual(f.m.recovery_since,since)
        f.now=since+31.;f.tick(False)
        self.assertEqual(f.m.state,'BLOCKED')
        self.assertTrue(f.m.reason.startswith('OBSERVATION_RECOVERY_TIMEOUT:'))

    def grasp_retry_fixture(self):
        f,detect=self.observation_fixture()
        f.m.result(f.compute(detect),f.now)
        request=next(data for kind,data in f.m.drain() if kind=='compute')
        failed=f.compute(request);failed.update(ok=False,reason='RGBD_STALE_OR_REPEATED')
        f.m.result(failed,f.now)
        for _ in range(30):
            for kind,retry in f.tick(False):
                if kind=='compute':
                    marker={key:retry[key] for key in ('id','operation','session_id',
                        'navigation_session_id','owner_session_id','goal_revision')}
                    marker['verified_ns']=int(f.now*1e9)
                    return f,retry,marker
        self.fail('No retry')

    def test_verified_retry_ends_recovery_but_keeps_planning_deadline(self):
        f,retry,marker=self.grasp_retry_fixture();since=f.m.recovery_since
        f.m.cfg['execution']['state_timeout_s']=300.
        entered=f.m.entered;commands=f.commands
        f.perception['observation_verified']=marker;f.tick(False)
        self.assertIsNone(f.m.recovery_since);self.assertFalse(f.m.recovery_retry_active)
        f.now=since+31.;f.tick(False)
        self.assertEqual(f.m.state,'PLAN_GRASP');self.assertEqual(f.m.pending['id'],retry['id'])
        self.assertEqual(f.commands,commands);self.assertEqual(f.m.entered,entered)
        # The old milestone cannot clear a later live-feedback outage.
        f.perception.update(ready=False,reason='SCENE_SNAPSHOT_STALE:live_arm')
        f.tick(False);new_since=f.m.recovery_since
        f.tick(False);self.assertEqual(f.m.recovery_since,new_since)
        f.perception.update(ready=True,reason='READY');f.tick(False)
        f.now=entered+f.m.cfg['execution']['state_timeout_s']+.1;f.tick(False)
        self.assertEqual(f.m.state,'BLOCKED');self.assertEqual(f.m.reason,'PHASE_TIMEOUT')

    def test_unverified_or_late_retry_milestone_cannot_clear_deadline(self):
        for invalid in ('id','session_id','navigation_session_id','owner_session_id',
                        'goal_revision','operation','before_request','future','late','stale_health','stale_scene'):
            with self.subTest(invalid=invalid):
                f,retry,marker=self.grasp_retry_fixture();since=f.m.recovery_since
                if invalid=='before_request':marker['verified_ns']=int((retry['at']-.1)*1e9)
                elif invalid=='future':marker['verified_ns']=int((f.now+10)*1e9)
                elif invalid=='late':f.now=since+31.;marker['verified_ns']=int(f.now*1e9)
                elif invalid not in ('stale_health','stale_scene'):marker[invalid]='wrong'
                f.perception['observation_verified']=marker;f.feed()
                if invalid=='stale_health':f.m.received['perception']=f.now-2.
                if invalid=='stale_scene':f.m.inputs['scene']['live_feedback_ns']=int((f.now-f.m.cfg['limits']['arm_age_s']-.01)*1e9)
                f.m.wait_for_observation(f.now)
                self.assertEqual(f.m.recovery_since,since)
                if invalid=='late':self.assertEqual(f.m.state,'BLOCKED')

    def test_ready_status_cannot_clear_recovery_with_stale_measurements(self):
        f,request=self.observation_fixture()
        f.perception.update(ready=False,reason='SCENE_SNAPSHOT_STALE:live_arm');f.tick(False)
        since=f.m.recovery_since
        f.perception.update(ready=True,reason='READY');f.feed()
        f.m.inputs['scene']['live_feedback_ns']=int((f.now-f.m.cfg['limits']['arm_age_s']-.01)*1e9)
        self.assertTrue(f.m.wait_for_observation(f.now))
        self.assertEqual(f.m.recovery_since,since)
        self.assertIn('STALE',f.m.recovery_reason)

    def test_recovery_is_bounded_and_does_not_hide_authority_loss(self):
        for failure in ('timeout','authority','epoch','motion'):
            with self.subTest(failure=failure):
                f,request=self.observation_fixture()
                f.perception.update(ready=False,reason='SCENE_SNAPSHOT_STALE:body')
                if failure=='timeout':
                    f.m.cfg['execution']['observation_recovery_timeout_s']=.1
                    for _ in range(8):f.tick(False)
                    self.assertTrue(f.m.reason.startswith('OBSERVATION_RECOVERY_TIMEOUT:'))
                elif failure=='authority':
                    f.r3['fault_latched']=True;f.tick(False)
                elif failure=='epoch':
                    f.perception['reason']='WORLD_ANCHOR_EPOCH_CHANGED';f.tick(False)
                else:
                    f.m.pending=dict(f.m.pending,kind='motion');f.tick(False)
                self.assertIn(f.m.state,TERMINAL)
                self.assertFalse(f.m.perception_service_ready(f.now))
                self.assertFalse(f.m.result(f.compute(request),f.now))

    def test_repeated_failed_frames_share_one_recovery_deadline(self):
        f,request=self.observation_fixture();f.m.cfg['execution']['observation_recovery_timeout_s']=.8
        failed=f.compute(request);failed.update(ok=False,reason='RGBD_STALE_OR_REPEATED')
        f.m.result(failed,f.now);since=f.m.recovery_since
        for _ in range(60):
            for kind,data in f.tick(False):
                if kind=='compute':
                    failed=f.compute(data);failed.update(ok=False,reason='RGBD_STALE_OR_REPEATED')
                    f.m.result(failed,f.now);self.assertEqual(f.m.recovery_since,since)
            if f.m.state in TERMINAL:break
        self.assertEqual(f.m.state,'BLOCKED')
        self.assertTrue(f.m.reason.startswith('OBSERVATION_RECOVERY_TIMEOUT:'))

    def test_retry_covers_observe_and_world_grasp_plan(self):
        for op in ('observe_grasp','grasp_plan'):
            with self.subTest(operation=op):
                f,request=self.observation_fixture()
                f.m.pending['operation']=op
                if op=='grasp_plan':f.m.state='PLAN_GRASP'
                failed=f.compute(request);failed.update(ok=False,reason='SCENE_SNAPSHOT_SKEW')
                f.m.result(failed,f.now)
                operations=[]
                for _ in range(20):
                    for kind,data in f.tick(False):
                        self.assertNotEqual(kind,'trajectory')
                        if kind=='compute':operations.append(data['operation'])
                self.assertEqual(operations,[op])

    def test_recovery_preserves_nav_health_lease_and_blocks_new_start(self):
        f,request=self.observation_fixture()
        f.perception.update(ready=False,reason='BODY_TF_STALE_OR_UNBOUND')
        f.tick(False)
        h=MissionHealth(.25);gid=b'recovery-test';h.graph([gid],int((f.now-.01)*1e9))
        h.observe(json.dumps(dict(observed_ns=int(f.now*1e9),session_id=f.m.session,
            navigation_session_id='nav',ready=f.m.perception_service_ready(f.now),
            physical=True,base_locked=True)),gid,int(f.now*1e9),'nav')
        self.assertTrue(h.valid(int(f.now*1e9),'nav'));self.assertTrue(h.base_locked)
        self.assertFalse(h.valid(int((f.now+1)*1e9),'nav'))
        fresh=Fixture();fresh.perception.update(ready=False,reason='BODY_TF_STALE_OR_UNBOUND')
        fresh.feed();self.assertFalse(fresh.m.perception_service_ready(fresh.now))

    def test_late_success_does_not_extend_recovery_deadline(self):
        f,request=self.observation_fixture();f.m.defer_observation('BODY_TF_STALE_OR_UNBOUND',f.now)
        f.now+=31;result=f.compute(request)
        self.assertFalse(f.m.result(result,f.now));self.assertEqual(f.m.state,'BLOCKED')
        self.assertTrue(f.m.reason.startswith('OBSERVATION_RECOVERY_TIMEOUT:'))

    def test_stale_input_identifies_source_and_freezes_original_evidence(self):
        f=Fixture();f.tick();f.feed()
        f.m.received['owner']=f.now-.31
        f.m.step(f.now,int(f.now*1e9))
        self.assertEqual(f.m.reason,'STALE_INPUT:owner')
        d=copy.deepcopy(f.m.stale_input_diagnostic)
        self.assertEqual(d['stale_sources'],['owner'])
        self.assertEqual(d['state_before'],'WAIT_GOAL')
        self.assertAlmostEqual(d['inputs']['owner']['accepted_age_s'],.31)
        f.tick(False)
        self.assertEqual(f.m.stale_input_diagnostic,d)

    def test_input_diagnostics_distinguishes_silence_decode_and_producer_age(self):
        from mission.input_diagnostics import InputDiagnostics
        d=InputDiagnostics();d.tick(1.);d.tick(1.4)
        d.callback('nav',1.,1.01,'accepted',{'observed_ns':990000000},1)
        d.callback('nav',1.3,1.32,'stamp_rejected',{'observed_ns':990000000},1)
        d.callback('owner',1.3,1.31,'publisher_count_invalid',None,2)
        a=d.snapshot(1.5);r=a['streams']['nav']
        self.assertEqual((r['accepted'],r['rejected']),(1,1))
        self.assertAlmostEqual(r['producer_age_at_decode_s'],.33)
        self.assertAlmostEqual(r['callback_silence_s'],.2)
        self.assertAlmostEqual(a['tick_gap_s'],.4)
        self.assertEqual(a['streams']['owner']['publishers'],2)
        d.callback('nav',1.6,1.61,'accepted',{'observed_ns':1600000000},1)
        self.assertEqual(r['callbacks'],2) # frozen report is not a live view

    def test_immediately_succeeded_goal_still_waits_for_zero_and_settle(self):
        f=Fixture();f.tick()
        f.nav['goal']=dict(revision=1,stamp_ns=int((f.now+.01)*1e9),state='SUCCEEDED')
        self.assertEqual(f.tick(False),[]);self.assertEqual(f.m.state,'NAVIGATING')
        self.assertEqual(f.tick(False),[]);self.assertEqual(f.m.state,'BASE_SETTLE')
        f.r3['motion_command']=[.1,0,0]
        self.assertEqual(f.tick(False),[])
        self.assertEqual(f.m.stability_reason,'WAITING_FOR_RL_ZERO')
        self.assertIsNone(f.m.stable_since)
        f.r3['motion_command']=[0,0,0];f.tick(False)
        since=f.m.stable_since;self.assertIsNotNone(since)
        f.now=since+f.m.cfg['settle']['min_stable_s']-.1
        self.assertEqual(f.tick(False),[]);self.assertEqual(f.m.state,'BASE_SETTLE')
        f.now=since+f.m.cfg['settle']['min_stable_s']
        effects=f.tick(False)
        self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        self.assertEqual([(kind,data['operation']) for kind,data in effects],[('compute','scan_pose')])
        self.assertEqual(f.commands,0)

    def test_succeeded_goal_does_not_bypass_identity_or_failure_guards(self):
        for case in ('old_revision','old_stamp','wrong_session','cancelled','failed','localization_lost'):
            with self.subTest(case=case):
                f=Fixture();f.tick()
                f.nav['goal']=dict(revision=1,stamp_ns=int((f.now+.01)*1e9),state='SUCCEEDED')
                if case=='old_revision':f.nav['goal']['revision']=f.m.last_revision
                elif case=='old_stamp':f.nav['goal']['stamp_ns']=f.m.armed_ros_ns
                elif case=='wrong_session':f.nav['session_id']='another-navigation'
                else:f.nav['goal']['state']=case.upper()
                effects=f.tick(False)
                self.assertEqual(f.m.state,'CANCELLED' if case=='wrong_session' else 'WAIT_GOAL')
                self.assertFalse(any(kind in ('compute','trajectory') for kind,_ in effects))

    def test_succeeded_goal_from_before_localization_recovery_is_not_reused(self):
        f=Fixture();f.tick();f.nav['navigation_ready']=False;f.tick(False)
        f.nav['goal']=dict(revision=1,stamp_ns=int(f.now*1e9),state='SUCCEEDED')
        f.nav['navigation_ready']=True;f.tick(False)
        self.assertEqual(f.m.reason,'RECOVERED_NEW_GOAL_REQUIRED')
        self.assertEqual(f.tick(False),[]);self.assertEqual(f.m.state,'WAIT_GOAL')
        f.nav['goal']=dict(revision=2,stamp_ns=int((f.now+.01)*1e9),state='SUCCEEDED')
        f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')

    def test_wait_goal_nav_stale_recovers_requires_new_goal(self):
        f=Fixture();f.tick();f.feed();f.m.received['nav']=f.now-.601
        effects=f.m.step(f.now,int(f.now*1e9))
        self.assertEqual(f.m.state,'WAIT_GOAL');self.assertEqual([k for k,d in effects],['invalidate_navigation_goal'])
        self.assertEqual(f.m.stale_input_diagnostic['stale_sources'],['nav'])
        f.nav['goal']=dict(revision=1,stamp_ns=int(f.now*1e9),state='ACTIVE')
        f.tick(False);self.assertEqual(f.m.reason,'RECOVERED_NEW_GOAL_REQUIRED')
        f.tick(False);self.assertEqual(f.m.state,'WAIT_GOAL')
        f.nav['goal']=dict(revision=2,stamp_ns=int((f.now+.01)*1e9),state='REQUESTED')
        f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')

    def test_wait_goal_soft_loss_does_not_latch_but_owner_loss_still_blocks(self):
        f=Fixture();f.tick();f.feed();f.m.received['nav']=f.now-.601
        f.m.step(f.now,int(f.now*1e9));since=f.m.wait_goal_stale_since
        f.now+=31;f.feed();f.m.step(f.now,int(f.now*1e9))
        self.assertEqual(f.m.reason,'RECOVERED_NEW_GOAL_REQUIRED')
        self.assertEqual(f.m.state,'WAIT_GOAL');self.assertIsNone(f.m.wait_goal_stale_since)
        f=Fixture();f.tick();f.feed();f.m.received['owner']=f.now-.3
        f.m.step(f.now,int(f.now*1e9));self.assertEqual(f.m.state,'BLOCKED')

    def test_closed_gripper_measurement_tolerance_is_not_command_expansion(self):
        from mission.collision import measured_gripper_width
        class Geometry:
            def opening_width(self,a):
                if not 0<=a<=1:raise ValueError('outside limits')
                return a
        g=Geometry();tol=.025
        for a in (-.00705718994,-tol,0.,tol,.02,float(np.float32(-tol)),float(np.float32(tol))):self.assertEqual(measured_gripper_width(g,a,tol),0.)
        self.assertEqual(measured_gripper_width(g,tol+1e-6,tol),tol+1e-6)
        for a in (-tol-1e-6,1.01,float('nan')):
            with self.assertRaises(ValueError):measured_gripper_width(g,a,tol)

    def test_gripper_diagnostics_do_not_gate_mission(self):
        # Neither uncertain nor malformed optional telemetry is a grasp/HOME gate.
        for torque in (0.,2.3,float('nan')):
            with self.subTest(torque=torque):
                f=Fixture();f.owner['torque']=[0.]*6+[torque]
                f.owner['gripper_observation']={'state':'UNKNOWN','error':'disk full'}
                self.assertEqual(f.run(),'GRASP_PRESUMED_SUCCESS')
                self.assertTrue(f.owner['arm_home_ready'])

    def test_detection_weak_frames_are_not_mission_faults(self):
        from mission.planning import Planning
        from types import SimpleNamespace
        from unittest.mock import patch
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
        planner.cfg.update(object_class='light blue coffee cup',object_class_aliases=['cup'])
        planner.grasp={'graspnet':{'min_depth':.05,'max_depth':1.}}
        planner.camera=SimpleNamespace(K=np.eye(3));planner.yolo=None;planner.yolo_options={}
        detection=SimpleNamespace(class_name='cup',conf=.9,mask=None)
        depth=np.zeros((3,3));sample={'odom_arm':np.eye(4),'arm_camera':np.eye(4)}
        planner.frame=lambda context:(None,depth,sample)
        module=SimpleNamespace(detect_objects=lambda *args:(None,[detection]))
        with patch.dict(sys.modules,{'utils.yolo_utils':module}):
            self.assertIsNone(planner.detect(None,1))
            detection.mask=np.ones((3,3),dtype=bool)
            self.assertIsNone(planner.detect(None,1))
            depth[1,1]=500 # no arbitrary 20-pixel admission threshold
            self.assertIsNotNone(planner.detect(None,1))
            planner.frame=lambda context:(_ for _ in ()).throw(ValueError('RGBD_STALE_OR_REPEATED'))
            with self.assertRaisesRegex(ValueError,'RGBD_STALE'):planner.detect(None,1)

    def test_search_confirms_two_of_five_without_relaxing_geometry(self):
        from mission.planning import Planning
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        def run(frames,required=2,class_name="cup",confidence=.9):
            planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
            planner.cfg.update(object_class='light blue coffee cup',object_class_aliases=['cup'])
            planner.cfg['search']['max_center_shift_px']=1.
            planner.grasp={'graspnet':{'min_depth':.05,'max_depth':1.}}
            planner.camera=NS(K=np.eye(3));planner.yolo=None;planner.yolo_options={}
            planner.frame=lambda context:(None,np.full((5,5),500),{'odom_arm':np.eye(4),'arm_camera':np.eye(4)})
            def detection(position):
                if position is None:return []
                mask=np.zeros((5,5),bool);mask[position,position]=True
                return [NS(class_name=class_name,conf=confidence,mask=mask)]
            sequence=iter(frames)
            module=NS(detect_objects=lambda *a:(None,detection(next(sequence))))
            with patch.dict(sys.modules,{'utils.yolo_utils':module}):result=planner.detect(None,5,required=required)
            return result,planner.compute_metrics
        result,metrics=run([1,1],class_name='light blue coffee cup')
        self.assertIsNotNone(result)
        result,metrics=run([1]*5,class_name='bottle')
        self.assertIsNone(result)
        result,metrics=run([1]*5,class_name='light blue coffee cup',confidence=.24)
        self.assertIsNone(result)
        result,metrics=run([None,1,None,None,1])
        self.assertIsNotNone(result);self.assertEqual(metrics['confirmation_frames'],[2,5])
        result,metrics=run([1,1])
        self.assertIsNotNone(result);self.assertEqual(metrics['yolo_calls'],2)
        result,metrics=run([None,1,None,None,None])
        self.assertIsNone(result);self.assertEqual(metrics['yolo_calls'],5)
        result,metrics=run([0,None,4,None,None])
        self.assertIsNone(result) # two spatially inconsistent detections
        result,metrics=run([0,None,4,None,4])
        self.assertIsNotNone(result);self.assertEqual(metrics['confirmation_frames'],[3,5])
        result,metrics=run([1,None,1,1,1],required=None)
        self.assertIsNone(result) # refinement still requires consecutive frames

    def test_search_speed_does_not_limit_grasp_motion(self):
        from mission.planning import Planning
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
        planner.cfg['search']['speed_rad_s']=.01
        planner.grasp={'robot':{'rars01':{'position_velocity_limits_rad_s':[1.]*7,'command_rate_hz':100}}}
        start=np.zeros(7);target=np.array([.1,0,0,0,0,0])
        regular=planner.samples(start,[target],.1)
        scanning=planner.samples(start,[target],.1,search=True)
        self.assertGreater(len(scanning),len(regular)*5)
        planner.cfg['search']['speed_rad_s']=.005
        np.testing.assert_array_equal(regular,planner.samples(start,[target],.1))

    def test_geometric_rejection_keeps_infrastructure_reason(self):
        from mission.planning import Planning,CandidatePathRejected
        from types import SimpleNamespace
        planner=Planning.__new__(Planning);planner.cfg=CFG
        planner.collision_scene=lambda sample:{}
        for reason in ('COLLISION:arm:leg','COLLISION_STATE_STALE:legs_ns','PAYLOAD_GEOMETRY_MISSING'):
            planner.collision=SimpleNamespace(check=lambda *args:False,last_reason=reason,last_failure={'reason':reason})
            with self.assertRaises(ValueError) as failure:planner.checked([np.zeros(7)]*2,{'scene_snapshot':{}},False)
            self.assertEqual(isinstance(failure.exception,CandidatePathRejected),reason.startswith('COLLISION:'))
            self.assertIn(reason,str(failure.exception))

    def test_object_lost_after_refinement_resumes_bounded_search(self):
        f=Fixture();f.start()
        for _ in range(1000):
            if f.m.state=='PLAN_GRASP':break
            f.tick()
        for _ in range(100):
            f.tick(False)
            if f.m.pending:break
        self.assertEqual(f.m.pending['operation'],'grasp_plan')
        p=f.m.pending
        result=dict(id=p['id'],session_id=f.m.session,navigation_session_id=f.m.nav_session,
                    owner_session_id=f.m.owner_session,goal_revision=f.m.revision,observed_ns=int(f.now*1e9),
                    ok=False,reason='OBJECT_LOST_AFTER_REFINEMENT')
        self.assertTrue(f.m.result(result,f.now));self.assertEqual(f.m.state,'ARM_SEARCH')
        self.assertEqual(f.m.scan_index,1);self.assertEqual(f.m.pending['operation'],'scan_step')
        f.m.scan_index=CFG['search']['max_attempts']-1;f.m.next_search(f.now)
        self.assertEqual(f.m.state,'RETURN_HOME');self.assertEqual(f.m.result_after_home,'OBJECT_NOT_FOUND')

    def test_refinement_yaw_limits_and_camera_axis_sign(self):
        from mission.refinement import select_scan_yaw
        for sign in (-1,1):
            def camera(q):return [np.sin(.3-sign*q),.4,np.cos(.3-sign*q)]
            q,info=select_scan_yaw(0,camera,-2.5,2.5,.5,.02)
            self.assertAlmostEqual(q,sign*.3,places=6)
            self.assertTrue(info['centered'])
            self.assertLess(info['after_rad'],info['before_rad'])
        q,info=select_scan_yaw(0,lambda q:[1-q,0,1],-.2,.2,.5,.02)
        self.assertAlmostEqual(q,.2)
        self.assertFalse(info['centered'])
        q,info=select_scan_yaw(0,lambda q:[0,0,1],-2,2,.5,.02)
        self.assertEqual(q,0)
        with self.assertRaisesRegex(ValueError,'BEHIND_CAMERA'):
            select_scan_yaw(0,lambda q:[0,0,-1],-2,2,.5,.02)

    def test_zero_first_sweep_return_zero_then_home(self):
        f=Fixture('not_found');self.assertEqual(f.run(),'OBJECT_NOT_FOUND')
        operations=[data for kind,data in f.effects if kind=='compute']
        scans=[d for d in operations if d['operation'] in ('scan_pose','scan_step','scan_return','home')]
        self.assertEqual([d['operation'] for d in scans],(['scan_pose']+['scan_step']*15+['scan_return','home'])*2)
        self.assertEqual([d['scan_index'] for d in scans[1:16]],list(range(1,16)))
        observations=[d['owner']['q'][0] for d in operations if d['operation']=='detect']
        np.testing.assert_allclose(observations,[0.,-.5,-1.,-1.5,-2.,-2.5,-2.,-1.5,-1.,-.5,0.,.5,1.,1.5,2.,2.5]*2)
        self.assertEqual(scans[-1]['owner']['q'][0],0.)
        f=Fixture();f.start()
        for _ in range(1000):
            f.tick(False)
            if f.m.state=='ARM_SCAN_POSE':break
        f.m.pending=None;f.m.done_motion('scan_pose',f.now)
        self.assertAlmostEqual(f.m.detect_at-f.now,1.)
        for _ in range(49):
            effects=f.tick(False)
            self.assertFalse(any(kind=='compute' and data['operation']=='detect' for kind,data in effects))
        effects=f.tick(False)+f.tick(False) # next timer tick after the 1s boundary
        self.assertTrue(any(kind=='compute' and data['operation']=='detect' for kind,data in effects))

    def test_wide_search_configuration_and_motion_budget(self):
        import tempfile,yaml
        from rars01_graspnet.real_grasp_trajectory import joint_motion_samples
        search=CFG['search'];step=search['joint1_step_rad']
        angles=np.asarray(search_angles(search))
        self.assertEqual(angles[0],0.)
        self.assertEqual(search['settle_s'],1.)
        self.assertEqual(search['mode'],'stop_and_observe')
        np.testing.assert_allclose(angles,[0.,-.5,-1.,-1.5,-2.,-2.5,-2.,-1.5,-1.,-.5,0.,.5,1.,1.5,2.,2.5],atol=1e-12)
        self.assertEqual(search['speed_rad_s'],.2)
        # Exercise the unchanged real interpolation, including its 80% reserve.
        points,duration=joint_motion_samples([angles[1]],[angles[2]],.01,.01,[search['speed_rad_s']])
        q=np.asarray(points)[:,0]
        self.assertLessEqual(np.max(np.abs(np.diff(q)))/.01,search['speed_rad_s'])
        self.assertTrue(np.all(np.diff(q)<=0));self.assertAlmostEqual(q[-1],angles[2])
        self.assertIsNone(search['timeout_s'])
        _,entry_duration=joint_motion_samples([0.],[angles[1]],.01,.01,[search['speed_rad_s']])
        self.assertLessEqual(entry_duration,CFG['execution']['max_plan_duration_s'])
        self.assertIsNone(CFG['execution']['state_timeout_s'])
        self.assertIsNone(CFG['collision']['check_timeout_s'])
        raw=yaml.safe_load((ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())['go2_mission']
        with tempfile.TemporaryDirectory(prefix='wide-search-config-') as directory:
            path=Path(directory)/'mission.yaml'
            for mode,reason in [('continuous','reserved but not implemented'),('typo','invalid search.mode')]:
                invalid=copy.deepcopy(raw);invalid['search']['mode']=mode;path.write_text(yaml.safe_dump({'go2_mission':invalid}))
                with self.assertRaisesRegex(ValueError,reason):load(path,ROOT)
            invalid=copy.deepcopy(raw);invalid['search']['max_attempts']-=1;path.write_text(yaml.safe_dump({'go2_mission':invalid}))
            with self.assertRaisesRegex(ValueError,'limits/step/attempts disagree'):load(path,ROOT)
            invalid=copy.deepcopy(raw);invalid['search'].update(joint1_min_rad=-3.,joint1_max_rad=3.,max_attempts=19)
            path.write_text(yaml.safe_dump({'go2_mission':invalid}))
            with self.assertRaisesRegex(ValueError,'exceeds joint1 limits'):load(path,ROOT)

    def at(self,f,now):
        f.now=now-.02
        return f.tick(False)

    def test_navigation_phases_and_delayed_zero_ack(self):
        f=Fixture();f.tick();self.assertEqual(f.m.state,'WAIT_GOAL')
        # A velocity status can arrive before the slower goal-status topic.
        f.r3.update(phase='RL_ACTIVE',motion_command=[.1,0,0])
        f.tick(False);self.assertEqual(f.m.state,'WAIT_GOAL')
        f.nav['goal']=dict(revision=1,stamp_ns=int((f.now+.01)*1e9),state='ACTIVE')
        f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')
        for phase,command in [('RL_ZERO',[0,0,0]),('RL_ACTIVE',[.1,0,0])]:
            f.r3.update(phase=phase,motion_command=command)
            f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')
        f.nav['goal']['state']='SUCCEEDED'
        f.tick(False);self.assertEqual(f.m.state,'BASE_SETTLE')
        self.at(f,f.m.goal_reached+2.)
        self.assertEqual(f.m.state,'BASE_SETTLE');self.assertIsNone(f.m.stable_since)
        self.assertEqual(f.m.stability_reason,'WAITING_FOR_RL_ZERO')
        f.r3.update(phase='RL_ZERO',motion_command=[0,0,0]);f.tick(False)
        since=f.m.stable_since;self.assertIsNotNone(since)
        self.at(f,since+3.)
        f.r3.update(phase='RL_ACTIVE',motion_command=[.1,0,0]);f.tick(False)
        self.assertIsNone(f.m.stable_since)
        f.r3.update(phase='RL_ZERO',motion_command=[0,0,0]);f.tick(False)
        self.assertGreater(f.m.stable_since,since);since=f.m.stable_since
        self.at(f,since+4.999)
        self.assertEqual(f.m.state,'BASE_SETTLE');self.assertIsNone(f.m.pending)
        self.at(f,since+5.+1e-12)
        self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        f.r3.update(phase='RL_ACTIVE',motion_command=[.1,0,0]);f.tick(False)
        self.assertEqual(f.m.reason,'BASE_COMMAND_NONZERO')
        # Missing zero acknowledgement cannot wait forever or start scanning.
        f=Fixture();f.start();f.r3.update(phase='RL_ACTIVE',motion_command=[.1,0,0])
        self.at(f,f.m.goal_reached+f.m.cfg['settle']['timeout_s']+.01)
        self.assertEqual(f.m.state,'BLOCKED');self.assertEqual(f.m.reason,'BASE_ZERO_ACK_TIMEOUT')

    def test_settle_5s_and_operator_boundaries(self):
        f=Fixture();f.start();reached=f.m.goal_reached;f.tick(False);since=f.m.stable_since
        self.at(f,since+4.999)
        self.assertEqual(f.m.state,'BASE_SETTLE');self.assertIsNone(f.m.pending)
        self.at(f,since+5.0+1e-12)
        self.assertEqual(f.m.state,'ARM_SCAN_POSE');self.assertGreaterEqual(f.now-reached,5.)
        captured=f.m.scene_anchor
        self.at(f,f.now+.02) # duplicate success must not recapture or restart
        self.assertEqual(f.m.scene_anchor,captured)
        self.assertEqual(sum(e['state']=='ARM_SCAN_POSE' for e in f.m.events),1)
        for metres,reset in ((.019,False),(.020,False),(.021,False)):
            f=Fixture();f.start();f.tick(False);since=f.m.stable_since
            f.base['pose'][2]=metres
            self.at(f,since+3.)
            self.assertEqual(f.m.stable_since>since,reset)
            self.assertEqual(f.m.state,'BASE_SETTLE')
        initial=Rotation.from_euler('xyz',[.02,-.03,.1])
        axis=np.array([1.,2.,3.]);axis/=np.linalg.norm(axis)
        for degrees,reset in ((4.9,False),(5.,False),(5.1,False)):
            f=Fixture();f.base['pose'][3:]=initial.as_quat().tolist()
            f.start();f.tick(False);since=f.m.stable_since
            f.base['pose'][3:]=(initial*Rotation.from_rotvec(axis*np.deg2rad(degrees))).as_quat().tolist()
            self.at(f,since+3.)
            self.assertEqual(f.m.stable_since>since,reset)
        f=Fixture();f.start();f.tick(False);since=f.m.stable_since
        f.base['pose'][0]=.021;self.at(f,since+3.);restart=f.m.stable_since
        self.at(f,restart+4.999);self.assertEqual(f.m.state,'BASE_SETTLE')
        self.at(f,restart+5.+1e-12);self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        f=Fixture();f.start();f.base['tilt_rad']=f.m.cfg['settle']['max_tilt_rad']+.01
        self.at(f,f.m.goal_reached+f.m.cfg['settle']['timeout_s']+.01)
        self.assertEqual(f.m.reason,'BASE_TILT_UNSAFE')

    def test_coherent_anchor_1mm_and_priority(self):
        f=Fixture();f.start();f.tick(False);since=f.m.stable_since
        f.base['pose'][0]=.001
        self.at(f,since+5.+1e-12)
        anchor=copy.deepcopy(f.m.scene_anchor)
        self.assertEqual(anchor['body']['map_body'][0][3],.001)
        self.assertEqual(anchor['body']['mono_ns'],anchor['legs_ns'])
        self.assertEqual(anchor['feedback_ns'],anchor['legs_ns'])
        self.at(f,f.now+.02)
        self.assertEqual(f.m.state,'ARM_SCAN_POSE');self.assertEqual(f.m.scene_anchor,anchor)
        # Intended arm motion differs from scene-anchor q; compare owner q_des.
        result=f.compute(f.m.pending|dict(session_id=f.m.session,navigation_session_id=f.m.nav_session,
            owner_session_id=f.m.owner_session,goal_revision=f.m.revision,owner=f.owner))
        f.m.result(result,f.now)
        f.owner.update(mission_id=f.m.pending['id'],mission_state='EXECUTING',q=[.1]*6+[0.],accepted_target=[.1]*6)
        f.tick(False);self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        f.owner['q'][0]+=.03;f.tick(False);self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        for state in ('CONTROLLED_STOP','EMERGENCY_DAMP'):
            f=Fixture();f.start();f.r3['state']=state;f.tick(False)
            self.assertEqual(f.m.state,'CANCELLED');self.assertIsNone(f.m.pending)
        f=Fixture();f.start();f.r3['fault_latched']=True;f.tick(False)
        self.assertEqual(f.m.state,'CANCELLED')
        f=Fixture();f.start();f.tick(False);f.now+=1
        f.m.step(f.now,int(f.now*1e9));self.assertEqual(f.m.state,'BLOCKED')

    def test_common_base_world_anchor_and_discontinuities(self):
        cfg=copy.deepcopy(CFG);chain=FrameChain(cfg)
        # Reproduce the previous cloud-consistent derivation from CURRENT YAML
        # and pinned URDF without rewriting its historical evidence report.
        nav=__import__('yaml').safe_load(Path(cfg['paths']['navigation']).read_text())
        sensors=nav['sensors'];lio=nav['nodes']['laserMapping']
        processed_raw=xyzrpy(dict(xyz_m=[0.,0.,-sensors['cam_offset_m']],rpy_rad=[0.,sensors['cloud_pitch_rad'],0.]))
        state_processed=np.eye(4);state_processed[:3,:3]=np.asarray(lio['mapping.extrinsic_R']).reshape(3,3)
        state_processed[:3,3]=lio['mapping.extrinsic_T']
        state_vehicle=np.eye(4);state_vehicle[:2,3]=[-nav['robot']['sensor_offset_x_m'],-nav['robot']['sensor_offset_y_m']]
        expected=np.linalg.inv(state_vehicle)@state_processed@processed_raw@np.linalg.inv(fixed_joint(cfg['paths']['full_urdf'],'radar_joint'))
        np.testing.assert_allclose(chain.body(np.eye(4)),expected,atol=1e-12)
        # Check the ACTIVE YAML through the raw-cloud route, without replacing
        # it by the candidate. Nonzero world rotations catch inverse/order errors.
        rng=np.random.default_rng(17002026)
        raw_points=np.c_[rng.uniform(-2,2,(128,3)),np.ones(128)].T
        body_raw=fixed_joint(cfg['paths']['full_urdf'],'radar_joint')
        for _ in range(32):
            world_state=xyzrpy(dict(xyz_m=rng.uniform(-3,3,3),rpy_rad=rng.uniform(-1,1,3)))
            cloud_route=world_state@state_processed@processed_raw@raw_points
            model_route=chain.body(world_state@state_vehicle)@body_raw@raw_points
            np.testing.assert_allclose(model_route,cloud_route,rtol=0,atol=1e-12)
        provider=BodyPoseProvider(cfg)
        vehicle=xyzrpy(dict(xyz_m=[1.,-2.,.4],rpy_rad=[.12,-.08,.5]))
        now=10**9
        body=provider.observe(vehicle,now,now,now,'nav','tf-r1')
        np.testing.assert_allclose(body.map_body,chain.body(vehicle),atol=1e-12)
        from mission.planning import Planning
        planner=Planning.__new__(Planning);planner.cfg=cfg;planner.frames=chain
        planner.grasp={'robot':{'rars01':{'command_rate_hz':100,'position_velocity_limits_rad_s':[1.]*7}}}
        snapshot=SceneSnapshot(body,tuple([0.]*7),tuple([0.,.8,-1.5]*4),now,now,'arm')
        context=dict(body=body,odom_vehicle=vehicle,odom_mono_ns=now,legs=list(snapshot.legs),legs_ns=now,feedback_ns=now,owner_session_id='arm',scene_snapshot=snapshot.data())
        np.testing.assert_allclose(planner.collision_scene(context)['map_body'],body.map_body,atol=1e-12)
        map_grasp=chain.arm(vehicle)@xyzrpy(dict(xyz_m=[.3,.1,.2],rpy_rad=[.1,.2,.3]))
        anchor=WorldGraspAnchor(map_grasp.tolist(),now,now,'nav','tf-r1',4,.9)
        original=copy.deepcopy(anchor.data());local_before=anchor.rebase(chain.arm(vehicle),body,now,180,4,.5)
        moved=vehicle@xyzrpy(dict(xyz_m=[.002,0.,0.],rpy_rad=[.003,.002,-.001]))
        next_body=provider.observe(moved,now+10**7,now+10**7,now+10**7,'nav','tf-r1')
        local_after=anchor.rebase(chain.arm(moved),next_body,now+10**7,180,4,.5)
        self.assertFalse(np.allclose(local_before,local_after))
        np.testing.assert_allclose(chain.arm(moved)@local_after,map_grasp,atol=1e-12)
        self.assertEqual(anchor.data(),original)
        with self.assertRaisesRegex(ValueError,'STALE'):anchor.validate(next_body,now+181*10**9,180,4,.5)
        with self.assertRaisesRegex(ValueError,'TF_STALE'):anchor.validate(next_body,now+10**9,180,4,.5)
        with self.assertRaisesRegex(ValueError,'EPOCH'):anchor.validate(next_body,now+10**7,180,5,.5)
        # Long candidate selection does not age the world target out; validate
        # against a fresh body while preserving the captured world pose/epoch.
        later=now+5*10**9
        fresh_body=provider.observe(moved,later,later,later,'nav','tf-r1')
        anchor.validate(fresh_body,later,180,4,.5)
        self.assertEqual(anchor.data(),original)
        with self.assertRaisesRegex(ValueError,'TF_STALE'):anchor.validate(body,later,180,4,.5)

        for change in ('time','jump','session','revision'):
            p=BodyPoseProvider(cfg);p.observe(vehicle,now,now,now,'nav','r1')
            candidate=vehicle.copy();stamp=now+10**7;session,revision='nav','r1'
            if change=='time':stamp=now-1
            if change=='jump':candidate[0,3]+=cfg['frames']['continuity_translation_m']+.001
            if change=='session':session='new-nav'
            if change=='revision':revision='r2'
            with self.assertRaises(ValueError):p.observe(candidate,stamp,stamp,now+10**7,session,revision)
            self.assertTrue(p.fault)
        with self.assertRaisesRegex(ValueError,'STALE'):BodyPoseProvider(cfg).observe(vehicle,now,now,now+int(cfg['limits']['sensor_age_s']*1e9)+1,'nav','r')
        buffer=SceneBuffer(cfg);buffer.bodies.append(body)
        with self.assertRaises(ValueError):buffer.snapshot(now)
        buffer.arm([0.]*7,now,'arm');buffer.leg([0.,.8,-1.5]*4,now)
        self.assertEqual(buffer.snapshot(now).body,body)
        skew=SceneBuffer(cfg);skew.bodies.append(body)
        skew.arm([0.]*7,now-60_000_000,'arm');skew.leg([0.,.8,-1.5]*4,now)
        with self.assertRaisesRegex(ValueError,'SKEW'):skew.snapshot(now)

    def test_historical_camera_pose_keeps_live_feedback_required(self):
        from dataclasses import replace
        now=10**9;capture=now-600_000_000
        body=BodyPose(capture,capture,'nav','tf',np.eye(4).tolist(),np.eye(4).tolist())
        buffer=SceneBuffer(CFG);buffer.bodies.append(body)
        buffer.arm([.1]*7,capture,'arm');buffer.leg([0.]*12,capture)
        # History alone must not bypass live feedback limits.
        with self.assertRaisesRegex(ValueError,'STALE'):buffer.snapshot(now,capture)
        buffer.bodies.append(replace(body,mono_ns=now,ros_ns=now))
        buffer.arm([.2]*7,now,'arm');buffer.leg([0.]*12,now)
        snapshot=buffer.snapshot(now,capture)
        self.assertEqual(snapshot.q7,tuple([.1]*7))
        self.assertEqual(snapshot.feedback_ns,capture)
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):buffer.snapshot(now,now-int(CFG['execution']['camera_max_age_s']*1e9)-1_000_000)
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):buffer.snapshot(now,now+1)
        # Historical state must belong to the same arm owner and TF epoch.
        buffer.arms[-1]=(now,tuple([.2]*7),'restarted')
        with self.assertRaisesRegex(ValueError,'EPOCH'):buffer.snapshot(now,capture)

    def test_camera_freshness_separate_from_motor_feedback(self):
        from mission.planning import Planning
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        planner=Planning.__new__(Planning);planner.cfg=CFG
        planner.collision=NS(check_deadline=lambda:None);planner.compute_metrics={}
        planner.cfg=copy.deepcopy(CFG);planner.cfg['execution']['camera_admission_candidates']=1
        planner.feedback_ttl=.1;planner.last_frame_mono_ns=0
        timing=dict(color_system_ns=9_500_000_000,depth_system_ns=9_500_000_000)
        planner.camera=NS(get_frame=lambda:(np.zeros((2,2,3)),np.ones((2,2))),last_frame_timing=timing)
        planner.grasp={'camera':{'fps':30}}
        planner.eye=NS(chain=lambda *args:(np.eye(4),np.eye(4)))
        def context(capture,ros):
            return dict(feedback_ns=capture,q=[0.]*6,odom_mono_ns=capture,odom_vehicle=np.eye(4))
        with patch('mission.planning.time.time_ns',return_value=10_000_000_000),patch('mission.planning.time.monotonic_ns',return_value=5_000_000_000):
            _,_,sample=planner.frame(context)
            self.assertEqual(sample['capture_mono_ns'],4_500_000_000)
            with self.assertRaisesRegex(ValueError,'RGBD_STALE_OR_REPEATED'):planner.frame(context)
            planner.last_frame_mono_ns=0;timing['color_system_ns']=timing['depth_system_ns']=10_000_000_000-int(CFG['execution']['camera_max_age_s']*1e9)-1_000_000
            with self.assertRaisesRegex(ValueError,'RGBD_STALE_OR_REPEATED'):planner.frame(context)

    def test_body_source_ignores_unrelated_tf_writers(self):
        # Real ROS adapter with an in-memory TF buffer: no node, DDS or hardware.
        from mission.ros_scene import ROSBodySource
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        now=10**9
        msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='aft_mapped'
        msg.header.stamp.sec=1;msg.pose.pose.orientation.w=1.
        owner=[NS(endpoint_gid=b'odom-owner')];offset=[0.];missing=[False]
        topics=[]
        def writers(topic):
            topics.append(topic)
            return owner if topic=='/state_estimation' else [NS(endpoint_gid=b'unrelated')]
        def lookup(parent,child,at):
            if missing[0]:raise ValueError('missing actual TF')
            tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child
            tf.header.stamp=msg.header.stamp;tf.transform.rotation.w=1.
            if (parent,child)==('aft_mapped','vehicle'):tf.transform.translation.x=offset[0]
            return tf
        source=ROSBodySource.__new__(ROSBodySource);source.cfg=CFG
        source.diagnostics={};source.last_clock_interval=None
        source.node=NS(get_publishers_info_by_topic=writers,get_clock=lambda:NS(now=lambda:NS(nanoseconds=now)))
        source.buffer=NS(lookup_transform=lookup);source.provider=BodyPoseProvider(CFG)
        def read():
            nonlocal now
            now+=10_000_000
            msg.header.stamp.nanosec=now-10**9
            return source.read(msg,'nav')
        with patch('mission.ros_scene.time.monotonic_ns',side_effect=lambda:now):
            first=read()
            self.assertEqual(read().tf_revision,first.tf_revision)
            self.assertEqual(set(topics),{'/state_estimation'})
            # Relevant fixed geometry changes still invalidate the epoch.
            offset[0]=.001
            with self.assertRaisesRegex(ValueError,'EPOCH'):read()
            offset[0]=0.;source.provider=BodyPoseProvider(CFG);read()
            owner[:]=[NS(endpoint_gid=b'restarted-odom')]
            with self.assertRaisesRegex(ValueError,'EPOCH'):read()
            source.provider=BodyPoseProvider(CFG);missing[0]=True
            with self.assertRaisesRegex(ValueError,'missing actual TF'):read()

    def test_bounded_segment_revalidation(self):
        # Fake geometry backend here tests orchestration, not real clearance.
        # collision_alignment_test separately checks real fail-closed admission.
        from mission.planning import Planning
        from types import SimpleNamespace
        f=Fixture();f.feed();scene=copy.deepcopy(f.m.inputs['scene'])
        body=BodyPose(**scene['body']);now=int(f.now*1e9)
        sample=dict(body=body,odom_vehicle=body.map_vehicle,odom_mono_ns=now,
            legs=list(scene['legs']),legs_ns=now,feedback_ns=now,owner_session_id='arm',
            q=[0.]*6,q7=[0.]*7,scene_snapshot=scene)
        planner=Planning.__new__(Planning);planner.cfg=f.m.cfg;planner.frames=f.frames
        planner.grasp={'robot':{'rars01':{'command_rate_hz':100,'position_velocity_limits_rad_s':[1.]*7}}}
        checked=[];geometry=[]
        planner.collision=SimpleNamespace(validated_scene=lambda *args:checked.append('fresh admission'),
            check=lambda points,scene,payload:geometry.append(copy.deepcopy(scene)) or True,
            last_failure={'reason':'COLLISION:arm:leg'})
        original=dict(trajectory=dict(points=[[0.]*7]*2,collision_context=planner.collision_scene(sample),payload_checked=False,collision_checked=True))
        moved=copy.deepcopy(sample);moved['odom_vehicle']=np.asarray(moved['odom_vehicle']).copy();moved['odom_vehicle'][0,3]+=.001
        def coherent(state):
            state['body']=BodyPose(now,now,'nav','synthetic-tf-r1',np.asarray(state['odom_vehicle']).tolist(),planner.frames.body(state['odom_vehicle']).tolist())
            state['scene_snapshot']=SceneSnapshot(state['body'],tuple(state['q7']),tuple(state['legs']),now,now,'arm').data()
            return state
        coherent(moved)
        rebuilds=[]
        def rebuild(request,context):
            state=context(0,0);rebuilds.append(state)
            return dict(trajectory=dict(points=[state['q7']]*2,collision_context=planner.collision_scene(state),payload_checked=False,collision_checked=True))
        planner.execute=rebuild
        result=copy.deepcopy(original)
        planner.validate_result(result,lambda *args:moved,request={'operation':'pregrasp'})
        self.assertEqual(len(rebuilds),0);self.assertEqual(len(checked),1)
        self.assertEqual(geometry,[]) # common rigid transform preserves robot distances
        np.testing.assert_allclose(result['trajectory']['collision_context']['map_body'],planner.frames.body(moved['odom_vehicle']))
        legs=copy.deepcopy(sample);legs['legs'][0]+=.00001;coherent(legs)
        result=copy.deepcopy(original)
        planner.validate_result(result,lambda *args:legs)
        self.assertEqual(geometry,[]) # legs stay frozen at original approval
        self.assertEqual(result['trajectory']['scene_snapshot']['legs'],tuple(legs['legs']))
        self.assertEqual(result['trajectory']['approval_collision_context']['legs'],sample['legs'])
        planner.collision.check=lambda *args:(_ for _ in ()).throw(AssertionError('unexpected geometry pass'))
        planner.validate_result(copy.deepcopy(original),lambda *args:legs)
        planner.collision.check=lambda points,scene,payload:geometry.append(copy.deepcopy(scene)) or True
        planner.cfg['collision']['environment_enabled']=True
        planner.validate_result(copy.deepcopy(original),lambda *args:moved)
        self.assertEqual(len(geometry),1) # enabled world geometry still rechecked
        planner.cfg['collision']['environment_enabled']=False
        arm=copy.deepcopy(sample);arm['q'][0]=arm['q7'][0]=.001;coherent(arm)
        planner.validate_result(copy.deepcopy(original),lambda *args:arm)
        self.assertEqual(rebuilds,[]) # existing owner's 0.02rad start tolerance
        arm['q'][0]=arm['q7'][0]=.03;coherent(arm)
        planner.validate_result(copy.deepcopy(original),lambda *args:arm,request={'operation':'home'})
        self.assertEqual(len(rebuilds),1)
        reads=[]
        def changing_arm(*args):
            reads.append(None);state=copy.deepcopy(sample)
            state['q'][0]=state['q7'][0]=.03*len(reads)
            return coherent(state)
        rebuilds.clear()
        with self.assertRaisesRegex(ValueError,'TRAJECTORY_START_CHANGED'):
            planner.validate_result(copy.deepcopy(original),changing_arm,request={'operation':'pregrasp'})
        self.assertEqual(len(rebuilds),1) # no unbounded chase/replan loop

    def test_confirmed_scan_home_is_revoked_by_arm_motion(self):
        f=Fixture();f.feed()
        self.assertIsNone(f.m.scan_home_plan())
        f.m.done_motion('scan_pose',f.now)
        self.assertEqual(f.m.scan_home_plan()['scan_q'],f.owner['q'][:6])
        f.owner['q'][1]+=.03;f.feed()
        self.assertIsNone(f.m.scan_home_plan())
        f.owner['q'][1]=float('nan');f.feed()
        self.assertIsNone(f.m.scan_home_plan())

    def test_scene_recovers_before_scan_dispatch_and_never_reuses_old_plan(self):
        f=Fixture();f.start()
        for _ in range(500):
            f.tick()
            if f.m.state=='ARM_SEARCH':break
        f.feed();f.m.request('scan_step',f.now)
        request=f.m.effects[-1][1];f.m.drain()
        result=f.compute(request)
        f.m.received['legs']=f.now-1.
        self.assertTrue(f.m.result(result,f.now))
        self.assertIsNone(f.m.pending)
        self.assertEqual(f.m.retry_operation,'scan_step')
        self.assertFalse(any(k=='trajectory' for k,_ in f.m.drain()))
        self.assertTrue(f.m.wait_for_observation(f.now))
        f.now+=.3;f.feed()
        self.assertTrue(f.m.wait_for_observation(f.now))
        effects=f.m.drain();self.assertEqual([k for k,_ in effects],['compute'])
        self.assertGreater(effects[0][1]['id'],request['id'])
        f.m.result(f.compute(effects[0][1]),f.now)
        self.assertEqual([k for k,_ in f.m.drain()],['trajectory'])
        self.assertIsNone(f.m.recovery_since)
        # Once published, a freshness failure must cancel, never retry motion.
        f.m.inputs['perception']['ready']=False
        f.m.inputs['perception']['reason']='SCENE_SNAPSHOT_STALE:live_legs'
        self.assertTrue(f.m.wait_for_observation(f.now))
        self.assertEqual(f.m.state,'BLOCKED')
        self.assertIsNone(f.m.retry_operation)

    def test_three_branches(self):
        for branch,end in [('success','GRASP_PRESUMED_SUCCESS'),('not_found','OBJECT_NOT_FOUND'),('ik_fail','GRASP_UNREACHABLE')]:
            f=Fixture(branch);self.assertEqual(f.run(),end)
            self.assertTrue(f.owner['arm_home_ready'])
            self.assertFalse(f.owner['gripper_hold_confirmed']);self.assertIsNone(f.m.status(f.now)['object_grasped'])
            self.assertEqual(f.owner['gripper_command_active'],branch=='success')
            count=f.commands
            for _ in range(10):f.tick()
            self.assertEqual(f.commands,count)
            print(json.dumps(dict(scenario=branch,states=[e['state'] for e in f.m.events],synthetic_trajectory_plans=f.commands,physical_actuator_commands=0)))
    def test_unverified_grasp_loss_returns_home(self):
        f=Fixture();f.start()
        for _ in range(1000):
            f.tick()
            if f.m.state=='RETREAT':break
        self.assertEqual(f.m.state,'RETREAT');self.assertFalse(f.owner['gripper_hold_confirmed'])
        f.owner['q'][6]=0. # lost or absent object does not change verified state
        for _ in range(100):
            f.tick()
            if f.m.state in TERMINAL:break
        self.assertEqual(f.m.state,'GRASP_PRESUMED_SUCCESS')
        self.assertTrue(f.owner['arm_home_ready']);self.assertIsNone(f.m.status(f.now)['object_grasped'])

    def test_trajectory_preparation_is_not_heartbeat(self):
        f=Fixture();f.start()
        while f.m.state=='BASE_SETTLE':f.tick(False)
        p=f.m.pending
        result=f.compute(p|dict(session_id=f.m.session,navigation_session_id=f.m.nav_session,
            owner_session_id=f.m.owner_session,goal_revision=f.m.revision,owner=f.owner))
        f.now+=2.;f.feed() # slow delivery/decoding with fresh live statuses
        self.assertTrue(f.m.result(result,f.now))
        f.now+=2.;f.tick(False) # owner still preparing the same plan
        self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        self.assertEqual(f.m.pending['kind'],'motion')
        f.owner.update(mission_id=p['id'],mission_state='SUCCEEDED')
        f.tick(False);self.assertEqual(f.m.state,'ARM_SEARCH')
        # A late trajectory cannot revive a cancelled request or changed goal.
        self.assertFalse(f.m.result(result,f.now))
        f=Fixture();f.start()
        while f.m.state=='BASE_SETTLE':f.tick(False)
        p=f.m.pending
        result=f.compute(p|dict(session_id=f.m.session,navigation_session_id=f.m.nav_session,
            owner_session_id=f.m.owner_session,goal_revision=f.m.revision,owner=f.owner))
        self.assertFalse(f.m.result(result|dict(goal_revision=f.m.revision+1),f.now))
        f.r3['state']='CONTROLLED_STOP';f.tick(False)
        self.assertFalse(f.m.result(result,f.now))

    def test_ready_observe_never_commands_grasp_or_scan(self):
        f=Fixture();f.m.ready_observe=True;f.start()
        moves=[];computes=[]
        for _ in range(450):
            effects=f.tick(False)
            for kind,data in effects:
                if kind=='compute':
                    computes.append(data['operation'])
                    if data['operation']=='observe_grasp':
                        result={k:data[k] for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')}
                        result.update(ok=True,observed_ns=int(f.now*1e9),observation=dict(found=True,grasp_count=3))
                        f.m.result(result,f.now)
                    else:f.effect(kind,data)
                elif kind=='trajectory':moves.append(data['kind']);f.effect(kind,data)
        self.assertEqual(moves,['move'])
        self.assertEqual(set(computes),{'scan_pose','observe_grasp'})
        self.assertEqual(f.m.state,'ARM_SEARCH')
        self.assertEqual(f.m.last_observation['grasp_count'],3)
        with self.assertRaisesRegex(ValueError,'FORBIDDEN'):f.m.request('close',f.now)
        f.r3['state']='CONTROLLED_STOP';f.tick(False)
        self.assertEqual(f.m.state,'CANCELLED')

    def test_replay_cancel_and_stale(self):
        f=Fixture();f.nav['goal']=dict(revision=7,stamp_ns=1,state='SUCCEEDED')
        for _ in range(5):f.tick()
        self.assertEqual(f.m.state,'WAIT_GOAL');self.assertEqual(f.commands,0)
        f=Fixture();f.start()
        while not (f.m.pending and f.m.pending['operation']=='detect'):
            f.tick(process=f.m.state!='ARM_SEARCH')
        late=f.compute(f.m.pending|dict(session_id=f.m.session,navigation_session_id=f.m.nav_session,owner_session_id=f.m.owner_session,goal_revision=f.m.revision,owner=f.owner))
        f.r3['state']='CONTROLLED_STOP';effects=f.tick(False)
        self.assertEqual(f.m.state,'CANCELLED');self.assertTrue(any(k=='cancel' for k,_ in effects))
        self.assertFalse(f.m.result(late,f.now));self.assertIsNone(f.m.pending)
        f=Fixture();f.start();f.now+=1
        effects=f.m.step(f.now,int(f.now*1e9));self.assertEqual(f.m.state,'BLOCKED')
        self.assertTrue(any(k=='cancel' for k,_ in effects))
        f=Fixture();f.start();f.owner['owner_session_id']='restarted';f.tick()
        self.assertEqual(f.m.state,'CANCELLED')
    def test_motion_allows_body_and_leg_variations(self):
        for change,delta in (('legs',.00001),('legs',.001),('base',.00001),('base',.001),('jump',CFG['frames']['continuity_translation_m']+.001),('tilt',CFG['settle']['max_tilt_rad']+.01)):
            f=Fixture();f.start()
            while f.m.state=='BASE_SETTLE':f.tick(False)
            # Produce a checked segment but leave it executing in the owner.
            pending=f.m.pending
            result=f.compute(pending|dict(session_id=f.m.session,navigation_session_id=f.m.nav_session,
                owner_session_id=f.m.owner_session,goal_revision=f.m.revision,owner=f.owner))
            f.m.result(result,f.now)
            if change=='legs':f.legs['q'][0]+=delta
            elif change=='tilt':f.base['tilt_rad']=delta
            else:f.base['pose'][0]+=delta
            effects=f.tick(False)
            if change in ('jump','tilt'):
                self.assertEqual(f.m.state,'BLOCKED')
                self.assertEqual(f.m.reason,'SCENE_ANCHOR_DISCONTINUITY')
                continue
            self.assertEqual(f.m.state,'ARM_SCAN_POSE')
            self.assertEqual(f.m.pending['kind'],'motion')
            self.assertFalse(any(kind=='cancel' for kind,_ in effects))

    def test_tf_and_health(self):
        class FK:
            def forward(self,q):
                return pose_matrix([.1,.2,.3,0,0,np.sin(.2),np.cos(.2)])
        eye=EyeInHand(CFG['paths']['hand_eye'],FK(),{'xyz_m':[.3,-.2,.1],'rpy_rad':[.1,.2,.3]},.1,fixed_joint(CFG['paths']['arm_description'],'arm_mount_to_base_link'))
        odom=pose_matrix([1,2,3,0,0,np.sin(.3),np.cos(.3)])
        camera,arm=eye.chain(10**9,950000000,[0]*6,950000000,odom)
        np.testing.assert_allclose(camera,FK().forward([0]*6)@np.load(CFG['paths']['hand_eye'],allow_pickle=False)['T_result'],atol=1e-12)
        from utils.camera_utils import compose_cam_to_base_transform
        compensation={'x':.01,'y':-.02,'z':.03}
        compensated=EyeInHand(CFG['paths']['hand_eye'],FK(),{'xyz_m':[.3,-.2,.1],'rpy_rad':[.1,.2,.3]},.1,
                             fixed_joint(CFG['paths']['arm_description'],'arm_mount_to_base_link'),compensation=compensation)
        cam2,arm2=compensated.chain(10**9,950000000,[0]*6,950000000,odom)
        expected=compose_cam_to_base_transform(FK().forward([0]*6),eye.end_camera,{'calibration':{'hand_eye_compensation_m':compensation}})
        np.testing.assert_allclose(cam2,expected,atol=1e-12);np.testing.assert_allclose(arm2,arm,atol=1e-12)
        g=np.eye(4);g[:3,3]=[.2,.1,.4]
        anchor=eye.anchor(camera,arm,g)
        np.testing.assert_allclose(eye.rebase(arm,anchor),camera@g,atol=1e-12)
        np.testing.assert_allclose(np.linalg.inv(camera)@eye.rebase(arm,anchor),g,atol=1e-12)
        with self.assertRaises(ValueError):eye.chain(10**9,1,[0]*6,950000000,odom)
        with self.assertRaises(ValueError):rigid(np.full((4,4),np.nan))
        h=MissionHealth(.25);h.graph([b'a'],10)
        d=dict(observed_ns=100,session_id='one',navigation_session_id='nav',ready=True,physical=True)
        h.observe(json.dumps(d),b'a',110,'nav');self.assertTrue(h.valid(120,'nav'))
        h.observe(json.dumps(d),b'a',120,'nav');self.assertFalse(h.valid(120,'nav'))
        d['observed_ns']=130;h.observe(json.dumps(d),b'a',140,'nav');self.assertTrue(h.valid(150,'nav'))
        self.assertFalse(h.valid(300000140,'nav'))
        d.update(observed_ns=160,base_locked=True);h.observe(json.dumps(d),b'a',170,'nav');self.assertTrue(h.base_locked)
        h.graph([b'a',b'b'],180);self.assertFalse(h.valid(190,'nav'));self.assertTrue(h.base_locked)
        self.assertEqual(blockers(CFG),[]);self.assertIsInstance(CFG['physical_enabled'],bool)

    def test_arm_geometry_tolerance_preserves_actual_pose_and_leg_limits(self):
        from mission.collision import validate_geometry_joint
        tol=CFG['collision']['arm_joint_measurement_tolerance_rad']
        self.assertAlmostEqual(tol,.025)
        self.assertEqual(tol,CFG['execution']['start_tolerance_rad'])
        for name in ('joint1','joint2','joint3','joint4','joint5','joint6'):
            for q in (-tol,-.000190734863,0.,3.6,3.6+tol):
                self.assertEqual(validate_geometry_joint(name,q,0.,3.6,tol),q)
            for q in (-tol-1e-6,3.6+tol+1e-6,float('nan'),float('inf')):
                with self.assertRaisesRegex(ValueError,'COLLISION_JOINT_LIMIT'):
                    validate_geometry_joint(name,q,0.,3.6,tol)
        for name in ('FR_hip_joint','gripper_left_joint'):
            with self.assertRaisesRegex(ValueError,'COLLISION_JOINT_LIMIT'):
                validate_geometry_joint(name,-.00019,0.,3.6,tol)
        import tempfile,yaml
        raw=yaml.safe_load((ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            for value in (-.1,float('nan'),True,np.deg2rad(2)+.001):
                raw['go2_mission']['collision']['arm_joint_measurement_tolerance_rad']=float(value) if isinstance(value,np.floating) else value
                path.write_text(yaml.safe_dump(raw))
                with self.assertRaisesRegex(ValueError,'arm_joint_measurement_tolerance'):
                    load(path,ROOT)


    def test_measured_undershoot_does_not_become_out_of_bounds_command(self):
        from mission.planning import Planning
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
        planner.grasp={'robot':{'rars01':{'position_velocity_limits_rad_s':[1.]*7,'command_rate_hz':100}}}
        start=np.zeros(7);start[1]=-.00057220459;start[6]=-.0036
        points=planner.samples(start,[np.zeros(6)],1.)
        self.assertGreaterEqual(points[:,1].min(),0.);self.assertGreaterEqual(points[:,6].min(),0.)
        self.assertEqual(start[1],-.00057220459) # measured state not rewritten
        start[1]=-.04
        with self.assertRaisesRegex(ValueError,'OUTSIDE_TOLERANCE'):
            planner.samples(start,[np.zeros(6)],1.)


if __name__=='__main__':unittest.main()
