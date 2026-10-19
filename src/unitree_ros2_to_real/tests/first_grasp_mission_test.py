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
            if data['kind']=='close':self.owner.update(q=self.owner['q'][:6]+[.4],gripper_command_active=True,gripper_hold_target=.4)
            self.owner.update(mission_id=data['id'],mission_state='SUCCEEDED',arm_home_ready=all(abs(x)<1e-6 for x in self.owner['q'][:6]))
        elif kind=='cancel':self.owner['mission_state']='CANCELLED'
    def compute(self,data):
        out={k:data[k] for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')}
        out.update(ok=True,observed_ns=int(self.now*1e9))
        op=data['operation']
        if op=='detect':out['found']=self.branch!='not_found'
        elif op=='grasp_plan':
            if self.branch=='ik_fail':out.update(ok=False,reason='IK_FAIL')
            else:out['grasp_plan']={'fixture':'synthetic-IK'}
        else:
            start=data['owner']['q'];end=start.copy()
            if op=='home':end[:6]=[0.]*6
            elif op=='scan_pose':end[:6]=[0.,.2,.2,.1,0,0];end[6]=.8
            elif op=='refine':end[1]+=.01;end[2]+=.01
            elif op in ('scan_step','scan_return'):end[0]=0. if op=='scan_return' else search_angles(CFG['search'])[data['scan_index']]
            elif op!='close':end[1]+=.01
            sys.path.insert(0,str(ROOT/'repos/rars01_graspnet'))
            from rars01_graspnet.real_grasp_trajectory import joint_motion_samples
            points,_=joint_motion_samples(start,end,.1,.01,np.ones(7))
            out['trajectory']=dict(kind='home' if op=='home' else 'close' if op=='close' else 'move',points=[q.tolist() for q in points],collision_checked=True,payload_checked=False)
        return out
    def start(self):
        self.tick();assert self.m.state=='WAIT_GOAL'
        self.nav['goal']=dict(revision=1,stamp_ns=int((self.now+.01)*1e9),state='REQUESTED')
        self.tick();assert self.m.state=='NAVIGATING'
        self.nav['goal']['state']='SUCCEEDED';self.tick()
    def run(self):
        self.start()
        for _ in range(1000):
            self.tick()
            if self.m.state in TERMINAL:return self.m.state
        raise AssertionError(self.m.status(self.now))

class Integration(unittest.TestCase):
    def test_gripper_diagnostics_do_not_gate_mission(self):
        # Neither uncertain nor malformed optional telemetry is a grasp/HOME gate.
        for torque in (0.,2.3,float('nan')):
            with self.subTest(torque=torque):
                f=Fixture();f.owner['torque']=[0.]*6+[torque]
                f.owner['gripper_observation']={'state':'UNKNOWN','error':'disk full'}
                self.assertEqual(f.run(),'MISSION_COMPLETED_UNVERIFIED')
                self.assertTrue(f.owner['arm_home_ready'])

    def test_detection_weak_frames_are_not_mission_faults(self):
        from mission.planning import Planning
        from types import SimpleNamespace
        from unittest.mock import patch
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
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

    def test_search_speed_does_not_limit_grasp_motion(self):
        from mission.planning import Planning
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
        planner.cfg['search']['speed_rad_s']=.01;planner.cfg['execution']['arm_speed_rad_s']=.1
        planner.grasp={'robot':{'rars01':{'position_velocity_limits_rad_s':[1.]*7,'command_rate_hz':100}}}
        start=np.zeros(7);target=np.array([.01,0,0,0,0,0])
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

    def test_refinement_accepts_zero_or_one_joint_change(self):
        from mission.planning import Planning
        from types import SimpleNamespace
        planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG)
        planner.grasp={'robot':{'motion':{},'ready_pose':{'duration':1.}}}
        planner.last_detection=(None,None,None,None,np.array([0.,0.,1.]))
        planner.eye=SimpleNamespace(chain=lambda *args:(np.eye(4),np.eye(4)))
        planner.fk=SimpleNamespace(forward=lambda q:np.eye(4))
        planner.samples=lambda start,targets,*args,**kwargs:np.vstack([start,np.r_[targets[0],start[6]]])
        planner.checked=lambda points,*args:{'points':points}
        sample=dict(q7=[0.]*7,q=[0.]*6,legs=[0.]*12,owner_session_id='arm',
                    body=SimpleNamespace(navigation_session_id='nav'),capture_ns=1,
                    feedback_ns=1,odom_mono_ns=1,odom_vehicle=np.eye(4))
        request=dict(operation='refine',owner={},owner_session_id='arm',navigation_session_id='nav')
        for target in (np.zeros(6),np.array([.01,0,0,0,0,0]),np.array([.01,.01,0,0,0,0])):
            with self.subTest(target=target.tolist()):
                planner.ik=SimpleNamespace(solve=lambda *args,**kwargs:SimpleNamespace(success=True,joints=target))
                result=planner.execute(request,lambda *args:sample)
                np.testing.assert_allclose(result['trajectory']['points'][-1][:6],target)
        from mission.planning import CandidatePathRejected
        checked=planner.checked
        def reject_motion(points,*args):
            if not np.array_equal(points[0],points[-1]):raise CandidatePathRejected('COLLISION:arm:leg')
            return checked(points,*args)
        planner.checked=reject_motion
        planner.ik=SimpleNamespace(solve=lambda *args,**kwargs:SimpleNamespace(success=True,joints=np.array([.01,0,0,0,0,0])))
        result=planner.execute(request,lambda *args:sample)
        np.testing.assert_array_equal(result['trajectory']['points'],np.zeros((2,7)))
        planner.checked=checked
        for success,target in ((False,np.zeros(6)),(True,np.array([.11,0,0,0,0,0]))):
            planner.ik=SimpleNamespace(solve=lambda *args,**kwargs:SimpleNamespace(success=success,joints=target))
            result=planner.execute(request,lambda *args:sample)
            np.testing.assert_allclose(result['trajectory']['points'][-1][:6],target if success else np.zeros(6))

    def test_zero_first_sweep_return_zero_then_home(self):
        f=Fixture('not_found');self.assertEqual(f.run(),'OBJECT_NOT_FOUND')
        operations=[data for kind,data in f.effects if kind=='compute']
        scans=[d for d in operations if d['operation'] in ('scan_pose','scan_step','scan_return','home')]
        self.assertEqual([d['operation'] for d in scans],['scan_pose']+['scan_step']*11+['scan_return','home'])
        self.assertEqual([d['scan_index'] for d in scans[1:12]],list(range(1,12)))
        observations=[d['owner']['q'][0] for d in operations if d['operation']=='detect']
        np.testing.assert_allclose(observations,[0.,-2.5,-2.,-1.5,-1.,-.5,0.,.5,1.,1.5,2.,2.5])
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
        np.testing.assert_allclose(angles[1:],np.linspace(-2.5,2.5,11),atol=1e-12)
        self.assertEqual(search['speed_rad_s'],.1)
        # Exercise the unchanged real interpolation, including its 80% reserve.
        points,duration=joint_motion_samples([angles[1]],[angles[2]],.01,.01,[search['speed_rad_s']])
        q=np.asarray(points)[:,0]
        self.assertLessEqual(np.max(np.abs(np.diff(q)))/.01,search['speed_rad_s'])
        self.assertTrue(np.all(np.diff(q)>=0));self.assertAlmostEqual(q[-1],angles[2])
        self.assertGreater(search['timeout_s'],duration*(len(angles)-1)+search['settle_s']*len(angles))
        _,entry_duration=joint_motion_samples([0.],[angles[1]],.01,.01,[search['speed_rad_s']])
        self.assertLessEqual(entry_duration,CFG['execution']['max_plan_duration_s'])
        self.assertGreater(CFG['execution']['state_timeout_s'],entry_duration+CFG['collision']['check_timeout_s'])
        raw=yaml.safe_load((ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())['go2_mission']
        with tempfile.TemporaryDirectory(prefix='wide-search-config-') as directory:
            path=Path(directory)/'mission.yaml'
            for mode,reason in [('continuous','reserved but not implemented'),('typo','invalid search.mode')]:
                invalid=copy.deepcopy(raw);invalid['search']['mode']=mode;path.write_text(yaml.safe_dump({'go2_mission':invalid}))
                with self.assertRaisesRegex(ValueError,reason):load(path,ROOT)
            invalid=copy.deepcopy(raw);invalid['search']['max_attempts']-=1;path.write_text(yaml.safe_dump({'go2_mission':invalid}))
            with self.assertRaisesRegex(ValueError,'limits/step/attempts disagree'):load(path,ROOT)
            invalid=copy.deepcopy(raw);invalid['search'].update(joint1_min_rad=-3.,joint1_max_rad=3.,max_attempts=14)
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
        self.assertEqual(f.m.state,'BLOCKED');self.assertEqual(f.m.reason,'BASE_SETTLE_TIMEOUT')

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
        for metres,reset in ((.019,False),(.020,False),(.021,True)):
            f=Fixture();f.start();f.tick(False);since=f.m.stable_since
            f.base['pose'][2]=metres
            self.at(f,since+3.)
            self.assertEqual(f.m.stable_since>since,reset)
            self.assertEqual(f.m.state,'BASE_SETTLE')
        initial=Rotation.from_euler('xyz',[.02,-.03,.1])
        axis=np.array([1.,2.,3.]);axis/=np.linalg.norm(axis)
        for degrees,reset in ((4.9,False),(5.,False),(5.1,True)):
            f=Fixture();f.base['pose'][3:]=initial.as_quat().tolist()
            f.start();f.tick(False);since=f.m.stable_since
            f.base['pose'][3:]=(initial*Rotation.from_rotvec(axis*np.deg2rad(degrees))).as_quat().tolist()
            self.at(f,since+3.)
            self.assertEqual(f.m.stable_since>since,reset)
        f=Fixture();f.start();f.tick(False);since=f.m.stable_since
        f.base['pose'][0]=.021;self.at(f,since+3.);restart=f.m.stable_since
        self.at(f,restart+4.999);self.assertEqual(f.m.state,'BASE_SETTLE')
        self.at(f,restart+5.+1e-12);self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        f=Fixture();f.start();f.base['tilt_rad']=.3
        self.at(f,f.m.goal_reached+f.m.cfg['settle']['timeout_s']+.01)
        self.assertEqual(f.m.reason,'BASE_SETTLE_TIMEOUT')

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
            if change=='jump':candidate[0,3]+=.101
            if change=='session':session='new-nav'
            if change=='revision':revision='r2'
            with self.assertRaises(ValueError):p.observe(candidate,stamp,stamp,now+10**7,session,revision)
            self.assertTrue(p.fault)
        with self.assertRaisesRegex(ValueError,'STALE'):BodyPoseProvider(cfg).observe(vehicle,now,now,now+10**9,'nav','r')
        buffer=SceneBuffer(cfg);buffer.bodies.append(body)
        with self.assertRaises(ValueError):buffer.snapshot(now)
        buffer.arm([0.]*7,now,'arm');buffer.leg([0.,.8,-1.5]*4,now)
        self.assertEqual(buffer.snapshot(now).body,body)
        skew=SceneBuffer(cfg);skew.bodies.append(body)
        skew.arm([0.]*7,now-60_000_000,'arm');skew.leg([0.,.8,-1.5]*4,now)
        with self.assertRaisesRegex(ValueError,'SKEW'):skew.snapshot(now)

    def test_historical_camera_pose_keeps_live_feedback_required(self):
        from dataclasses import replace
        now=10**9;capture=now-500_000_000
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
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):buffer.snapshot(now,now-601_000_000)
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):buffer.snapshot(now,now+1)
        # Historical state must belong to the same arm owner and TF epoch.
        buffer.arms[-1]=(now,tuple([.2]*7),'restarted')
        with self.assertRaisesRegex(ValueError,'EPOCH'):buffer.snapshot(now,capture)

    def test_camera_freshness_separate_from_motor_feedback(self):
        from mission.planning import Planning
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        planner=Planning.__new__(Planning);planner.cfg=CFG
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
            planner.last_frame_mono_ns=0;timing['color_system_ns']=timing['depth_system_ns']=9_399_000_000
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
        original=dict(trajectory=dict(points=[[0.]*7]*2,collision_context=planner.collision_scene(sample),payload_checked=False))
        moved=copy.deepcopy(sample);moved['odom_vehicle']=np.asarray(moved['odom_vehicle']).copy();moved['odom_vehicle'][0,3]+=.001
        def coherent(state):
            state['body']=BodyPose(now,now,'nav','synthetic-tf-r1',np.asarray(state['odom_vehicle']).tolist(),planner.frames.body(state['odom_vehicle']).tolist())
            state['scene_snapshot']=SceneSnapshot(state['body'],tuple(state['q7']),tuple(state['legs']),now,now,'arm').data()
            return state
        coherent(moved)
        rebuilds=[]
        def rebuild(request,context):
            state=context(0,0);rebuilds.append(state)
            return dict(trajectory=dict(points=[state['q7']]*2,collision_context=planner.collision_scene(state),payload_checked=False))
        planner.execute=rebuild
        result=copy.deepcopy(original)
        planner.validate_result(result,lambda *args:moved,request={'operation':'pregrasp'})
        self.assertEqual(len(rebuilds),0);self.assertEqual(len(checked),1)
        self.assertEqual(geometry,[]) # common rigid transform preserves robot distances
        np.testing.assert_allclose(result['trajectory']['collision_context']['map_body'],planner.frames.body(moved['odom_vehicle']))
        legs=copy.deepcopy(sample);legs['legs'][0]+=.00001;coherent(legs)
        result=copy.deepcopy(original)
        planner.validate_result(result,lambda *args:legs)
        self.assertEqual(len(geometry),1)
        self.assertEqual(geometry[-1]['legs'],legs['legs'])
        self.assertEqual(result['trajectory']['scene_snapshot']['legs'],tuple(legs['legs']))
        planner.collision.check=lambda *args:False
        with self.assertRaisesRegex(ValueError,'COLLISION:arm:leg'):
            planner.validate_result(copy.deepcopy(original),lambda *args:legs)
        planner.collision.check=lambda points,scene,payload:geometry.append(copy.deepcopy(scene)) or True
        planner.cfg['collision']['environment_enabled']=True
        planner.validate_result(copy.deepcopy(original),lambda *args:moved)
        self.assertEqual(len(geometry),2) # enabled world geometry still rechecked
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

    def test_three_branches(self):
        for branch,end in [('success','MISSION_COMPLETED_UNVERIFIED'),('not_found','OBJECT_NOT_FOUND'),('ik_fail','GRASP_UNREACHABLE')]:
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
        self.assertEqual(f.m.state,'MISSION_COMPLETED_UNVERIFIED')
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
        for change,delta in (('legs',.00001),('legs',.001),('base',.00001),('base',.001),('jump',.101),('tilt',.3)):
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

if __name__=='__main__':unittest.main()
