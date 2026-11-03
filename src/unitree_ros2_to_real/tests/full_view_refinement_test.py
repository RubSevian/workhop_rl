"""Framing, cancellation, path admission and bounded-search regressions. No robot IO."""
import copy
import sys
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.refinement import frame_bounds,select_full_view,distant_view_candidates
import architecture_planning_contract_test as contracts
from first_grasp_mission_test import Fixture

K=np.array([[100.,0,100],[0,100.,100],[0,0,1.]])
def pose(q):
    a,b=q[0],q[3]
    ry=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    rx=np.array([[1,0,0],[0,np.cos(b),-np.sin(b)],[0,np.sin(b),np.cos(b)]])
    T=np.eye(4);T[:3,:3]=ry@rx;return T

def solve(corners,cancel=lambda:None,**kwargs):
    return select_full_view(np.zeros(6),pose,np.array(corners),K,(200,200),
        -np.ones(6),np.ones(6),(-.5,.5),0.,.5,.35,.05,.05,cancel,**kwargs)

class Framing(unittest.TestCase):
    def test_horizontal_and_vertical_centering(self):
        q,d=solve([[.5,.6,1],[.9,.6,1],[.9,1.,1],[.5,1.,1]])
        self.assertFalse(d['before']['predicted_full_view'])
        self.assertTrue(d['after']['predicted_full_view'])
        self.assertLess(d['after']['center_error_fraction'],d['before']['center_error_fraction'])
        self.assertLessEqual(abs(q[0]),.5);self.assertLessEqual(abs(q[3]),.35)
        np.testing.assert_array_equal(q[[1,2,4,5]],np.zeros(4))
    def test_cancel_propagates(self):
        def cancel():raise ValueError('CANCELLED')
        with self.assertRaisesRegex(ValueError,'CANCELLED'):solve([[0,0,1],[1,0,1],[1,1,1],[0,1,1]],cancel)
    def test_unseen_extent_requires_new_frame(self):
        mask=np.zeros((200,200),bool);mask[90:200,70:130]=True
        v=frame_bounds(mask,[70,90,129,199],mask.shape,.05)
        self.assertFalse(v['full_view']);self.assertTrue(v['clipped'])
        mask[:]=False;mask[40:160,70:130]=True
        self.assertTrue(frame_bounds(mask,[70,40,129,159],mask.shape,.05)['full_view'])
        # Mask extends beyond a misleadingly small detection box.
        mask[199,100]=True
        self.assertFalse(frame_bounds(mask,[70,40,129,159],mask.shape,.05)['full_view'])
    def test_impossible_field_of_view_not_reported_full(self):
        _,d=solve([[-4,-4,1],[4,-4,1],[4,4,1],[-4,4,1]])
        self.assertFalse(d['after']['predicted_full_view'])
    def test_absolute_pitch_and_translation_bounds(self):
        q=np.zeros(6);q[3]=-.34
        def translated(q):
            T=pose(q);T[0,3]=q[3];return T
        result,d=select_full_view(q,translated,np.array([[.5,.6,1],[.9,.6,1],[.9,1,1],[.5,1,1]]),K,(200,200),
            -np.ones(6),np.ones(6),(-.5,.5),0.,.5,.35,.02,.05)
        self.assertLessEqual(abs(result[3]),.35)
        self.assertLessEqual(d['after']['camera_translation_m'],.02+1e-9)

    def test_runtime_can_exceed_old_pitch_and_camera_displacement_caps(self):
        def translated(q):
            T=pose(q);T[0,3]=q[3]*.2;return T
        q,d=select_full_view(np.zeros(6),translated,np.array([[.2,1.2,1],[.6,1.2,1],[.6,1.6,1],[.2,1.6,1]]),K,(200,200),
            -np.ones(6)*2,np.ones(6)*2,(-.5,.5),0.,.5,None,None,.05)
        self.assertTrue(d['after']['predicted_full_view'])
        self.assertGreater(abs(q[3]),.35)
        self.assertGreater(d['after']['camera_translation_m'],.05)

    def test_farther_camera_uses_ik_and_reduces_projected_size(self):
        def camera(q):
            T=np.eye(4);T[2,3]=q[1];return T
        calls=[]
        def ik(T,reference):
            calls.append(T.copy());q=reference.copy();q[1]=T[2,3];return q
        corners=np.array([[-1.2,-1.2,1],[1.2,-1.2,1],[1.2,1.2,1],[-1.2,1.2,1]])
        choices=distant_view_candidates(np.zeros(6),camera,ik,corners,K,(200,200),-np.ones(6)*3,np.ones(6)*3,(-.5,.5),.5,.05,3.)
        self.assertTrue(choices);self.assertLessEqual(len(calls),3)
        for q,d in choices:
            self.assertTrue(d['predicted_full_view']);self.assertGreater(d['camera_distance_after_m'],d['camera_distance_before_m'])
            self.assertGreater(d['camera_translation_m'],.05)

    def test_farther_camera_rejects_unreachable_and_outside_yaw(self):
        camera=lambda q:np.eye(4)
        corners=np.array([[-1.2,-1.2,1],[1.2,-1.2,1],[1.2,1.2,1],[-1.2,1.2,1]])
        for ik in (lambda *a:None,lambda T,q:np.ones(6)*10):
            self.assertEqual(distant_view_candidates(np.zeros(6),camera,ik,corners,K,(200,200),-np.ones(6),np.ones(6),(-.5,.5),.5,.05,3.),[])

    def test_farther_camera_cancel_stops_ik(self):
        def cancel():raise RuntimeError('CANCELLED')
        with self.assertRaisesRegex(RuntimeError,'CANCELLED'):
            distant_view_candidates(np.zeros(6),lambda q:np.eye(4),lambda *a:self.fail('IK after cancel'),
                np.array([[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]),K,(200,200),-np.ones(6),np.ones(6),(-.5,.5),.5,.05,3.,cancel)

class Admission(unittest.TestCase):
    def planner(self):
        p,s=contracts.PlanningContracts().setup_planner();p.compute_metrics={}
        p.cfg['refinement'].update(full_view_enabled=True,max_pitch_rad=.35,max_camera_translation_m=.05,frame_margin_fraction=.05)
        p.scan_joints=np.array(s['q7'][:6]);p.angles=[-2.5,2.5]
        p.grasp['robot']['ready_pose']={'duration':4.5}
        p.grasp['graspnet']={'min_depth':.05,'max_depth':1.}
        p.camera=NS(K=K)
        q0=np.array(s['q7'][:6])
        p.eye=NS(chain=lambda a,b,q,*args:(pose(q-q0),np.eye(4)))
        p.ik=NS(joint_limits=lambda:(-np.ones(6)*3,np.ones(6)*3))
        p.samples=lambda start,targets,*a,**k:np.vstack([start,np.r_[targets[0],start[6]]])
        depth=np.full((200,200),700,dtype=np.uint16);mask=np.zeros(depth.shape,bool);mask[150:200,150:190]=True
        det=NS(mask=mask,bbox_xyxy=[150,150,189,199])
        p.last_detection=(np.zeros((200,200,3)),depth,det,dict(odom_arm=np.eye(4),arm_camera=np.eye(4)),np.array([.5,.6,.7]))
        return p,s
    def test_vertical_refine_requires_checked_path(self):
        p,s=self.planner();calls=[]
        def checked(points,*a,**k):
            calls.append(points);return dict(points=points.tolist(),collision_checked=True)
        p.checked=checked
        request=dict(operation='refine',owner={},owner_session_id='arm',navigation_session_id='nav')
        t=p.execute(request,lambda *a:copy.deepcopy(s))['trajectory']
        self.assertTrue(t['collision_checked']);self.assertNotIn('preset_operation',t)
        self.assertEqual(len(calls),1);self.assertNotEqual(calls[0][0,3],calls[0][-1,3])
        def reject(*a,**k):raise ValueError('COLLISION_REJECTED')
        p.checked=reject
        with self.assertRaisesRegex(ValueError,'COLLISION_REJECTED'):p.execute(request,lambda *a:copy.deepcopy(s))
        from mission.planning import CandidatePathRejected
        def geometry_reject(*a,**k):raise CandidatePathRejected('COLLISION:arm')
        p.checked=geometry_reject
        with self.assertRaisesRegex(ValueError,'TARGET_VIEW_UNREACHABLE'):p.execute(request,lambda *a:copy.deepcopy(s))
    def test_retreat_tries_next_ik_route_only_after_geometry_rejection(self):
        from unittest.mock import patch
        from mission.planning import CandidatePathRejected
        p,s=self.planner();calls=[]
        q=np.array(s['q7'][:6]);q[1]+=.1
        choices=[(q.copy(),{'mode':'farther_camera_ik'}),(q.copy()+.01,{'mode':'farther_camera_ik'})]
        def checked(points,*a,**k):
            calls.append(points)
            if len(calls)==1:raise CandidatePathRejected('COLLISION:arm')
            return dict(points=points.tolist(),collision_checked=True)
        p.checked=checked
        request=dict(operation='refine',view_retreat=True,owner={},owner_session_id='arm',navigation_session_id='nav')
        with patch('mission.refinement.distant_view_candidates',return_value=choices):
            t=p.execute(request,lambda *a:copy.deepcopy(s))['trajectory']
        self.assertEqual(len(calls),2);self.assertTrue(t['collision_checked'])
        p.checked=lambda *a,**k:(_ for _ in ()).throw(ValueError('SCENE_SNAPSHOT_STALE:legs'))
        with patch('mission.refinement.distant_view_candidates',return_value=choices):
            with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:legs'):
                p.execute(request,lambda *a:copy.deepcopy(s))

    def test_next_scan_restores_pitch_with_checked_path(self):
        p,s=self.planner();s['q7'][3]-=.2
        calls=[]
        p.checked=lambda points,*a,**k:(calls.append(points) or dict(points=points.tolist(),collision_checked=True))
        request=dict(operation='scan_step',scan_index=1,owner={},owner_session_id='arm',navigation_session_id='nav')
        t=p.execute(request,lambda *a:copy.deepcopy(s))['trajectory']
        self.assertTrue(t['collision_checked']);self.assertAlmostEqual(calls[0][-1,3],p.scan_joints[3])
    def test_next_scan_restores_repositioned_arm_joints(self):
        p,s=self.planner();s['q7'][1]+=.2;s['q7'][2]+=.2;s['q7'][4]+=.1
        p.checked=lambda points,*a,**k:dict(points=points.tolist(),collision_checked=True)
        request=dict(operation='scan_step',scan_index=1,owner={},owner_session_id='arm',navigation_session_id='nav')
        t=p.execute(request,lambda *a:copy.deepcopy(s))['trajectory']
        np.testing.assert_allclose(t['points'][-1][1:6],p.scan_joints[1:6])
        self.assertTrue(t['collision_checked'])

    def test_home_from_repositioned_scan_requires_checked_path(self):
        p,s=self.planner();s['q7'][1]+=.2;s['q']=s['q7'][:6]
        calls=[]
        p.checked=lambda points,*a,**k:(calls.append(points) or dict(points=points.tolist(),collision_checked=True))
        request=dict(operation='home',owner={},owner_session_id='arm',navigation_session_id='nav',home_from_scan=True,grasp_plan=dict(scan_q=s['q']))
        t=p.execute(request,lambda *a:copy.deepcopy(s))['trajectory']
        self.assertTrue(t['collision_checked']);self.assertEqual(len(calls),1)

    def test_clipped_fresh_frame_never_calls_graspnet(self):
        p,s=self.planner();p.resume_camera=lambda:None;p.detect=lambda *a:p.last_detection
        p.worker=NS(infer=lambda *a,**k:self.fail('GraspNet called for clipped target'))
        request=dict(id=1,operation='grasp_plan',owner={},owner_session_id='arm',navigation_session_id='nav',session_id='mission',goal_revision=1)
        with self.assertRaisesRegex(ValueError,'TARGET_VIEW_INCOMPLETE'):p.execute(request,lambda *a:copy.deepcopy(s))
    def test_retreat_used_in_initial_refine_is_counted(self):
        f=Fixture();f.start()
        for _ in range(1000):
            if f.m.state=='VIEW_REFINEMENT' and f.m.pending and f.m.pending['kind']=='compute':break
            f.tick()
        p=f.m.pending;self.assertEqual(p['operation'],'refine')
        result=f.compute(dict(p,session_id=f.m.session,navigation_session_id=f.m.nav_session,
            owner_session_id=f.m.owner_session,goal_revision=f.m.revision,owner=copy.deepcopy(f.owner)))
        result['view_retreat_used']=True
        self.assertTrue(f.m.result(result,f.now));self.assertTrue(f.m.view_retreat_attempted)

    def test_incomplete_view_gets_one_retreat_then_home(self):
        f=Fixture();f.start()
        for _ in range(1000):
            if f.m.state=='PLAN_GRASP':break
            f.tick()
        for _ in range(100):
            f.tick(False)
            if f.m.pending:break
        def fail(reason):
            p=f.m.pending
            result=dict(id=p['id'],session_id=f.m.session,navigation_session_id=f.m.nav_session,
                        owner_session_id=f.m.owner_session,goal_revision=f.m.revision,observed_ns=int(f.now*1e9),ok=False,reason=reason)
            self.assertTrue(f.m.result(result,f.now))
        fail('TARGET_VIEW_INCOMPLETE')
        self.assertEqual(f.m.state,'VIEW_REFINEMENT')
        self.assertTrue(f.m.view_retreat_attempted)
        self.assertEqual(f.m.pending['operation'],'refine')
        self.assertTrue(f.m.effects[-1][1]['view_retreat'])
        fail('TARGET_VIEW_UNREACHABLE')
        self.assertEqual(f.m.state,'RETURN_HOME')
        self.assertEqual(f.m.pending['operation'],'home')
        self.assertEqual(f.m.result_after_home,'GRASP_UNREACHABLE')

if __name__=='__main__':unittest.main()
