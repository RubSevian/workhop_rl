"""Framing, cancellation, path admission and bounded-search regressions. No robot IO."""
import copy
import sys
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.refinement import frame_bounds,select_full_view
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
        with self.assertRaisesRegex(ValueError,'REFINE_PATH_REJECTED'):p.execute(request,lambda *a:copy.deepcopy(s))
    def test_next_scan_restores_pitch_with_checked_path(self):
        p,s=self.planner();s['q7'][3]-=.2
        calls=[]
        p.checked=lambda points,*a,**k:(calls.append(points) or dict(points=points.tolist(),collision_checked=True))
        request=dict(operation='scan_step',scan_index=1,owner={},owner_session_id='arm',navigation_session_id='nav')
        t=p.execute(request,lambda *a:copy.deepcopy(s))['trajectory']
        self.assertTrue(t['collision_checked']);self.assertAlmostEqual(calls[0][-1,3],p.scan_joints[3])
    def test_clipped_fresh_frame_never_calls_graspnet(self):
        p,s=self.planner();p.resume_camera=lambda:None;p.detect=lambda *a:p.last_detection
        p.worker=NS(infer=lambda *a,**k:self.fail('GraspNet called for clipped target'))
        request=dict(id=1,operation='grasp_plan',owner={},owner_session_id='arm',navigation_session_id='nav',session_id='mission',goal_revision=1)
        with self.assertRaisesRegex(ValueError,'TARGET_VIEW_INCOMPLETE'):p.execute(request,lambda *a:copy.deepcopy(s))
    def test_incomplete_view_resumes_existing_bounded_search(self):
        f=Fixture();f.start()
        for _ in range(1000):
            if f.m.state=='PLAN_GRASP':break
            f.tick()
        for _ in range(100):
            f.tick(False)
            if f.m.pending:break
        p=f.m.pending
        result=dict(id=p['id'],session_id=f.m.session,navigation_session_id=f.m.nav_session,
                    owner_session_id=f.m.owner_session,goal_revision=f.m.revision,observed_ns=int(f.now*1e9),ok=False,reason='TARGET_VIEW_INCOMPLETE')
        self.assertTrue(f.m.result(result,f.now));self.assertEqual(f.m.state,'ARM_SEARCH')
        self.assertEqual(f.m.pending['operation'],'scan_step')
        f.m.scan_index=f.m.cfg['search']['max_attempts']-1;f.m.next_search(f.now)
        self.assertEqual(f.m.state,'RETURN_HOME')

if __name__=='__main__':unittest.main()
