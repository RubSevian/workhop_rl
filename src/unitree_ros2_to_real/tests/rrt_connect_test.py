"""RRT topology, exact admission, exhaustion and Mission callback regression."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.rrt_connect import connect,SearchBudget,SearchBudgetExceeded,fallback_pregrasp,validate_config
from mission.planning import CandidatePathRejected

CFG=dict(enabled=True,timeout_s=None,max_extensions=2000,range_rad=.35,seed=41,shortcut_checks=8)

class RRT(unittest.TestCase):
    @staticmethod
    def edge(a,b):
        p=np.linspace(a,b,201)
        return not np.any((np.abs(p[:,0])<.15)&(np.abs(p[:,1])<.65))

    def test_detour_exact_endpoints_both_tree_orders_and_repeatability(self):
        a=np.array([-.8,0]);b=np.array([.8,0]);lo=np.array([-1.,-1.]);hi=-lo
        self.assertFalse(self.edge(a,b))
        for seed in range(8):
            path=connect(a,b,lo,hi,self.edge,lambda:None,SearchBudget(None,2000),seed=seed)
            np.testing.assert_array_equal(path[0],a);np.testing.assert_array_equal(path[-1],b)
            self.assertTrue(all(self.edge(x,y) for x,y in zip(path[:-1],path[1:])))
            self.assertTrue(np.all(path>=lo) and np.all(path<=hi))
            repeat=connect(a,b,lo,hi,self.edge,lambda:None,SearchBudget(None,2000),seed=seed)
            np.testing.assert_array_equal(path,repeat)

    def test_unreachable_is_bounded_not_approximate(self):
        def wall(a,b):return a[0]*b[0]>0
        budget=SearchBudget(None,50)
        with self.assertRaisesRegex(SearchBudgetExceeded,'EXTENSION'):
            connect([-1,0],[1,0],[-2,-2],[2,2],wall,lambda:None,budget)
        self.assertEqual(budget.extensions,50)

    def test_endpoint_and_input_rejection(self):
        self.assertIsNone(connect([0],[1],[-2],[2],lambda a,b:False,lambda:None,SearchBudget(None,10)))
        self.assertIsNone(connect([0],[3],[-2],[2],lambda a,b:True,lambda:None,SearchBudget(None,10)))
        with self.assertRaises(ValueError):connect([float('nan')],[1],[-2],[2],lambda a,b:True,lambda:None,SearchBudget(None,10))

    def test_cancel_and_scene_error_propagate(self):
        def cancel():raise RuntimeError('authority_revoked')
        with self.assertRaisesRegex(RuntimeError,'authority_revoked'):
            connect([0],[1],[-2],[2],lambda a,b:True,cancel,SearchBudget(None,10))
        def stale(a,b):raise ValueError('SCENE_STALE')
        with self.assertRaisesRegex(ValueError,'SCENE_STALE'):
            connect([0],[1],[-2],[2],stale,lambda:None,SearchBudget(None,10))

    def test_time_budget(self):
        with patch('mission.rrt_connect.time.monotonic',return_value=0):budget=SearchBudget(1,100)
        with patch('mission.rrt_connect.time.monotonic',return_value=2):
            with self.assertRaisesRegex(SearchBudgetExceeded,'TIME_BUDGET'):budget.check()

    def fixture(self):
        prior=lambda:None
        collision=NS(cancel_check=prior)
        collision.check_deadline=lambda:collision.cancel_check()
        p=NS(collision=collision,_command_joint_bounds=np.array([[-1.,1.]]*6),
             cfg={'collision':{'sample_max_joint_step_rad':.005}})
        p.collision_scene=lambda current:dict(velocity_limits=[1.]*7,command_rate_hz=100)
        seen=[]
        def checked(points,current,payload,reference_ns):
            collision.check_deadline();seen.append(points.copy())
            self.assertEqual(reference_ns,123);self.assertFalse(payload)
            self.assertTrue(np.all(np.abs(np.diff(points,axis=0))*100<=1.+1e-9))
            self.assertTrue(np.all(points[:,6]==.4))
            if not self.edge(points[0,:2],points[-1,:2]):raise CandidatePathRejected('COLLISION:body')
        p.checked=checked
        start=np.array([-.8,0,0,0,0,0,.4])
        candidate=NS(pregrasp_joints=np.array([.8,0,0,0,0,0]),cost=.2)
        return p,start,candidate,seen,prior

    def test_adapter_uses_full_checker_and_final_validation(self):
        p,start,candidate,seen,prior=self.fixture();records=[];final=[]
        def validate(c,path):
            self.assertIs(p.collision.cancel_check,prior)
            final.append(path);return True
        chosen=fallback_pregrasp(p,start,{},123,[candidate],validate,CFG,records)
        self.assertIs(chosen,candidate);self.assertTrue(seen);self.assertEqual(len(final),1)
        np.testing.assert_array_equal(final[0][-1],candidate.pregrasp_joints)
        self.assertEqual(records[-1]['status'],'SELECTED_COLLISION_CHECKED')

    def test_adapter_final_rejection_cannot_execute(self):
        p,start,candidate,_,prior=self.fixture();records=[]
        self.assertIsNone(fallback_pregrasp(p,start,{},123,[candidate],lambda *a:False,CFG,records))
        self.assertEqual(records[-1]['status'],'FINAL_PATH_REJECTED')
        self.assertIs(p.collision.cancel_check,prior)

    def test_adapter_stale_error_restores_hook(self):
        p,start,candidate,_,prior=self.fixture()
        def stale(*args,**kw):raise ValueError('COLLISION_STATE_STALE')
        p.checked=stale
        with self.assertRaisesRegex(ValueError,'COLLISION_STATE_STALE'):
            fallback_pregrasp(p,start,{},123,[candidate],lambda *a:True,CFG,[])
        self.assertIs(p.collision.cancel_check,prior)

    def test_adapter_budget_restores_hook(self):
        p,start,candidate,_,prior=self.fixture();records=[]
        result=fallback_pregrasp(p,start,{},123,[candidate],lambda *a:self.fail('final'),
                                dict(CFG,max_extensions=1),records)
        self.assertIsNone(result);self.assertIs(p.collision.cancel_check,prior)
        self.assertEqual(records[-1]['status'],'EXTENSION_BUDGET')

    def test_config_rejects_invalid_settings(self):
        validate_config(CFG)
        for key,value in [('enabled',1),('range_rad',0),('timeout_s',-1),('max_extensions',True),('seed',-1),('shortcut_checks',-1)]:
            with self.assertRaises(ValueError):validate_config(dict(CFG,**{key:value}))

if __name__=='__main__':unittest.main()
