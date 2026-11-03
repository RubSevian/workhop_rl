"""Retry triggers, common budget, cancellation and immutable scene regressions."""
import copy
import tempfile
import yaml
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.grasp_generations import run_generations,unseen_grasps,GenerationBudgetExceeded
import architecture_planning_contract_test as contracts

class Generations(unittest.TestCase):
    def run_attempts(self,errors):
        calls=[];records=[]
        def attempt(remaining):
            calls.append(remaining())
            outcome=errors[len(calls)-1]
            if isinstance(outcome,Exception):raise outcome
            return outcome
        result=run_generations(attempt,lambda:None,lambda:None,3,90,records)
        return result,calls,records
    def test_stop_on_first_success(self):
        result,calls,records=self.run_attempts([{'selected':True}])
        self.assertTrue(result['selected']);self.assertEqual(len(calls),1)
    def test_retry_ik_and_geometry_then_success(self):
        result,calls,records=self.run_attempts([ValueError('IK_FAIL'),ValueError('NO_COLLISION_FREE_CANDIDATE: pair'),{'selected':True}])
        self.assertEqual(len(calls),3)
        self.assertEqual([r['status'] for r in records],['REJECTED','REJECTED','SELECTED'])
    def test_no_more_than_three(self):
        records=[];calls=[]
        def fail(left):calls.append(1);raise ValueError('NO_CANDIDATES')
        with self.assertRaisesRegex(ValueError,'NO_CANDIDATES'):
            run_generations(fail,lambda:None,lambda:None,3,90,records)
        self.assertEqual(len(calls),3)
    def test_no_retry_on_stale_scene_epoch_or_authority(self):
        for reason in ('SCENE_SNAPSHOT_STALE:body','WORLD_ANCHOR_EPOCH_CHANGED','authority_revoked','GRASP_GENERATION_ARM_MOVED'):
            calls=[]
            def fail(left):calls.append(1);raise ValueError(reason)
            with self.assertRaisesRegex(ValueError,reason):run_generations(fail,lambda:None,lambda:None,3,90,[])
            self.assertEqual(len(calls),1)
    def test_shared_budget_does_not_restart(self):
        clock=[0.];records=[];calls=[]
        def fail(left):
            calls.append(left());clock[0]+=6;raise ValueError('IK_FAIL')
        with patch('mission.grasp_generations.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(ValueError,'IK_FAIL'):
                run_generations(fail,lambda:None,lambda:None,3,10,records)
        self.assertEqual(calls,[10,4]);self.assertEqual(records[-1]['status'],'BUDGET_EXHAUSTED')
    def test_null_budget_keeps_count_and_worker_budget_callback_unbounded(self):
        calls=[]
        def attempt(remaining):
            calls.append(remaining())
            if len(calls)<3:raise ValueError('NO_CANDIDATES')
            return 'selected'
        self.assertEqual(run_generations(attempt,lambda:None,lambda:None,3,None,[]),'selected')
        self.assertEqual(calls,[float('inf')]*3)

    def test_cancel_checked_between_attempts(self):
        checks=[]
        def cancel():checks.append(1);raise RuntimeError('CANCELLED')
        with self.assertRaisesRegex(RuntimeError,'CANCELLED'):
            run_generations(lambda remaining:self.fail('inference after cancel'),lambda:None,cancel,3,90,[])
    def test_skip_exact_duplicates_but_keep_nearby_new_orientation(self):
        a=np.zeros(17);a[1:4]=[.1,.02,.04];a[4:13]=np.eye(3).ravel();a[13:16]=[0,0,.7]
        b=a.copy();b[0]=.99
        c=a.copy();theta=.1;c[4:13]=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]).ravel()
        d=a.copy();d[3]=.05
        novel=unseen_grasps([b,c,d],[a]);self.assertEqual(len(novel),2)
        np.testing.assert_allclose(novel,[c,d])

    def test_float32_rotation_duplicate_does_not_need_exact_orthogonality(self):
        row=np.zeros(17);row[4:13]=(np.eye(3)*.99999994).ravel()
        self.assertEqual(len(unseen_grasps([row],[row])),0)

    def test_generation_config_accepts_null_and_rejects_invalid_limits(self):
        from first_grasp_mission_test import ROOT
        from mission.config import load
        raw=yaml.safe_load((ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())
        self.assertIsNone(load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)['compute']['grasp_generation_budget_s'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            for key,values in [('grasp_generation_count',[0,4,True,1.5]),('grasp_generation_budget_s',[0,-1,True,float('nan')])]:
                for value in values:
                    changed=copy.deepcopy(raw);changed['go2_mission']['compute'][key]=value
                    path.write_text(yaml.safe_dump(changed))
                    with self.assertRaisesRegex(ValueError,key):load(path,ROOT)

class Integration(unittest.TestCase):
    def planner(self):
        p,s=contracts.PlanningContracts().setup_planner();p.compute_metrics={}
        p.cfg['compute'].update(grasp_generation_count=3,grasp_generation_budget_s=90)
        p.grasp['graspnet']={'worker_timeout_s':30}
        return p,s
    def test_same_observation_and_prior_cancel_hook_preserved(self):
        p,s=self.planner();observation=(1,2,3,{'body':s['body']},5);calls=[];hooks=[]
        original=lambda:hooks.append(1)
        p.collision.cancel_check=original
        def attempt(request,context,detection,op,trace,progress,tested,tested_geometry,remaining):
            self.assertIs(detection,observation);remaining();p.collision.cancel_check()
            calls.append(len(tested));tested.append([0]*17)
            if len(calls)<3:raise ValueError('IK_FAIL')
            return {'selected':True}
        p._grasp_generation=attempt
        self.assertTrue(p.grasp_generations({},lambda *a:copy.deepcopy(s),observation,'grasp_plan',None,None)['selected'])
        self.assertEqual(calls,[0,1,2]);self.assertIs(p.collision.cancel_check,original)
    def test_arm_movement_prevents_next_generation(self):
        p,s=self.planner();observation=(1,2,3,{'body':s['body']},5);calls=[]
        def attempt(*args):calls.append(1);s['q'][2]+=.2;raise ValueError('IK_FAIL')
        p._grasp_generation=attempt
        with self.assertRaisesRegex(ValueError,'GRASP_GENERATION_ARM_MOVED'):
            p.grasp_generations({},lambda *a:copy.deepcopy(s),observation,'grasp_plan',None,None)
        self.assertEqual(len(calls),1)
    def test_scene_fault_prevents_next_generation(self):
        p,s=self.planner();observation=(1,2,3,{'body':s['body']},5);calls=[]
        def attempt(*args):calls.append(1);p.collision.validated_scene=lambda *a,**k:(_ for _ in ()).throw(ValueError('SCENE_SNAPSHOT_STALE:legs'));raise ValueError('NO_CANDIDATES')
        p._grasp_generation=attempt
        with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:legs'):
            p.grasp_generations({},lambda *a:copy.deepcopy(s),observation,'grasp_plan',None,None)
        self.assertEqual(len(calls),1)

if __name__=='__main__':unittest.main()
