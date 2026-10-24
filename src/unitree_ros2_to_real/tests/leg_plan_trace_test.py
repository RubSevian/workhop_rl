"""Diagnostic trace only; no ROS or physical device."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.leg_plan_trace import LegPlanTrace


class Tests(unittest.TestCase):
    def test_frozen_baseline_signed_delta_execution_and_next_operation(self):
        with tempfile.TemporaryDirectory() as root:
            trace=LegPlanTrace(root)
            request=dict(id=5,operation='scan_pose',session_id='mission')
            sample=dict(legs=[.1]*12,legs_ns=10,owner_session_id='owner',
                body=NS(mono_ns=9,navigation_session_id='nav',tf_revision='tf'))
            trace.begin(request,sample)
            sample['legs'][0]=9. # Must not mutate the stored baseline.
            live=[.2,-.1]+[.1]*10
            trace.observe(live,11,busy=True,r3_phase='RL_ZERO')
            trace.observe(live,11,busy=False,owner_status={'mission_state':'EXECUTING'})
            # Reused paths retain their approved scene, not the new live legs.
            request['id']=6
            trace.begin(request,sample,dict(legs=[0.]*12,legs_ns=3,body={'mono_ns':2}))
            trace.observe(live,12)
            trace.result(request,dict(ok=True,trajectory={'points':[[0.]*7]*2}),{'ik_elapsed_s':1.})
            trace.close()
            self.assertFalse(trace.thread.is_alive());self.assertEqual(trace.error,'')
            rows=[json.loads(line) for line in trace.path.read_text().splitlines()]
            self.assertEqual(len(rows),6)
            first=rows[1]
            self.assertEqual(first['q_legs_plan'],[.1]*12)
            self.assertEqual(first['q_legs_live'],live)
            self.assertAlmostEqual(first['delta_q'][0],.1)
            self.assertAlmostEqual(first['delta_q'][1],-.2)
            self.assertAlmostEqual(first['max_abs_delta_q'],.2)
            self.assertEqual(rows[2]['live_legs_ns'],11) # Never renew source stamps.
            self.assertFalse(rows[2]['busy'])
            self.assertEqual(rows[-1]['operation_id'],6)
            self.assertEqual(rows[-2]['plan_legs_ns'],3)
            self.assertEqual(rows[-2]['q_legs_plan'],[0.]*12)
            self.assertEqual(rows[-1]['record_type'],'compute_result')
            self.assertEqual(len(rows[-1]['trajectory']['points']),2)

    def test_ik_details_survive_removal_from_periodic_status(self):
        with tempfile.TemporaryDirectory() as root:
            trace=LegPlanTrace(root)
            metrics={'ik_calls':[{'target_pose6d':[0.]*6,'attempt_count':1,'success':False}],
                     'ik_transform_audit':{'tcp_frame':'End_link'}}
            trace.result(dict(id=7,operation='grasp_plan',session_id='session'),
                         dict(ok=False,reason='IK_REJECTED'),metrics)
            metrics.pop('ik_calls')
            trace.close()
            row=json.loads(trace.path.read_text())
            self.assertEqual(row['metrics']['ik_calls'][0]['attempt_count'],1)
            self.assertEqual(row['operation_id'],7)
            self.assertEqual(trace.error,'')

    def test_file_error_is_diagnostic_only(self):
        with tempfile.TemporaryDirectory() as root:
            blocker=Path(root)/'file';blocker.write_text('not a directory')
            trace=LegPlanTrace(blocker)
            trace.thread.join(2.)
            self.assertTrue(trace.error)
            trace.observe([0.]*12,1)
            trace.close()


if __name__=='__main__':unittest.main()
