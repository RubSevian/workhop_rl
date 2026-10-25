"""Stage records survive ordinary failure/cancellation, without ROS or hardware."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.planning_metrics import planning_stage, ik_summary
from mission.leg_plan_trace import LegPlanTrace
from mission.collision import CollisionCancelled


class Tests(unittest.TestCase):
    def test_cancelled_nested_stage_is_flushed_without_final_result(self):
        with tempfile.TemporaryDirectory() as directory:
            trace=LegPlanTrace(directory);metrics={}
            request=dict(id=1,operation='grasp_plan',session_id='test')
            with self.assertRaises(CollisionCancelled):
                with planning_stage(metrics,'selection',trace,request):
                    with planning_stage(metrics,'collision_retreat',trace,request) as stage:
                        stage['collision_profile']={'samples':17}
                        raise CollisionCancelled('COLLISION_CANCELLED:SHUTDOWN')
            trace.close()
            rows=[json.loads(line) for line in trace.path.read_text().splitlines()]
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[0]['stage']['stage'],'collision_retreat')
            self.assertEqual(rows[0]['stage']['collision_profile']['samples'],17)
            self.assertTrue(all(r['stage']['status']=='cancelled' for r in rows))
            self.assertTrue(all(r['stage']['elapsed_s']>=0 and r['stage']['thread_cpu_s']>=0 for r in rows))
            self.assertIsNone(metrics['active_stage'])
            self.assertFalse(trace.error)

    def test_separate_ik_cpu_summary_and_completed_stage(self):
        metrics={}
        with planning_stage(metrics,'graspnet'):
            self.assertEqual(metrics['active_stage']['stage'],'graspnet')
        self.assertEqual(metrics['planning_stages'][0]['status'],'completed')
        self.assertEqual(ik_summary([dict(elapsed_s=2.,thread_cpu_s=.5,attempt_count=3)]),
                         dict(calls=1,cache_hits=0,elapsed_s=2.,thread_cpu_s=.5,attempts=3,
                              fallback_attempts=0,fallback_accepted=0,fallback_elapsed_s=0))

    def test_fallback_summary_separates_attempts_and_acceptance(self):
        summary=ik_summary([dict(attempts=[
            dict(method='dls',elapsed_s=.1),
            dict(method='bounded_least_squares',elapsed_s=.2,accepted=False),
            dict(method='bounded_least_squares',elapsed_s=.3,accepted=True)])])
        self.assertEqual(summary['fallback_attempts'],2)
        self.assertEqual(summary['fallback_accepted'],1)
        self.assertAlmostEqual(summary['fallback_elapsed_s'],.5)


if __name__=='__main__':unittest.main()
