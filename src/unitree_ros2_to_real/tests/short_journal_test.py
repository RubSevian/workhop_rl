"""Short event journal without ROS, devices, image payloads or mission effects."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.short_journal import ShortJournal,compute_summary,make_short_journal


class Tests(unittest.TestCase):
    def test_ten_thousand_unchanged_statuses_write_one_transition(self):
        with tempfile.TemporaryDirectory() as directory:
            journal=ShortJournal(directory,'mission','test')
            engine=NS(pending=dict(id=1,kind='compute',operation='grasp_plan'),
                state='PLAN_GRASP',reason='',entered=1.,revision=2,
                inputs=dict(owner=dict(grasp_assessment='UNKNOWN'),perception={}))
            for index in range(10000):journal.transition(engine,1.+index/50.)
            engine.state='BLOCKED';engine.reason='CAMERA_FRAME_STALE';engine.pending=None
            engine.inputs['perception']=dict(runtime_readiness=dict(color_age_s=2.01,ipc_health_age_s=.4,reason='CAMERA_FRAME_STALE'))
            journal.transition(engine,201.);journal.close()
            rows=[json.loads(line) for line in journal.path.read_text().splitlines()]
            self.assertEqual(len(rows),2);self.assertEqual(journal.error,'')
            self.assertEqual(rows[1]['camera']['color_age_s'],2.01)
            self.assertEqual(rows[0]['pending']['operation'],'grasp_plan')
            self.assertLess(journal.path.stat().st_size,3000)

    def test_failed_ik_summary_is_bounded_and_preserves_identity_and_stages(self):
        metrics=dict(elapsed_s=2.,ik_summary=dict(calls=70),grasp_filter_counts=dict(after_bbox=20,after_nms=2,selected_candidate=None),
            ik_calls=[dict(success=False,candidate='rank=1 flip=0',stage='pregrasp',waypoint=0,
                reason='IK_OR_FK_TOLERANCE_OR_LIMIT',position_error_m=.09,orientation_error_rad=.2,
                position_tolerance_m=.004,orientation_tolerance_rad=.035,
                lower_margin_rad=[1.]*6,upper_margin_rad=[1.,1.,1.,0.,1.,1.],
                attempts=[dict(seed=[99]*6,reason='FK_POSITION')]) for _ in range(70)],
            planning_stages=[dict(stage='graspnet',elapsed_s=1.,status='completed',payload=[99]*100)],
            giant_raw_array=[99]*10000)
        with tempfile.TemporaryDirectory() as directory:
            journal=ShortJournal(directory,'perception','worker')
            request=dict(id=12,session_id='mission',operation='grasp_plan',goal_revision=2)
            journal.compute(request,dict(ok=False,reason='IK_FAIL'),metrics);journal.close()
            row=json.loads(journal.path.read_text());summary=row['summary']
            self.assertEqual(row['request_id'],12);self.assertEqual(row['mission_session_id'],'mission')
            self.assertEqual(summary['failed_ik_count'],70);self.assertEqual(len(summary['failed_ik']),64)
            self.assertEqual(summary['failed_ik_omitted'],6)
            self.assertEqual(summary['failed_ik'][0]['near_joint_limits'][0]['joint'],4)
            self.assertNotIn('attempts',summary['failed_ik'][0]);self.assertNotIn('payload',summary['stages'][0])
            self.assertNotIn('giant_raw_array',summary)
            self.assertEqual(summary['filter_counts']['after_nms'],2)

    def test_nonfinite_diagnostics_and_disk_cap_do_not_break_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            journal=ShortJournal(directory,'test','finite')
            journal.put('failure',error=float('inf'));journal.close()
            self.assertIsNone(json.loads(journal.path.read_text())['error'])
            capped=ShortJournal(directory,'test','capped',max_bytes=100)
            capped.put('large',value='x'*1000);capped.close()
            self.assertTrue(capped.limit_reached);self.assertLessEqual(capped.path.stat().st_size,100)
            self.assertFalse(capped.error)

    def test_disabled_factory_creates_no_directory_and_bad_path_is_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertIsNone(make_short_journal(dict(execution=dict(short_journal_enabled=False)),directory,'test','session'))
            self.assertFalse((Path(directory)/'runtime').exists())
            blocker=Path(directory)/'file';blocker.write_text('not a directory')
            failed=ShortJournal(blocker,'test','session');failed.put('start');failed.close()
            self.assertTrue(failed.error)


if __name__=='__main__':unittest.main()
