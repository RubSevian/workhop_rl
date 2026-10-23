"""Cancellation identity, late-result suppression and recovery; no ROS or motors."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.compute_cancel import ComputeCancellation
from mission.collision import Collision,CollisionCancelled

class CancellationTests(unittest.TestCase):
    def setUp(self):
        self.g=ComputeCancellation()
        self.r=dict(id=3,session_id='a',observed_ns=100,operation='grasp_plan')
        self.g.begin(self.r)
    def test_stale_and_foreign_status_do_not_cancel(self):
        for status in (dict(session_id='b',observed_ns=101,state='BLOCKED'),
                       dict(session_id='a',observed_ns=99,state='BLOCKED')):
            self.assertFalse(self.g.observe(status));self.g.check()
        self.assertTrue(self.g.allows(self.r))
    def test_cancel_blocks_collision_and_queued_result_then_new_request_recovers(self):
        for state in ('BLOCKED','CANCELLED'):
            self.g.begin(self.r)
            self.assertTrue(self.g.observe(dict(session_id='a',observed_ns=101,state=state)))
            c=Collision.__new__(Collision);c.cancel_check=self.g.check
            with self.assertRaises(CollisionCancelled):c.check_deadline()
            self.assertFalse(self.g.allows(self.r))
            self.g.observe(dict(session_id='a',observed_ns=102,state='PLAN_GRASP'))
            self.assertFalse(self.g.allows(self.r))
            new=self.r|dict(id=4,observed_ns=103)
            self.g.begin(new);self.g.check()
            self.assertTrue(self.g.allows(new));self.assertFalse(self.g.allows(self.r))
    def test_terminal_mission_does_not_cancel_controlled_stop_home(self):
        self.g.begin(self.r|dict(operation='home'))
        self.assertFalse(self.g.observe(dict(session_id='a',observed_ns=101,state='CANCELLED')))
        self.g.check()

if __name__=='__main__':unittest.main()
