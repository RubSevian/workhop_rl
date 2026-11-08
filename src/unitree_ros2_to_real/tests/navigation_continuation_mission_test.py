"""Mission state integration for retained FAR goals; no hardware or ROS init."""
import copy
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture


class MissionContinuationTests(unittest.TestCase):
    def outbound(self,state='REPLANNING'):
        f=Fixture();f.tick();self.assertEqual(f.m.state,'WAIT_GOAL')
        f.nav['goal']=dict(revision=1,stamp_ns=int((f.now+.01)*1e9),state=state)
        f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')
        return f

    def test_pending_states_do_not_cancel_or_begin_arm_work(self):
        f=self.outbound()
        for state in ('REPLANNING','ACTIVE','ARRIVAL_CONFIRMING','REPLANNING','ACTIVE'):
            f.nav['goal']['state']=state
            effects=f.tick(False)
            self.assertEqual(f.m.state,'NAVIGATING')
            self.assertFalse(any(k in ('compute','trajectory','cancel') for k,_ in effects))
        f.nav['goal']['state']='SUCCEEDED';f.tick(False)
        self.assertEqual(f.m.state,'BASE_SETTLE')

    def test_expired_continuation_has_explicit_reason(self):
        f=self.outbound();f.nav['goal']['state']='CONTINUATION_NOT_FOUND';f.tick(False)
        self.assertEqual(f.m.state,'CANCELLED')
        self.assertEqual(f.m.reason,'NAV_CONTINUATION_NOT_FOUND')

    def test_replaced_goal_and_cancel_still_win(self):
        f=self.outbound();f.nav['goal']['revision']=2;f.tick(False)
        self.assertEqual(f.m.reason,'GOAL_REPLACED')
        f=self.outbound();f.m.cancel(f.now,'OPERATOR_CANCEL')
        f.nav['goal']['state']='ACTIVE';f.tick(False)
        self.assertEqual(f.m.state,'CANCELLED')

    def test_return_navigation_accepts_same_intermediate_states(self):
        completed=Fixture();self.assertEqual(completed.run(),'GRASP_PRESUMED_SUCCESS')
        f=copy.deepcopy(completed);f.feed()
        accepted,reason=f.m.return_navigation.request(f.m,f.now)
        self.assertTrue(accepted,reason)
        f.nav.update(base_locked=False,return_request_ns=f.m.return_navigation.request_ns)
        effects=f.tick(False)
        goal=next(d for k,d in effects if k=='return_navigation_goal')
        f.nav['goal']=dict(revision=2,stamp_ns=goal['stamp_ns'],state='REPLANNING')
        f.tick(False);self.assertEqual(f.m.state,'RETURN_NAVIGATING')
        for state in ('ARRIVAL_CONFIRMING','ACTIVE','REPLANNING'):
            f.nav['goal']['state']=state;f.tick(False)
            self.assertEqual(f.m.state,'RETURN_NAVIGATING')

if __name__=='__main__':unittest.main()
