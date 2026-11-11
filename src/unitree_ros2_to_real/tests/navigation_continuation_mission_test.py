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
        for state in ('CONTINUATION_NOT_FOUND','ARRIVAL_NOT_CONFIRMED','REPEATED_ATTEMPT','PROGRESS_HISTORY_EXHAUSTED'):
            f=self.outbound();f.nav['goal']['state']=state;f.tick(False)
            self.assertEqual(f.m.state,'CANCELLED')
            self.assertEqual(f.m.reason,'NAV_'+state)

    def test_navigation_success_cannot_bypass_deadline(self):
        for limit,elapsed,expected in ((1.,2.,'BLOCKED'),(1.,1.,'BASE_SETTLE'),(None,100.,'BASE_SETTLE')):
            with self.subTest(limit=limit,elapsed=elapsed):
                f=self.outbound();f.nav['goal']['state']='SUCCEEDED';f.feed()
                f.m.cfg['execution']['navigation_timeout_s']=limit
                f.m.entered=f.now-elapsed
                effects=f.m.step(f.now,int(f.now*1e9))
                self.assertEqual(f.m.state,expected)
                self.assertFalse(any(kind in ('compute','trajectory') for kind,_ in effects))
                if expected=='BLOCKED':
                    self.assertEqual(f.m.reason,'NAV_TIMEOUT')
                    self.assertEqual([kind for kind,_ in effects],['cancel'])

    def test_navigation_failure_identity_and_authority_keep_priority(self):
        for cause,reason in (('failure','NAV_FAILED'),('goal','GOAL_REPLACED'),('authority','R3_AUTHORITY_REVOKED')):
            with self.subTest(cause=cause):
                f=self.outbound();f.m.cfg['execution']['navigation_timeout_s']=1.;f.m.entered=f.now-2
                f.nav['goal']['state']='FAILED' if cause=='failure' else 'SUCCEEDED'
                if cause=='goal':f.nav['goal']['revision']+=1
                if cause=='authority':f.r3['fault_latched']=True
                effects=f.tick(False)
                self.assertEqual(f.m.reason,reason)
                self.assertEqual([kind for kind,_ in effects],['cancel'])

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
        f.nav['goal']['state']='REPEATED_ATTEMPT';f.tick(False)
        self.assertEqual(f.m.reason,'RETURN_NAV_REPEATED_ATTEMPT')

if __name__=='__main__':unittest.main()
