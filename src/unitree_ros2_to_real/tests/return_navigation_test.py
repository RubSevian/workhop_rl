"""Complete outbound/grasp/HOME/return flow and return-lease rejection; no ROS init."""
import copy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture
from mission_health import MissionHealth


class ReturnNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.completed=Fixture();assert cls.completed.run()=='GRASP_PRESUMED_SUCCESS'

    def setUp(self):
        self.f=copy.deepcopy(self.completed)
        self.f.feed()

    def request(self):
        f=self.f
        accepted,reason=f.m.return_navigation.request(f.m,f.now)
        self.assertTrue(accepted,reason)
        f.nav.update(base_locked=False,return_request_ns=f.m.return_navigation.request_ns)
        effects=f.tick(False)
        goal=next(d for k,d in effects if k=='return_navigation_goal')
        f.nav['goal']=dict(revision=2,stamp_ns=goal['stamp_ns'],state='ACTIVE',error_xy_m=.5,tolerance_m=.25)
        f.tick(False)
        self.assertEqual(f.m.state,'RETURN_NAVIGATING')
        return goal

    def test_exact_window_and_manual_return_no_second_grasp(self):
        f=self.f;window=dict(f.m.base_motion_window)
        self.assertLess(window['scan_started_ns'],window['home_completed_ns'])
        self.assertIsNotNone(f.m.return_navigation.start)
        before=f.commands;goal=self.request()
        self.assertEqual(goal['target'],f.m.return_navigation.target())
        f.nav['goal'].update(state='SUCCEEDED',error_xy_m=.01)
        for _ in range(300):
            f.tick()
            if f.m.state=='RETURNED_TO_START':break
        self.assertEqual(f.m.state,'RETURNED_TO_START')
        self.assertEqual(f.commands,before)
        self.assertEqual(f.m.base_motion_window,window)
        self.assertFalse(f.m.return_navigation.closure['orientation_restored'])
        self.assertFalse(f.m.return_navigation.request(f.m,f.now)[0])

    def test_request_rejected_without_home_start_or_fresh_localization(self):
        for mutation,reason in [
            (lambda f:setattr(f.m.return_navigation,'start',None),'RETURN_START_POSE_UNAVAILABLE'),
            (lambda f:f.owner.update(arm_home_ready=False),'RETURN_ARM_NOT_HOME'),
            (lambda f:f.nav.update(cancel_latched=True),'RETURN_NAV_UNAVAILABLE_OR_CANCELLED'),
            (lambda f:f.owner.update(grasp_assessment='NOT_GRASPED'),'RETURN_GRASP_NO_LONGER_CONFIRMED'),
            (lambda f:setattr(f,'tf_revision','new-epoch'),'RETURN_LOCALIZATION_OR_OWNER_EPOCH_CHANGED')]:
            with self.subTest(reason=reason):
                f=copy.deepcopy(self.completed);mutation(f);f.feed()
                self.assertEqual(f.m.return_navigation.request(f.m,f.now),(False,reason))

    def test_loss_during_return_cancels_and_does_not_resume(self):
        self.request();f=self.f
        f.owner['grasp_assessment']='UNKNOWN';effects=f.tick(False)
        self.assertEqual(f.m.state,'BLOCKED');self.assertIn(('cancel',{'reason':'RETURN_GRASP_NO_LONGER_CONFIRMED'}),effects)
        f.owner['grasp_assessment']='GRASPED';f.tick(False)
        self.assertEqual(f.m.state,'BLOCKED')

    def test_new_goal_during_return_is_rejected(self):
        self.request();f=self.f;f.nav['goal']['revision']=3;f.tick(False)
        self.assertEqual(f.m.reason,'RETURN_GOAL_REPLACED')

    def test_abort_window_never_claims_completed_home(self):
        f=Fixture();f.start()
        for _ in range(300):
            f.tick()
            if f.m.state=='ARM_SCAN_POSE':break
        f.m.cancel(f.now,'OPERATOR_CANCEL')
        self.assertIsNone(f.m.base_motion_window['home_completed_ns'])


class ReturnLeaseTests(unittest.TestCase):
    def health(self):
        h=MissionHealth(.25);h.graph([b'a'],1)
        data=dict(observed_ns=1_000_000_000,session_id='m',navigation_session_id='n',ready=True,
                  physical=True,phase='RETURN_HOME',base_locked=True)
        h.observe(json.dumps(data),b'a',1_000_000_001,'n')
        self.assertTrue(h.base_locked)
        return h,data

    def grant(self,h,d,**changes):
        d=dict(d,observed_ns=1_100_000_000,phase='RETURN_NAV_PREPARE',base_locked=False,
               return_request_ns=1_090_000_000,home_completed_ns=1_050_000_000,
               arm_home_ready=True,return_target=[1.,2.,0.]);d.update(changes)
        h.observe(json.dumps(d),h.gid,1_100_000_001,'n');return d

    def test_only_explicit_same_owner_return_can_unlock_once(self):
        h,d=self.health();d=self.grant(h,d)
        self.assertFalse(h.base_locked);self.assertEqual(h.return_target,(1.,2.,0.))
        d.update(observed_ns=1_200_000_000,phase='RETURNED_TO_START',base_locked=True)
        h.observe(json.dumps(d),b'a',1_200_000_001,'n');self.assertTrue(h.base_locked)
        self.grant(h,d,observed_ns=1_300_000_000,return_request_ns=1_290_000_000)
        self.assertTrue(h.base_locked)

    def test_ordinary_unlock_stale_wrong_home_and_new_owner_are_rejected(self):
        for changes in [dict(phase='NAVIGATING'),dict(ready=False),dict(arm_home_ready=False),
                        dict(home_completed_ns=None),dict(return_request_ns=0),dict(observed_ns=1)]:
            h,d=self.health();self.grant(h,d,**changes);self.assertTrue(h.base_locked)
        h,d=self.health();h.graph([b'b'],1_010_000_000);self.grant(h,d)
        self.assertTrue(h.base_locked)


if __name__=='__main__':unittest.main()
