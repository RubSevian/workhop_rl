"""Correlated negative admission; no hardware, services or automatic retries."""
import json
import unittest
from first_grasp_mission_test import Fixture
from mission.plan_wire import encode_plan


class RejectionTests(unittest.TestCase):
    def waiting(self):
        f=Fixture();f.start()
        for _ in range(500):
            effects=f.tick(False)
            for kind,data in effects:
                if kind=='trajectory':return f,data
                f.effect(kind,data)
        self.fail('no trajectory')

    def rejection(self,f,command):
        return dict(rejected_plan_id=command['id'],rejected_session_id=f.m.session,
            rejected_owner_session_id=f.m.owner_session,rejected_navigation_session_id=f.m.nav_session,
            rejected_goal_revision=f.m.revision,rejected_ns=int(f.now*1e9),rejection_reason='start mismatch')

    def test_matching_rejection_blocks_and_cancels_once(self):
        f,command=self.waiting();f.owner.update(self.rejection(f,command))
        effects=f.tick(False)
        self.assertEqual(f.m.state,'BLOCKED');self.assertIsNone(f.m.pending)
        self.assertEqual(f.m.reason,'OWNER_REJECTED:start mismatch')
        self.assertEqual([k for k,_ in effects],['cancel'])
        self.assertEqual(f.tick(False),[])

    def test_old_identity_or_timestamp_cannot_end_wait(self):
        for key,value in [('rejected_plan_id',0),('rejected_session_id','old'),
                ('rejected_owner_session_id','old'),('rejected_navigation_session_id','old'),
                ('rejected_goal_revision',0),('rejected_ns',1)]:
            with self.subTest(key=key):
                f,command=self.waiting();reply=self.rejection(f,command);reply[key]=value
                f.owner.update(reply);f.tick(False)
                self.assertEqual(f.m.pending['id'],command['id'])
                self.assertNotEqual(f.m.state,'BLOCKED')

    def test_cancelled_request_cannot_be_revived_by_late_rejection(self):
        f,command=self.waiting();f.m.cancel(f.now,'OPERATOR_CANCEL')
        f.owner.update(self.rejection(f,command));f.tick(False)
        self.assertEqual(f.m.state,'CANCELLED');self.assertIsNone(f.m.pending)

    def test_wire_remains_json_and_identity_is_bounded_before_arrays(self):
        plan=dict(id=2**63+11,session_id='quoted"session',owner_session_id='owner',
                  navigation_session_id='nav',goal_revision=7,points=[[0]*7]*10000)
        wire=encode_plan(plan);decoded=json.loads(wire)
        identity=decoded.pop('request_identity')
        self.assertEqual(decoded,plan);self.assertEqual(identity['id'],plan['id'])
        self.assertLess(len(wire.split('\n',1)[0]),2048)
        with self.assertRaisesRegex(ValueError,'IDENTITY_TOO_LARGE'):
            encode_plan(dict(plan,session_id='x'*2049))

if __name__=='__main__':unittest.main()
