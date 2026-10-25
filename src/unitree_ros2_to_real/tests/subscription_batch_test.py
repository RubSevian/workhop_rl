#!/usr/bin/env python3
"""Offline queue/measurement tests; no ROS graph or actuator connections."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.subscription_batch import take_batch, dispatch_pending


class Handle:
    def __init__(self, rows):
        self.rows=deque(rows);self.calls=0
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def take_message(self,*args):
        self.calls+=1
        return (self.rows.popleft(),None) if self.rows else None


class Tests(unittest.TestCase):
    def subscription(self, rows):
        return NS(handle=Handle(rows),msg_type=object,raw=False)
    def test_bounded_drain_keeps_history_and_newest_status(self):
        rows=[dict(frame_received_ns=10+i,q=i) for i in range(8)]
        sub=self.subscription(rows[1:])
        batch=take_batch(sub,rows[0],4)
        self.assertEqual(batch,rows[:4])
        self.assertIs(batch[-1],rows[3])
        self.assertEqual(list(sub.handle.rows),rows[4:])
        self.assertEqual(sub.handle.calls,3)
        self.assertEqual([m['frame_received_ns'] for m in batch],[10,11,12,13])
    def test_empty_queue_does_not_wait_or_create_samples(self):
        first=object();sub=self.subscription([])
        self.assertEqual(take_batch(sub,first,32),[first])
        self.assertEqual(sub.handle.calls,1)
    def test_health_reads_queued_measurement_before_freshness_check(self):
        from first_grasp_mission_test import Fixture
        from mission.scene import SceneBuffer,BodyPose
        f=Fixture();f.feed();now=round(f.now*1e9)
        original=f.m.inputs['scene']
        for queued_age,fresh in ((10_000_000,True),(600_000_000,False)):
            scene=SceneBuffer(f.m.cfg)
            scene.bodies.append(BodyPose(**original['body']))
            scene.arm([0.]*7,now-700_000_000,'arm')
            scene.leg([0.]*12,now)
            with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:live_arm'):scene.snapshot(now)
            queued=dict(q=[0.]*7,frame_received_ns=now-queued_age)
            sub=self.subscription([queued])
            self.assertTrue(dispatch_pending(sub,lambda msg:scene.arm(msg['q'],msg['frame_received_ns'],'arm')))
            if fresh:
                result=scene.snapshot(now)
                self.assertEqual(result.live_feedback_ns,queued['frame_received_ns'])
            else:
                with self.assertRaisesRegex(ValueError,'SCENE_SNAPSHOT_STALE:live_arm'):scene.snapshot(now)
            self.assertFalse(dispatch_pending(sub,lambda msg:self.fail('empty queue fabricated data')))

    def test_dispatch_uses_original_bounded_callback(self):
        sub=self.subscription(list(range(8)));seen=[]
        self.assertTrue(dispatch_pending(sub,lambda first:seen.extend(take_batch(sub,first,4))))
        self.assertEqual(seen,[0,1,2,3]);self.assertEqual(list(sub.handle.rows),[4,5,6,7])
        self.assertFalse(dispatch_pending(None,lambda msg:self.fail('missing subscription')))

    def test_depth_one_and_invalid_bounds(self):
        sub=self.subscription([2])
        self.assertEqual(take_batch(sub,1,1),[1])
        self.assertEqual(sub.handle.calls,0)
        for limit in [0,401,True,2.5]:
            with self.assertRaises(ValueError):take_batch(sub,1,limit)


if __name__=='__main__':unittest.main()
