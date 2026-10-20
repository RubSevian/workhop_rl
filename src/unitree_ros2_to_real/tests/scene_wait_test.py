#!/usr/bin/env python3
"""Deterministic delayed-observation tests; no camera or robot."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.scene_wait import camera_context, select_camera_frame


class Tests(unittest.TestCase):
    def setUp(self):
        self.now=1_000_000_000
        self.capture=self.now
    def sleep(self, seconds):
        self.now+=round(seconds*1e9)
    def run_context(self, read):
        with patch('mission.scene_wait.time.monotonic_ns',side_effect=lambda:self.now),patch('mission.scene_wait.time.sleep',side_effect=self.sleep):
            return camera_context(read,self.capture,.6,.02)
    def test_wait_for_real_delayed_pair_keeps_exposure(self):
        def read():
            if self.now<self.capture+200_000_000:raise ValueError('SCENE_SNAPSHOT_SKEW')
            return dict(capture_ns=self.capture)
        result=self.run_context(read)
        self.assertEqual(result['capture_ns'],1_000_000_000)
        self.assertEqual(self.now,1_200_000_000)
    def test_timeout_cannot_refresh_exposure(self):
        def read():raise ValueError('SCENE_SNAPSHOT_STALE:body')
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):self.run_context(read)
        self.assertEqual(self.now,self.capture+600_000_000)
    def test_epoch_fault_is_immediate(self):
        def read():raise ValueError('ODOMETRY_TIME_RESET')
        with self.assertRaisesRegex(ValueError,'ODOMETRY_TIME_RESET'):self.run_context(read)
        self.assertEqual(self.now,self.capture)
    def test_success_after_deadline_is_rejected(self):
        def read():
            self.now+=601_000_000
            return object()
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):self.run_context(read)

    def run_burst(self, frames, admit, deadline=lambda:None):
        with patch('mission.scene_wait.time.monotonic_ns',side_effect=lambda:self.now),patch('mission.scene_wait.time.sleep',side_effect=self.sleep):
            return select_camera_frame(frames,admit,deadline,.6,.02)
    def test_burst_selects_newest_matching_real_exposure(self):
        calls=[]
        def admit(frame):
            calls.append(frame)
            if frame=='newest':raise ValueError('SCENE_SNAPSHOT_SKEW')
            return frame
        frames=[(800_000_000,'oldest'),(850_000_000,'matching'),(900_000_000,'newest')]
        self.assertEqual(self.run_burst(frames,admit),'matching')
        self.assertEqual(calls,['newest','matching'])
        self.assertEqual(self.now,1_000_000_000)
    def test_burst_waits_for_delayed_history_but_keeps_stamps(self):
        def admit(frame):
            if self.now<1_100_000_000:raise ValueError('BODY_SNAPSHOT_MISSING')
            return frame
        frame={'exposure':900_000_000}
        result=self.run_burst([(900_000_000,frame)],admit)
        self.assertIs(result,frame);self.assertEqual(result['exposure'],900_000_000)
    def test_burst_each_candidate_has_own_expiry(self):
        seen=[]
        def admit(frame):seen.append(frame);return frame
        self.assertEqual(self.run_burst([(300_000_000,'expired'),(900_000_000,'valid')],admit),'valid')
        self.assertEqual(seen,['valid'])
        def late(frame):self.now+=601_000_000;return frame
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):self.run_burst([(self.now,'late')],late)
    def test_older_expiry_does_not_discard_still_valid_newer_candidate(self):
        def admit(frame):
            if frame=='new':
                if self.now<1_050_000_000:raise ValueError('SCENE_SNAPSHOT_SKEW')
                return frame
            self.now+=100_000_000
            raise ValueError('RGBD_STALE_OR_REPEATED')
        self.assertEqual(self.run_burst([(450_000_000,'old'),(900_000_000,'new')],admit),'new')

    def test_burst_does_not_hide_epoch_or_operation_deadline(self):
        def fail(frame):raise ValueError('ODOMETRY_TIME_RESET')
        with self.assertRaisesRegex(ValueError,'ODOMETRY_TIME_RESET'):self.run_burst([(self.now,'frame')],fail)
        def expired():raise RuntimeError('operation deadline')
        with self.assertRaisesRegex(RuntimeError,'operation deadline'):self.run_burst([(self.now,'frame')],lambda f:f,expired)


if __name__=='__main__':unittest.main()
