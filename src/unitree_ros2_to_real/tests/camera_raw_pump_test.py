"""Bounded raw acquisition and unchanged sample age; no camera/ROS/serial."""
import sys,time,unittest,threading
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.planning import CameraPump

class Source:
    _fps=30;_FRAME_FAIL_LIMIT=3
    def __init__(self):
        self.K=np.eye(3);self.closed=False;self.raw=0;self.decoded=[];self.stamp=None;self.last_frame_timing=None;self.fail=False
    def acquire_frames(self):
        time.sleep(.003)
        if self.fail:return None
        self.raw+=1
        return dict(index=self.raw,stamp=self.stamp or time.time_ns())
    def frame_timing(self,frame):
        if frame is None:return None
        return dict(color_system_ns=frame['stamp'],depth_system_ns=frame['stamp'])
    def decode_frames(self,frame):
        self.decoded.append(frame['index']);self.last_frame_timing=self.frame_timing(frame)
        return np.full((2,2,3),frame['index']),np.full((2,2),frame['index'])
    def close(self):self.closed=True

class RawPumpTests(unittest.TestCase):
    def test_continuous_acquisition_decodes_only_requested_fresh_samples(self):
        source=Source();pump=CameraPump(source,processing_mode='latest_raw')
        try:
            time.sleep(.025);self.assertGreater(source.raw,3);self.assertEqual(source.decoded,[])
            self.assertFalse(pump.fresh(.6)) # Stream alone is not a successful image probe.
            a,b=pump.get_frame();first=source.decoded[-1];self.assertTrue(pump.fresh(.6))
            time.sleep(.025);self.assertEqual(source.decoded,[first]);self.assertGreater(source.raw,first+2)
            c,d=pump.get_frame();self.assertGreater(source.decoded[-1],first)
            np.testing.assert_array_equal(a[:,:,0],b);np.testing.assert_array_equal(c[:,:,0],d)
            self.assertEqual(pump.stats()['processed_frames'],2)
        finally:pump.close()
        self.assertTrue(source.closed);self.assertFalse(pump.thread.is_alive());self.assertIsNone(pump.latest)
    def test_original_stale_stamp_is_not_renewed_by_continuous_receipt(self):
        source=Source();source.stamp=time.time_ns()-2_000_000_000;pump=CameraPump(source,processing_mode='latest_raw')
        try:
            pump.get_frame();self.assertEqual(pump.last_frame_timing['color_system_ns'],source.stamp)
            self.assertFalse(pump.fresh(.6))
        finally:pump.close()
    def test_raw_dropout_invalidates_health_and_wakes_consumer(self):
        source=Source();pump=CameraPump(source,processing_mode='latest_raw')
        try:
            pump.get_frame();source.fail=True
            deadline=time.monotonic()+1
            while not pump.error and time.monotonic()<deadline:time.sleep(.005)
            self.assertFalse(pump.fresh(.6))
            with self.assertRaisesRegex(ValueError,'RGBD_ACQUISITION_FAILED'):pump.get_frame()
        finally:pump.close()
    def test_close_interrupts_waiting_consumer(self):
        source=Source();pump=CameraPump(source,processing_mode='latest_raw');pump.close()
        with self.assertRaisesRegex(ValueError,'RGBD_CLOSED'):pump.get_frame()
    def test_unsupported_camera_is_not_silently_switched(self):
        with self.assertRaisesRegex(ValueError,'RAW_INTERFACE'):CameraPump(object(),processing_mode='latest_raw')

if __name__=='__main__':unittest.main()
