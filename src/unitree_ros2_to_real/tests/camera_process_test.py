"""Only synthetic camera children; never imports a hardware driver or ROS node."""
import ctypes
import sys
import threading
import time
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from mission.camera_process import CameraProcess

class FakeCamera:
    _fps=30;_FRAME_FAIL_LIMIT=3
    def __init__(self,cfg):self.cfg=cfg;self.K=np.eye(3);self.last_frame_timing=None
    def open(self):
        # PyDLL deliberately holds the child GIL like a blocking native SDK.
        ctypes.PyDLL(None).usleep(int(self.cfg.get('open_s',0)*1e6))
        if self.cfg.get('fail'):raise RuntimeError('synthetic open failure')
    def get_frame(self):
        time.sleep(.01)
        stamp=self.cfg.get('stamp',time.time_ns())
        self.last_frame_timing=dict(color_system_ns=stamp,depth_system_ns=stamp)
        return np.ones((2,2,3),dtype=np.uint8),np.ones((2,2),dtype=np.float32)
    def acquire_frames(self):
        time.sleep(.01);return self.cfg.get('stamp',time.time_ns())
    def frame_timing(self,stamp):return dict(color_system_ns=stamp,depth_system_ns=stamp)
    def decode_frames(self,stamp):
        self.last_frame_timing=self.frame_timing(stamp)
        return np.ones((2,2,3),dtype=np.uint8),np.ones((2,2),dtype=np.float32)
    def close(self):ctypes.PyDLL(None).usleep(int(self.cfg.get('close_s',0)*1e6))

def make_fake(cfg):return FakeCamera(cfg)

class Tests(unittest.TestCase):
    def test_diagnostics_distinguish_freshness_and_process_faults(self):
        from types import SimpleNamespace as NS
        c=CameraProcess.__new__(CameraProcess);c.condition=threading.Condition()
        c.closed=False;c.error='';c.process=NS(is_alive=lambda:True)
        stamp=time.time_ns();c.health=({'color_system_ns':stamp,'depth_system_ns':stamp},True,'')
        self.assertTrue(c.diagnostics(.6)['ready'])
        c.health=({'color_system_ns':stamp-1_000_000_000,'depth_system_ns':stamp},True,'')
        d=c.diagnostics(.6);self.assertEqual(d['reason'],'CAMERA_FRAME_STALE')
        self.assertGreater(d['color_age_s'],.6);self.assertLess(d['depth_age_s'],.6)
        c.health=({'color_system_ns':time.time_ns()+10_000_000_000,'depth_system_ns':stamp},True,'')
        self.assertEqual(c.diagnostics(.6)['reason'],'CAMERA_TIMESTAMP_FUTURE')
        stamp=time.time_ns()-1_500_000_000
        c.health=({'color_system_ns':stamp,'depth_system_ns':stamp},True,'')
        self.assertTrue(c.diagnostics(2.)['ready'])
        self.assertEqual(c.diagnostics(2.)['source_timestamps']['color_system_ns'],stamp)
        stamp=time.time_ns()-2_100_000_000
        c.health=({'color_system_ns':stamp,'depth_system_ns':stamp},True,'')
        self.assertEqual(c.diagnostics(2.)['reason'],'CAMERA_FRAME_STALE')
        c.health=(None,False,'');self.assertEqual(c.diagnostics(.6)['reason'],'CAMERA_DECODE_INVALID')
        c.health=(None,True,'SDK error');self.assertEqual(c.diagnostics(.6)['reason'],'CAMERA_ACQUISITION_ERROR')
        c.health=None;self.assertEqual(c.diagnostics(.6)['reason'],'CAMERA_HEALTH_MISSING')
        c.process=NS(is_alive=lambda:False);self.assertEqual(c.diagnostics(.6)['reason'],'CAMERA_PROCESS_EXITED')

    def camera(self,cfg,timeout=10.):
        return CameraProcess(cfg,timeout,.3,'continuous','camera_process_test:make_fake')
    def test_blocking_open_does_not_block_parent_heartbeat(self):
        ticks=[];stop=threading.Event()
        def heartbeat():
            while not stop.wait(.01):ticks.append(time.monotonic())
        t=threading.Thread(target=heartbeat);t.start()
        try:
            camera=self.camera({'open_s':1.})
            try:
                color,depth=camera.get_frame()
                self.assertEqual(color.shape,(2,2,3));self.assertEqual(depth.shape,(2,2))
                deadline=time.monotonic()+1
                while not camera.fresh(.5) and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(camera.fresh(.5))
                self.assertLess(time.time_ns()-camera.last_frame_timing['color_system_ns'],500_000_000)
            finally:camera.close()
            self.assertFalse(camera.process.is_alive());self.assertFalse(camera.receiver.is_alive())
        finally:stop.set();t.join()
        self.assertGreater(len(ticks),20)
        self.assertLess(max(b-a for a,b in zip(ticks,ticks[1:])),.5)
    def test_original_stale_exposure_is_not_refreshed_by_ipc(self):
        stamp=time.time_ns()-10_000_000_000
        camera=self.camera({'stamp':stamp})
        try:
            camera.get_frame();time.sleep(.05)
            self.assertEqual(camera.last_frame_timing['color_system_ns'],stamp)
            self.assertFalse(camera.fresh(.5))
        finally:camera.close()
    def test_open_failure_and_timeout(self):
        with self.assertRaisesRegex(ValueError,'synthetic open failure'):self.camera({'fail':True})
        with self.assertRaisesRegex(ValueError,'CAMERA_OPEN_TIMEOUT'):self.camera({'open_s':10},.4)
    def test_hung_close_reaps_only_child(self):
        camera=self.camera({'close_s':10})
        camera.close();camera.close()
        self.assertFalse(camera.process.is_alive());self.assertFalse(camera.receiver.is_alive())
        self.assertFalse(camera.fresh(.5))
    def test_latest_raw_mode_preserves_frame_and_freshness(self):
        camera=CameraProcess({},10.,.3,'latest_raw','camera_process_test:make_fake')
        try:
            camera.get_frame();first=camera.last_frame_timing['color_system_ns']
            time.sleep(.04);camera.get_frame()
            self.assertGreater(camera.last_frame_timing['color_system_ns'],first)
            time.sleep(.04);self.assertTrue(camera.fresh(.5))
            diagnostics=camera.diagnostics(.5)
            self.assertGreater(diagnostics['acquisition']['max_frame_wait_s'],0.)
            self.assertGreater(diagnostics['acquisition']['max_frame_send_s'],0.)
            self.assertGreaterEqual(diagnostics['ipc_timing']['acquisition_transport_s'],0.)
            self.assertGreater(diagnostics['ipc_timing']['max_health_receive_gap_s'],0.)

        finally:camera.close()

    def test_child_exit_invalidates_health(self):
        camera=self.camera({})
        try:
            camera.get_frame();camera.process.terminate();camera.process.join(1)
            self.assertFalse(camera.fresh(.5))
        finally:camera.close()

if __name__=='__main__':unittest.main()
