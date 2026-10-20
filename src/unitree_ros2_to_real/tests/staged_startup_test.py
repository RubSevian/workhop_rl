#!/usr/bin/env python3
"""Offline startup/cache/camera lifecycle regressions; no ROS or hardware."""
import copy
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from mission.startup import StartupRelease,MARKER,verified
from mission.solid import MeshSolid
from mission.planning import Planning

class Shape:
    num_tris=4
    def vertices(self):return np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1.]])
    def tri_indices(self,i):return [(0,2,1),(0,1,3),(0,3,2),(1,2,3)][i]

class Camera:
    def __init__(self):
        self.K=np.eye(3);self.closed=False;self.last_frame_timing=None;self.reads=0
    def open(self):self.closed=False
    def close(self):self.closed=True
    def get_frame(self):
        assert not self.closed
        time.sleep(.003);self.reads+=1
        stamp=time.time_ns()
        self.last_frame_timing={'color_system_ns':stamp,'depth_system_ns':stamp}
        return np.zeros((4,4,3),dtype=np.uint8),np.zeros((4,4),dtype=np.uint16)

class StartupTests(unittest.TestCase):
    def test_nice_only_changes_compute_thread(self):
        import os,threading
        from mission.compute_scheduling import inference_thread_nice
        original=os.getpriority(os.PRIO_PROCESS,threading.get_native_id());observed=[]
        worker=threading.Thread(target=lambda:observed.append(inference_thread_nice(min(19,original+5))))
        worker.start();worker.join()
        self.assertEqual(observed,[min(19,original+5)])
        self.assertEqual(os.getpriority(os.PRIO_PROCESS,threading.get_native_id()),original)

    def test_shutdown_runtime_error_is_not_confused_with_operational_fault(self):
        from mission import ros_main
        from unittest.mock import Mock
        import signal
        handlers={};destroy=Mock()
        def install(sig,handler):handlers[sig]=handler
        def interrupted_spin(node):
            try:handlers[signal.SIGINT]()
            except KeyboardInterrupt:pass
            raise RuntimeError('interrupted take_message conversion')
        with patch.object(ros_main.rclpy,'init'),patch.object(ros_main.rclpy,'try_shutdown'),patch.object(ros_main.signal,'signal',side_effect=install),patch.object(ros_main.rclpy,'spin',side_effect=interrupted_spin):
            ros_main.spin(lambda:SimpleNamespace(destroy_node=destroy))
        destroy.assert_called_once()
        with patch.object(ros_main.rclpy,'init'),patch.object(ros_main.rclpy,'try_shutdown'),patch.object(ros_main.signal,'signal'),patch.object(ros_main.rclpy,'spin',side_effect=RuntimeError('real fault')):
            with self.assertRaisesRegex(RuntimeError,'real fault'):
                ros_main.spin(lambda:SimpleNamespace(destroy_node=lambda:None))

    def test_perception_shutdown_joins_worker_and_closes_backend(self):
        try:
            from grasp_perception import Perception
            from rclpy.node import Node
        except ImportError:self.skipTest('ROS message environment not sourced')
        import threading,queue
        calls=[];ready=threading.Event()
        class Backend:
            def __init__(self,*args,**kwargs):
                self.startup_checks=dict.fromkeys(('geometry','yolo','graspnet','camera'),True)
                self.collision=SimpleNamespace(work_deadline_ns=None)
                self.worker=SimpleNamespace(close=lambda force:calls.append('inference stopped'))
            def close(self):calls.append('backend closed')
        node=Perception.__new__(Perception)
        node.cfg={'startup':{'shutdown_timeout_s':1.}};node.backend=None;node.startup_checks={}
        node.stop_event=threading.Event();node.jobs=queue.Queue(1);node.completed=queue.Queue(1)
        node.get_logger=lambda:SimpleNamespace(info=lambda msg:ready.set(),error=lambda msg:None,warning=lambda msg:None)
        with patch.dict(sys.modules,{'mission.planning':SimpleNamespace(Planning=Backend)}),patch.object(Node,'destroy_node',return_value=True):
            node.worker_thread=threading.Thread(target=node.work)
            node.worker_thread.start();self.assertTrue(ready.wait(1.))
            node.destroy_node()
        self.assertFalse(node.worker_thread.is_alive());self.assertIsNone(node.backend)
        self.assertEqual(calls,['inference stopped','backend closed'])

    def test_close_releases_models_even_if_camera_close_fails(self):
        calls=[]
        p=Planning.__new__(Planning);p._closed=False
        p.yolo=object();p.last_detection=object()
        p.camera=SimpleNamespace(close=lambda:(_ for _ in ()).throw(RuntimeError('camera stop')))
        p.worker=SimpleNamespace(close=lambda force:calls.append('worker closed'))
        with self.assertRaisesRegex(RuntimeError,'PERCEPTION_CLOSE_FAILED'):p.close()
        self.assertEqual(calls,['worker closed'])
        self.assertIsNone(p.camera);self.assertIsNone(p.worker);self.assertIsNone(p.yolo);self.assertIsNone(p.last_detection)
        p.close();self.assertEqual(calls,['worker closed'])

    def test_launch_constructs_only_preload_until_verified_output(self):
        try:
            from launch import LaunchContext
            from launch.events.process import ProcessIO
            from launch.actions import RegisterEventHandler
            from launch_ros.actions import Node
        except ImportError:self.skipTest('ROS launch environment not sourced')
        import importlib.util
        import launch.logging
        log_dir=self.enterContext(tempfile.TemporaryDirectory())
        launch.logging.launch_config.log_dir=log_dir
        root=SCRIPTS.parents[4]
        spec=importlib.util.spec_from_file_location('staged_launch',SCRIPTS.parent/'launch/go2_rars01_first_grasp.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        context=LaunchContext()
        context.launch_configurations.update(workspace=str(root),physical='true',start_arm_owner='true',
            network_interface='enP8p1s0',mission_config_path=str(root/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'))
        actions=module._launch(context)
        self.assertEqual(len(actions),2)
        self.assertIsInstance(actions[0],RegisterEventHandler);self.assertIsInstance(actions[1],Node)
        handler=actions[0].event_handler
        def output(data):
            event=ProcessIO(action=actions[1],name='preload',cmd=[],cwd=None,env=None,pid=1,text=data,fd=1)
            self.assertTrue(handler.matches(event));return handler.handle(event,context)
        self.assertEqual(output(b'INITIALIZING:camera_probe\n'),[])
        self.assertEqual(output(MARKER[:10].encode()),[])
        released=output((MARKER[10:]+'\n').encode())
        self.assertGreaterEqual(len(released),7)
        self.assertEqual(output((MARKER+'\n').encode()),[])
        # No Action.execute or LaunchService.run: no process/hardware started.

    def test_launch_releases_once_only_on_complete_marker(self):
        r=StartupRelease()
        self.assertFalse(r.feed(b'INITIALIZING:collision_geometry\n'))
        self.assertFalse(r.feed(('prefix '+MARKER+'\n').encode()))
        self.assertFalse(r.feed(MARKER[:10].encode()))
        self.assertTrue(r.feed((MARKER[10:]+'\n').encode()))
        self.assertFalse(r.feed((MARKER+'\n').encode()))
        for missing in ('geometry','yolo','graspnet','camera'):
            checks=dict(geometry=True,yolo=True,graspnet=True,camera=True)
            self.assertTrue(verified(checks));checks[missing]=False
            self.assertFalse(verified(checks))

    def test_cache_roundtrip_invalidation_and_corruption(self):
        with tempfile.TemporaryDirectory() as root:
            mesh=Path(root)/'mesh';mesh.write_bytes(b'mesh v1')
            cache=Path(root)/'cache'
            a,hit=MeshSolid.cached(Shape(),cache,mesh,[1,1,1],'test')
            self.assertFalse(hit)
            with patch.object(MeshSolid,'__init__',side_effect=AssertionError('rebuild')):
                b,hit=MeshSolid.cached(Shape(),cache,mesh,[1,1,1],'test')
            self.assertTrue(hit)
            for point in ([.1,.1,.1],[2,2,2]):self.assertEqual(a.contains(point),b.contains(point))
            next(cache.glob('*.npz')).write_bytes(b'corrupted')
            _,hit=MeshSolid.cached(Shape(),cache,mesh,[1,1,1],'test');self.assertFalse(hit)
            mesh.write_bytes(b'mesh v2')
            _,hit=MeshSolid.cached(Shape(),cache,mesh,[1,1,1],'test');self.assertFalse(hit)
            _,hit=MeshSolid.cached(Shape(),cache,mesh,[2,1,1],'test');self.assertFalse(hit)
            _,hit=MeshSolid.cached(Shape(),cache,mesh,[2,1,1],'new-version');self.assertFalse(hit)

    def test_camera_probe_standby_resume_and_dead_worker(self):
        p=Planning.__new__(Planning)
        p.cfg={'startup':{'camera_probe_frames':3,'camera_probe_timeout_s':1.,'camera_stop_timeout_s':2.},'execution':{'camera_max_age_s':.6}}
        p.grasp={'camera':{'fps':30}}
        p.camera=None;p.camera_mode='unverified';p.camera_error=''
        p.startup_checks=dict(geometry=True,yolo=True,graspnet=True,camera=True)
        p.worker=SimpleNamespace(is_alive=lambda:True)
        cameras=[]
        def make_camera(cfg):
            c=Camera();cameras.append(c);return c
        with patch.dict(sys.modules,{'drivers.camera':SimpleNamespace(make_camera=make_camera)}):
            p.resume_camera();self.assertTrue(p.sensors_ready())
            first_stamp=p.camera.last_frame_timing['color_system_ns']
            p.pause_camera();self.assertTrue(cameras[0].closed);self.assertIsNone(p.camera)
            reads=cameras[0].reads;time.sleep(.02);self.assertEqual(reads,cameras[0].reads)
            self.assertTrue(p.sensors_ready())
            p.resume_camera();self.assertGreater(p.camera.last_frame_timing['color_system_ns'],first_stamp)
            self.assertEqual(len(cameras),2)
            p.pause_camera()
        p.worker.is_alive=lambda:False;self.assertFalse(p.sensors_ready())

    def test_camera_repeated_frames_do_not_pass_probe(self):
        p=Planning.__new__(Planning)
        p.cfg={'startup':{'camera_probe_frames':3,'camera_probe_timeout_s':.05,'camera_stop_timeout_s':2.},'execution':{'camera_max_age_s':.6}}
        p.grasp={'camera':{'fps':30}};p.camera=None;p.camera_mode='unverified';p.camera_error=''
        p.startup_checks=dict(geometry=True,yolo=True,graspnet=True,camera=False)
        camera=Camera();original=camera.get_frame;stamp=time.time_ns()
        def repeated():
            frames=original();camera.last_frame_timing={'color_system_ns':stamp,'depth_system_ns':stamp};return frames
        camera.get_frame=repeated
        with patch.dict(sys.modules,{'drivers.camera':SimpleNamespace(make_camera=lambda cfg:camera)}):
            with self.assertRaisesRegex(ValueError,'CAMERA_PROBE_TIMEOUT'):p.resume_camera()
        self.assertEqual(p.camera_mode,'failed');self.assertTrue(camera.closed)

    def test_camera_retry_discards_expired_frame_without_hiding_fault(self):
        from unittest.mock import Mock
        p=Planning.__new__(Planning)
        p.cfg={'execution':{'camera_admission_attempts':3}}
        p.compute_metrics={};p.collision=SimpleNamespace(check_deadline=Mock())
        result=object()
        p._frame_once=Mock(side_effect=[ValueError('RGBD_STALE_OR_REPEATED'),result])
        self.assertIs(p.frame(object()),result)
        self.assertEqual(p.compute_metrics['camera_admission_discarded'],1)
        self.assertEqual(p._frame_once.call_count,2)
        p._frame_once=Mock(side_effect=ValueError('ODOMETRY_TIME_RESET'))
        with self.assertRaisesRegex(ValueError,'ODOMETRY_TIME_RESET'):p.frame(object())
        self.assertEqual(p._frame_once.call_count,1)
        p._frame_once=Mock(side_effect=ValueError('RGBD_STALE_OR_REPEATED'))
        with self.assertRaisesRegex(ValueError,'RGBD_STALE'):p.frame(object())
        self.assertEqual(p._frame_once.call_count,3)
        p.collision.check_deadline=Mock(side_effect=ValueError('DEADLINE'))
        p._frame_once=Mock()
        with self.assertRaisesRegex(ValueError,'DEADLINE'):p.frame(object())
        p._frame_once.assert_not_called()

if __name__=='__main__':unittest.main()
