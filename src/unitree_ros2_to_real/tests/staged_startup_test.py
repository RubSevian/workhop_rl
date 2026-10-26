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
from mission.planning import Planning, CameraPump

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
    def test_controlled_stop_closes_only_task_inference_and_keeps_geometry(self):
        from unittest.mock import Mock
        p=Planning.__new__(Planning);p._closed=False
        p.worker=SimpleNamespace(close=Mock(),is_alive=lambda:False)
        p.camera=SimpleNamespace(close=Mock());camera=p.camera
        p.camera_mode='active';p.yolo=object();p.last_detection=object()
        p.startup_checks=dict.fromkeys(('geometry','yolo','graspnet','camera'),True)
        p.stop_task_perception()
        p.worker.close.assert_called_once_with(force=True);camera.close.assert_called_once()
        self.assertIsNone(p.yolo);self.assertIsNone(p.camera);self.assertTrue(p.sensors_ready())
        p.startup_checks['geometry']=False;self.assertFalse(p.sensors_ready())

    def perception_mock(self):
        from grasp_perception import Perception
        from unittest.mock import Mock
        import queue,threading
        node=Perception.__new__(Perception)
        node.cfg={'execution':{'heartbeat_ttl_s':.25}}
        node.controlled_stop=False;node.vision_cancel_complete=False;node.vision_reloading=False
        node.r3_output_stopped=True;node.r3_phase='';node.r3_state='';node.last_r3_stamp=0;node.cancel_thread=None
        node.backend=SimpleNamespace(collision=SimpleNamespace(work_deadline_ns=None),stop_task_perception=Mock(),
            startup_checks=dict.fromkeys(('geometry','yolo','graspnet','camera'),True),_preload_perception=Mock())
        node.busy=False;node.stop_event=threading.Event();node.last_request=0;node.request_session=''
        from mission.compute_cancel import ComputeCancellation
        node.compute_cancel=ComputeCancellation()
        node.jobs=queue.Queue(1);node.completed=queue.Queue(1)
        node.count_publishers=Mock(return_value=1);node.get_logger=Mock(return_value=Mock())
        return node

    def test_perception_x_stale_status_and_home_permissions(self):
        import json
        n=self.perception_mock()
        def status(state,phase,stamp):
            n.r3_status(SimpleNamespace(data=json.dumps(dict(state=state,phase=phase,observed_ns=stamp))))
        status('CONTROLLED_STOP','ARM_RETURN_HOME',time.monotonic_ns()-int(1e9))
        self.assertFalse(n.controlled_stop);n.backend.stop_task_perception.assert_not_called()
        n.busy=True;status('CONTROLLED_STOP','ARM_RETURN_HOME',time.monotonic_ns());n.cancel_thread.join(1.)
        self.assertTrue(n.vision_cancel_complete);self.assertTrue(n.controlled_stop)
        self.assertEqual(n.backend.collision.cancel_reason,'CONTROLLED_STOP');self.assertIsNone(n.backend.collision.work_deadline_ns);n.backend.stop_task_perception.assert_called_once()
        n.busy=False
        def request(i,operation):
            n.request(SimpleNamespace(data=json.dumps(dict(id=i,operation=operation,session_id='session',observed_ns=time.monotonic_ns()))))
        request(1,'detect');self.assertTrue(n.jobs.empty())
        request(2,'home');self.assertEqual(n.jobs.get_nowait()['operation'],'home');n.busy=False
        status('CONTROLLED_STOP','PD_CAPTURE',time.monotonic_ns());request(3,'home');self.assertTrue(n.jobs.empty())
        # B never starts model reload, and duplicate/stale status never repeats cancel.
        status('EMERGENCY_FAULT','FAULT',time.monotonic_ns());n.resume_task_perception();n.backend._preload_perception.assert_not_called()
        n.backend.stop_task_perception.assert_called_once()

    def test_perception_repreload_only_after_output_stop_and_new_fault_wins(self):
        n=self.perception_mock();n.controlled_stop=n.vision_cancel_complete=True
        n.request_session='retired';n.last_request=12
        n.r3_state='SYSTEM_HOLD';n.last_r3_stamp=time.monotonic_ns()
        n.backend._preload_perception.side_effect=lambda report:n.backend.startup_checks.update(dict.fromkeys(('yolo','graspnet','camera'),True))
        n.resume_task_perception();n.backend._preload_perception.assert_called_once()
        self.assertFalse(n.controlled_stop);self.assertFalse(n.vision_reloading);self.assertEqual(n.request_session,'')
        n=self.perception_mock();n.controlled_stop=n.vision_cancel_complete=True
        n.r3_state='SYSTEM_HOLD';n.last_r3_stamp=time.monotonic_ns()-int(1e9)
        n.resume_task_perception();n.backend._preload_perception.assert_not_called()
        n.last_r3_stamp=time.monotonic_ns()
        def preload(report):n.r3_state='EMERGENCY_FAULT'
        n.backend._preload_perception.side_effect=preload
        n.resume_task_perception();self.assertTrue(n.controlled_stop);n.backend.stop_task_perception.assert_called_once()

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
        node.cfg={'startup':{'shutdown_timeout_s':1.},'execution':{}};node.backend=None;node.startup_checks={}
        from mission.compute_cancel import ComputeCancellation
        node.compute_cancel=ComputeCancellation()
        node.stop_event=threading.Event();node.jobs=queue.Queue(1);node.completed=queue.Queue(1)
        node.get_logger=lambda:SimpleNamespace(info=lambda msg:ready.set(),error=lambda msg:None,warning=lambda msg:None)
        with patch.dict(sys.modules,{'mission.planning':SimpleNamespace(Planning=Backend)}),patch.object(Node,'destroy_node',return_value=True):
            node.worker_thread=threading.Thread(target=node.work)
            node.worker_thread.start();self.assertTrue(ready.wait(1.))
            node.destroy_node()
        self.assertFalse(node.worker_thread.is_alive());self.assertIsNone(node.backend)
        # Idle work can reach finally before destroy_node closes the inference worker.
        # Both resources must close and the thread must be joined, in either order.
        self.assertCountEqual(calls,['inference stopped','backend closed'])

    def test_shutdown_unblocks_inference_before_join_and_is_repeatable(self):
        from grasp_perception import Perception
        from rclpy.node import Node
        import threading,queue
        # A blocked inference call must be released by close, not by join timeout.
        entered=threading.Event();released=threading.Event();calls=[];warnings=[]
        def close(force):
            self.assertTrue(force)
            if not released.is_set():calls.append('inference stopped');released.set()
        backend=SimpleNamespace(collision=SimpleNamespace(),worker=SimpleNamespace(close=close))
        node=Perception.__new__(Perception)
        node.backend=backend;node.cfg={'startup':{'shutdown_timeout_s':1.}}
        node.stop_event=threading.Event();node.jobs=queue.Queue();node.completed=queue.Queue()
        node.get_logger=lambda:SimpleNamespace(warning=warnings.append)
        def blocked_work():
            entered.set()
            if released.wait(2.):calls.append('backend closed')
        node.worker_thread=threading.Thread(target=blocked_work)
        node.worker_thread.start();self.assertTrue(entered.wait(1.))
        with patch.object(Node,'destroy_node',return_value=True):
            node.destroy_node();node.destroy_node()
        self.assertFalse(node.worker_thread.is_alive());self.assertIsNone(node.backend)
        self.assertEqual(warnings,[])
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
        context.launch_configurations.update(workspace=str(root),physical='true',test_mode='ready_observe',start_arm_owner='true',
            architecture_monitor='true',network_interface='enP8p1s0',mission_config_path=str(root/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'))
        # Test this checkout's launch/config pair, independently of the active
        # installed baseline. Actions are only constructed, never executed.
        prefix=Path(self.enterContext(tempfile.TemporaryDirectory()))
        package_lib=prefix/'lib/unitree_legged_real'
        package_lib.mkdir(parents=True)
        (package_lib/'mission').symlink_to(SCRIPTS/'mission',target_is_directory=True)
        with patch.object(module,'get_package_prefix',return_value=str(prefix)),patch.object(module,'get_package_share_directory',return_value=str(SCRIPTS.parent)):
            actions=module._launch(context)
        self.assertEqual(len(actions),3)
        self.assertIsInstance(actions[0],Node) # diagnostic monitor starts before preload
        self.assertIn('architecture_monitor.py',str(actions[0].node_executable))
        self.assertIsInstance(actions[1],RegisterEventHandler);self.assertIsInstance(actions[2],Node)
        handler=actions[1].event_handler
        def output(data):
            event=ProcessIO(action=actions[2],name='preload',cmd=[],cwd=None,env=None,pid=1,text=data,fd=1)
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
        # Probe lifecycle is tested at the process facade boundary here;
        # real spawn/IPC and blocked SDK calls have their own camera_process_test.
        def make_camera(cfg, start_timeout, stop_timeout, mode):
            c=Camera();cameras.append(c);return CameraPump(c,stop_timeout,mode)
        with patch('mission.camera_process.CameraProcess',side_effect=make_camera):
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
        with patch('mission.camera_process.CameraProcess',
                   side_effect=lambda cfg,start_timeout,stop_timeout,mode:CameraPump(camera,stop_timeout,mode)):
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
