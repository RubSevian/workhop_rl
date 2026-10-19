#!/usr/bin/env python3
"""In-process TF replay; no rclpy.init, ROS graph, hardware or actuator output."""
import copy
import contextlib
import json
import queue
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
ROOT=Path(__file__).resolve().parents[5]
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from mission.config import load
from mission.scene import BodyPose, SceneBuffer, validate_scene_data
from mission.ros_scene import ROSBodySource, BodyTransformBuffer
from mission.collision import Collision
CFG=load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)


def odom(stamp):
    m=Odometry();m.header.stamp=Time(nanoseconds=stamp).to_msg()
    m.header.frame_id='camera_init';m.child_frame_id='aft_mapped';m.pose.pose.orientation.w=1.
    return m


def tf(parent,child,stamp=0):
    m=TransformStamped();m.header.frame_id=parent;m.child_frame_id=child
    m.header.stamp=Time(nanoseconds=stamp).to_msg();m.transform.rotation.w=1.
    return m


class PairingTests(unittest.TestCase):
    def setUp(self):
        self.now=10**12
        self.cfg=copy.deepcopy(CFG)
        self.buffer=BodyTransformBuffer(self.cfg['frames']['odometry_buffer_size'])
        self.buffer.set_transform_static(tf('map','camera_init'),'test')
        self.buffer.set_transform_static(tf('aft_mapped','vehicle'),'test')
        self.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=self.now)),
                     get_publishers_info_by_topic=lambda topic:[NS(endpoint_gid=[1]*16)])
        with patch('mission.ros_scene.BodyTransformBuffer',return_value=self.buffer),patch('mission.ros_scene.TransformListener'):
            self.source=ROSBodySource(self.node,self.cfg)
        self.scene=SceneBuffer(self.cfg)
        self.clock=patch('mission.ros_scene.time.monotonic_ns',side_effect=lambda:self.now)
        self.clock.start();self.addCleanup(self.clock.stop)

    def live(self,stamp=None,owner='arm'):
        stamp=self.now if stamp is None else stamp
        self.scene.arm([0.]*7,stamp,owner);self.scene.leg([0.]*12,stamp)

    def body(self,stamp):
        return BodyPose(stamp,stamp,'nav','tf',np.eye(4).tolist(),np.eye(4).tolist())

    def test_delayed_tf_with_continuous_new_odometry(self):
        ready=0
        start=self.now
        for i in range(100):
            self.now=start+i*10_000_000;self.live()
            message=odom(self.now);self.source.push(message)
            if i>=8:self.buffer.set_transform(tf('camera_init','aft_mapped',self.now-80_000_000),'test')
            if i%2:continue
            body=self.source.select('nav')
            if body:self.scene.bodies.append(body)
            if i>=8:
                validate_scene_data(self.scene.snapshot(self.now).data(),self.cfg,self.now,'nav','arm')
                ready+=1
                with self.assertRaises(TransformException):self.source.read(message,'nav')
        self.assertEqual(ready,46)
        self.assertLessEqual(len(self.source.pending),self.cfg['frames']['odometry_buffer_size'])
        self.assertAlmostEqual(self.source.diagnostics['selected_age_s'],.08)

    def test_tf_arriving_before_odometry(self):
        self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test')
        self.source.push(odom(self.now))
        self.assertEqual(self.source.select('nav').ros_ns,self.now)

    def test_pair_dropout_cannot_renew_freshness(self):
        self.live();self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test')
        self.source.push(odom(self.now));self.scene.bodies.append(self.source.select('nav'))
        original=self.source.provider.last.mono_ns
        for _ in range(60):
            self.now+=10_000_000;self.live();self.source.push(odom(self.now))
            self.assertIsNone(self.source.select('nav'))
        self.assertEqual(self.source.provider.last.mono_ns,original)
        with self.assertRaisesRegex(ValueError,'STALE:body'):self.scene.snapshot(self.now)

    def test_time_reset_latches_and_duplicate_does_not_refresh(self):
        self.source.push(odom(self.now));self.source.push(odom(self.now))
        self.assertEqual(len(self.source.pending),2)
        self.source.push(odom(self.now-1))
        with self.assertRaisesRegex(ValueError,'TIME_RESET'):self.source.select('nav')
        self.assertEqual(self.source.provider.fault,'ODOMETRY_TIME_RESET')

    def test_history_and_attempts_are_bounded(self):
        for i in range(600):self.source.push(odom(self.now+i))
        self.assertEqual(len(self.source.pending),512)
        self.now+=2_000_000_000
        self.assertIsNone(self.source.select('nav'));self.assertEqual(len(self.source.pending),0)
        self.now+=1_000_000_000
        for i in range(20):
            self.source.push(odom(self.now-20+i))
            self.buffer.set_transform(tf('camera_init','aft_mapped',self.now-20+i),'test')
        with patch.object(self.buffer,'lookup_transform',return_value=tf('map','vehicle',self.now)),patch.object(self.source,'read',side_effect=TransformException('delayed')) as read:
            self.assertIsNone(self.source.select('nav'))
            self.assertEqual(read.call_count,self.cfg['frames']['tf_lookup_attempts_per_tick'])

    def test_old_coherent_geometry_with_fresh_joint_streams(self):
        stamp=self.now-350_000_000
        self.live(stamp);self.scene.bodies.append(self.body(stamp));self.live()
        snapshot=self.scene.snapshot(self.now)
        self.assertEqual(snapshot.feedback_ns,stamp)
        self.assertEqual(snapshot.live_feedback_ns,self.now)
        validate_scene_data(snapshot.data(),self.cfg,self.now,'nav','arm')
        collision=Collision.__new__(Collision);collision.cfg=self.cfg;collision.environment_enabled=False
        collision.validated_scene(dict(scene_snapshot=snapshot.data(),navigation_session_id='nav',owner_session_id='arm',map_body=np.eye(4).tolist()),False)
        self.now+=251_000_000
        with self.assertRaisesRegex(ValueError,'STALE:live_arm'):self.scene.snapshot(self.now)
        with self.assertRaisesRegex(ValueError,'STALE'):validate_scene_data(snapshot.data(),self.cfg,self.now,'nav','arm')

    def test_history_does_not_bypass_skew_or_owner_change(self):
        stamp=self.now-350_000_000
        self.live(stamp-60_000_000);self.scene.bodies.append(self.body(stamp));self.live()
        with self.assertRaisesRegex(ValueError,'SKEW'):self.scene.snapshot(self.now)
        self.scene=SceneBuffer(self.cfg);self.live(stamp);self.scene.bodies.append(self.body(stamp));self.live(owner='new')
        with self.assertRaisesRegex(ValueError,'EPOCH'):self.scene.snapshot(self.now)

    def test_modelled_body_epoch_and_tf_mismatch_still_rejected(self):
        self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test');self.source.push(odom(self.now))
        self.source.select('nav');self.now+=10_000_000
        self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test');self.source.push(odom(self.now))
        with self.assertRaisesRegex(ValueError,'EPOCH'):self.source.select('different-nav')

    def test_both_node_consumers_use_the_pair_buffer(self):
        from grasp_perception import Perception
        from grasp_mission import Coordinator
        output=[];observations={}
        backend=NS(sensors_ready=lambda:True,startup_checks=dict(geometry=True,yolo=True,graspnet=True,camera=True),camera_mode='standby')
        perception=NS(cfg=self.cfg,body_source=self.source,scene_buffer=self.scene,
            leg_reader=NS(status=lambda:dict(mode='test',projected=0,fallback=0)),
            navigation_session='nav',lock=contextlib.nullcontext(),scene_error='',backend=backend,
            reason='READY',busy=False,last_compute={},startup_checks=backend.startup_checks,
            health=NS(publish=lambda msg:output.append(json.loads(msg.data))),completed=queue.Queue(),
            get_logger=lambda:NS(info=lambda text:None))
        with patch('mission.ros_scene.BodyTransformBuffer',return_value=self.buffer),patch('mission.ros_scene.TransformListener'):
            coordinator_source=ROSBodySource(self.node,self.cfg)
        coordinator_scene=SceneBuffer(self.cfg)
        engine=NS(inputs={'nav':{'session_id':'nav'}},received={},state='IDLE',
            observe=lambda name,data,now:observations.update({name:data}))
        coordinator=NS(cfg=self.cfg,body_source=coordinator_source,scene_buffer=coordinator_scene,
            engine=engine,scene_error='',get_logger=lambda:NS(info=lambda text:None))
        start=self.now
        for i in range(100):
            self.now=start+i*10_000_000
            self.live();coordinator_scene.arm([0.]*7,self.now,'arm');coordinator_scene.leg([0.]*12,self.now)
            msg=odom(self.now)
            Perception.odom(perception,msg);Coordinator.base(coordinator,msg)
            if i>=8:self.buffer.set_transform(tf('camera_init','aft_mapped',self.now-80_000_000),'test')
            if i%2:continue
            Perception.tick(perception);Coordinator.refresh_scene(coordinator)
            if i>=8:
                self.assertTrue(output[-1]['ready'],output[-1])
                self.assertEqual(coordinator.scene_error,'')
                validate_scene_data(observations['scene'],self.cfg,self.now,'nav','arm')
        self.assertTrue(output[-1]['tf_pair']['selected_stamp_ns'])

    def test_odometry_tf_pose_mismatch_is_reported_and_not_adopted(self):
        self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test')
        msg=odom(self.now);msg.pose.pose.position.x=1.
        self.source.push(msg)
        self.assertIsNone(self.source.select('nav'))
        self.assertEqual(self.source.diagnostics['candidate_reason'],'TF_ODOMETRY_REVISION_MISMATCH')

    def test_interpolated_tf_is_not_a_received_odometry_pair(self):
        old=self.now-100_000_000
        self.buffer.set_transform(tf('camera_init','aft_mapped',old),'test')
        self.source.push(odom(old))
        self.source.push(odom(self.now-50_000_000))
        newer=tf('camera_init','aft_mapped',self.now);newer.transform.translation.x=.08
        self.buffer.set_transform(newer,'test')
        # tf2 can interpolate this absent sample, but its pose is not the
        # measured zero translation. Select the actual older matching pair.
        self.assertAlmostEqual(self.buffer.lookup_transform('map','aft_mapped',Time(nanoseconds=self.now-50_000_000)).transform.translation.x,.04)
        self.assertEqual(self.source.select('nav').ros_ns,old)

    def test_repeated_stamp_corrected_odometry_is_retained(self):
        wrong=odom(self.now);wrong.pose.pose.position.x=.05
        self.source.push(wrong);self.source.push(odom(self.now))
        self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test')
        self.assertEqual(self.source.select('nav').map_vehicle[0][3],0.)

    def test_mismatched_new_revision_keeps_original_verified_pair_until_expiry(self):
        self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test');self.source.push(odom(self.now))
        body=self.source.select('nav');self.scene.bodies.append(body);self.live()
        self.now+=100_000_000;self.live()
        wrong=odom(self.now);wrong.pose.pose.position.x=.05
        self.source.push(wrong);self.buffer.set_transform(tf('camera_init','aft_mapped',self.now),'test')
        self.assertIsNone(self.source.select('nav'))
        self.assertEqual(self.scene.snapshot(self.now).body,body)
        self.assertGreater(self.source.diagnostics['candidate_position_error_m'],.001)
        self.now+=401_000_000;self.live()
        self.assertIsNone(self.source.select('nav'))
        with self.assertRaisesRegex(ValueError,'STALE:body'):self.scene.snapshot(self.now)

    def test_changed_tf_revision_in_buffer_can_fall_back_to_older_candidate(self):
        old=self.now-20_000_000
        for stamp in (old,self.now):
            self.source.push(odom(stamp));self.buffer.set_transform(tf('camera_init','aft_mapped',stamp),'test')
        original=self.source.read
        from mission.ros_scene import PairCandidateMismatch
        def lookup(msg,session):
            if msg.header.stamp==Time(nanoseconds=self.now).to_msg():raise PairCandidateMismatch('TF_ODOMETRY_REVISION_MISMATCH')
            return original(msg,session)
        with patch.object(self.source,'read',side_effect=lookup):
            self.assertEqual(self.source.select('nav').ros_ns,old)

if __name__=='__main__':unittest.main()
