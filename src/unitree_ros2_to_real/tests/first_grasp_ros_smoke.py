#!/usr/bin/env python3
"""Integration fixture ONLY: fake ROS sensors/planner/arm, no SDK/camera.

Explicitly uses synthetic readiness in memory; it never qualifies real hardware.
Requires domain228 and loopback. Not installed as an operator executable.
"""
import json
import os
from pathlib import Path
import sys
import time
import uuid
assert os.environ.get('ROS_DOMAIN_ID')=='228' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
sys.path.insert(0,str(Path(__file__).resolve().parent))
from first_grasp_mission_test import Fixture,CFG,SCRIPTS,ROOT
sys.path.insert(0,str(SCRIPTS))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from mission.scene import FrameChain
from grasp_mission import Coordinator
from mission.core import Mission,TERMINAL

class Fake(Node):
    def __init__(self):
        super().__init__('synthetic_grasp_fixture')
        self.fixture=Fixture();self.status={};self.seen=[];self.owner_cancels=0
        self.nav_ticks=self.stop_ticks=0;self.zero_since=None
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        transforms=[]
        for parent,child,x in [('map','camera_init',0.),('aft_mapped','vehicle',-.3)]:
            transform=TransformStamped();transform.header.stamp=self.get_clock().now().to_msg()
            transform.header.frame_id=parent;transform.child_frame_id=child
            transform.transform.translation.x=x;transform.transform.rotation.w=1.
            transforms.append(transform)
        self.static.sendTransform(transforms)
        self.pubs={n:self.create_publisher(String,t,1) for n,t in dict(r3='/go2/locomotion_status',nav='/go2/navigation/status',owner='/rars01/commissioning/state',perception='/rars01/mission/perception_health').items()}
        self.leg_pub=self.create_publisher(LowState,'/lowstate',1)
        self.odometry=self.create_publisher(Odometry,'/state_estimation',10)
        self.computed=self.create_publisher(String,'/rars01/mission/computed',1)
        self.create_subscription(String,'/go2/mission/status',self.observe,1)
        self.create_subscription(String,'/rars01/mission/compute',self.compute,1)
        self.create_subscription(String,'/rars01/mission/trajectory',self.trajectory,1)
        self.create_service(Trigger,'/rars01/mission/cancel_manipulation',self.cancel)
        self.create_service(Trigger,'/rars01/control/return_home',self.home)
        self.create_timer(.02,self.tick)
    def observe(self,msg):
        self.status=json.loads(msg.data)
        if not self.seen or self.seen[-1]!=self.status['state']:self.seen.append(self.status['state'])
    def compute(self,msg):
        data=json.loads(msg.data);self.fixture.now=time.monotonic()
        if data['operation']=='scan_pose':
            assert self.fixture.r3['phase']=='RL_ZERO'
            assert self.zero_since is not None and time.monotonic()-self.zero_since>=5.
        self.computed.publish(String(data=json.dumps(self.fixture.compute(data))))
    def trajectory(self,msg):self.fixture.effect('trajectory',json.loads(msg.data))
    def cancel(self,req,res):self.owner_cancels+=1;res.success=True;return res
    def home(self,req,res):res.success=True;return res
    def tick(self):
        f=self.fixture
        if self.status.get('state')=='WAIT_GOAL' and f.nav['goal']['revision']==0:
            f.nav['goal']=dict(revision=1,stamp_ns=self.get_clock().now().nanoseconds,state='REQUESTED')
        elif self.status.get('state')=='NAVIGATING':
            self.nav_ticks+=1
            f.r3.update(phase='RL_ACTIVE',motion_command=[.1,0,0])
            f.nav['goal']['state']='ACTIVE' if self.nav_ticks<4 else 'SUCCEEDED'
        elif self.status.get('state')=='BASE_SETTLE':
            self.stop_ticks+=1
            if self.stop_ticks>=5:
                f.r3.update(phase='RL_ZERO',motion_command=[0,0,0])
                if self.zero_since is None:self.zero_since=time.monotonic()
        for name,pub in self.pubs.items():
            if name=='owner':f.owner['frame_received_ns']=time.monotonic_ns()
            pub.publish(String(data=json.dumps(dict(getattr(f,name),observed_ns=time.monotonic_ns()))))
        state=LowState();state.tick=int(time.monotonic()*1000)%2**32
        for m,q in zip(state.motor_state,f.legs['q']):m.q=q
        self.leg_pub.publish(state)
        odom=Odometry();odom.header.stamp=self.get_clock().now().to_msg();odom.header.frame_id='camera_init';odom.child_frame_id='aft_mapped';odom.pose.pose.orientation.w=1.
        transform=TransformStamped();transform.header=odom.header;transform.child_frame_id=odom.child_frame_id
        transform.transform.rotation.w=1.;self.tf.sendTransform(transform)
        self.odometry.publish(odom)

def main():
    rclpy.init(args=['--ros-args','-p','workspace:='+str(ROOT),'-p','mission_config_path:='+str(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml')])
    executor=SingleThreadedExecutor();fake=Fake();executor.add_node(fake)
    try:
        for branch,end in [('success','MISSION_COMPLETED_UNVERIFIED'),('not_found','OBJECT_NOT_FOUND'),('ik_fail','GRASP_UNREACHABLE')]:
            fake.fixture=Fixture(branch);fake.status={};fake.seen=[]
            fake.nav_ticks=fake.stop_ticks=0;fake.zero_since=None
            coordinator=Coordinator()
            # Test fixture only: no robot process exists in this isolated graph.
            coordinator.physical=True;coordinator.blockers=[]
            coordinator.cfg['frames']['vehicle_to_mount']=coordinator.cfg['frames']['vehicle_to_mount_candidate']
            coordinator.body_source.provider.chain=FrameChain(coordinator.cfg)
            coordinator.cfg['search']['settle_s']=.05
            executor.add_node(coordinator)
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
                assert coordinator.count_publishers('/lowcmd')==0
                if fake.status.get('state') in TERMINAL:break
            assert fake.status.get('state')==end,(fake.status,coordinator.engine.inputs,coordinator.gids,{k:[list(x.endpoint_gid) for x in coordinator.get_publishers_info_by_topic(v)] for k,v in coordinator.topics.items()})
            assert fake.status['gripper_observation']['informational_only']
            assert fake.fixture.owner['arm_home_ready']
            assert not fake.fixture.owner['gripper_hold_confirmed']
            if branch=='success':assert fake.fixture.owner['gripper_command_active']
            print(json.dumps(dict(scenario=branch,result=end,states=fake.seen,plans=fake.fixture.commands,physical_actuator_commands=0)),flush=True)
            executor.remove_node(coordinator);coordinator.destroy_node()
            # Let graph removals settle before another node with the same topics.
            until=time.monotonic()+.3
            while time.monotonic()<until:executor.spin_once(timeout_sec=.02)
    finally:
        executor.remove_node(fake);fake.destroy_node();executor.shutdown();rclpy.try_shutdown()
if __name__=='__main__':main()
