#!/usr/bin/env python3
"""Bridge MuJoCo state to the URDF model used by RViz RobotModel."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

LEG_JOINTS=['FL_hip_joint','FL_thigh_joint','FL_calf_joint','FR_hip_joint','FR_thigh_joint','FR_calf_joint','RL_hip_joint','RL_thigh_joint','RL_calf_joint','RR_hip_joint','RR_thigh_joint','RR_calf_joint']
class Bridge(Node):
 def __init__(self):
  super().__init__('stage4d_robot_state_bridge'); self.js_pub=self.create_publisher(JointState,'/joint_states',10); self.tf=TransformBroadcaster(self); self.latest_js=None; self.latest_odom=None
  self.create_subscription(JointState,'/go2/motor_state',self.on_js,10); self.create_subscription(Odometry,'/sim/ground_truth_odom',self.on_odom,10); self.create_timer(.02,self.tick)
 def on_js(self,m): self.latest_js=m
 def on_odom(self,m): self.latest_odom=m
 def tick(self):
  if self.latest_js:
   out=JointState(); out.header=self.latest_js.header; out.name=LEG_JOINTS[:len(self.latest_js.position)]; out.position=list(self.latest_js.position); out.velocity=list(self.latest_js.velocity); out.effort=list(self.latest_js.effort); self.js_pub.publish(out)
  if self.latest_odom:
   o=self.latest_odom; t=TransformStamped(); t.header=o.header; t.header.frame_id=o.header.frame_id or 'camera_init'; t.child_frame_id='base'; t.transform.translation.x=o.pose.pose.position.x; t.transform.translation.y=o.pose.pose.position.y; t.transform.translation.z=o.pose.pose.position.z; t.transform.rotation=o.pose.pose.orientation; self.tf.sendTransform(t)
def main():
 rclpy.init(); n=Bridge()
 try: rclpy.spin(n)
 except KeyboardInterrupt: pass
 finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
