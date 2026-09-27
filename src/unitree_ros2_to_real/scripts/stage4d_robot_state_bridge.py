#!/usr/bin/env python3
"""Pass MuJoCo's named joints into an isolated RViz-only robot TF tree."""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

JOINT_NAMES = (
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
    'gripper_left_joint', 'gripper_right_joint',
)

class Bridge(Node):
    def __init__(self):
        super().__init__('stage4d_robot_state_bridge')
        self.js_pub = self.create_publisher(JointState, '/stage4d/joint_states', 10)
        self.tf = TransformBroadcaster(self)
        self.latest_js = None
        self.latest_odom = None
        self.warned_bad_joint_state = False
        self.create_subscription(JointState, '/go2/motor_state', self.on_js, 10)
        self.create_subscription(Odometry, '/sim/ground_truth_odom', self.on_odom, 10)
        self.create_timer(0.02, self.tick)

    def on_js(self, message):
        positions = dict(zip(message.name, message.position))
        if set(positions) != set(JOINT_NAMES) or len(message.name) != len(JOINT_NAMES):
            if not self.warned_bad_joint_state:
                self.get_logger().error('Rejecting /go2/motor_state: expected exactly the 20 named MuJoCo joints')
                self.warned_bad_joint_state = True
            return
        velocities = dict(zip(message.name, message.velocity)) if len(message.velocity) == len(message.name) else {}
        efforts = dict(zip(message.name, message.effort)) if len(message.effort) == len(message.name) else {}
        # Canonical output order makes diagnostics deterministic; values remain
        # name-derived, never inferred from a MuJoCo index in this bridge.
        out = JointState()
        out.header = message.header
        out.name = list(JOINT_NAMES)
        out.position = [positions[name] for name in JOINT_NAMES]
        out.velocity = [velocities.get(name, 0.0) for name in JOINT_NAMES]
        out.effort = [efforts.get(name, 0.0) for name in JOINT_NAMES]
        self.latest_js = out
        self.warned_bad_joint_state = False

    def on_odom(self, message):
        self.latest_odom = message

    def tick(self):
        if self.latest_js is not None:
            self.js_pub.publish(self.latest_js)
        if self.latest_odom is None:
            return
        odom = self.latest_odom
        transform = TransformStamped()
        transform.header = odom.header
        transform.header.frame_id = odom.header.frame_id or 'map'
        transform.child_frame_id = 'sim_visual_base'
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

def main():
    rclpy.init(); node = Bridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
