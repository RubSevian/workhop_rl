#!/usr/bin/env python3
"""Publish only the physically travelled MuJoCo trajectory for Stage4D RViz.

Robot geometry is intentionally not published here: RobotModel renders the
full combined URDF from /stage4d/joint_states and sim_visual_* TF.
"""
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

class GroundTruthPath(Node):
    def __init__(self):
        super().__init__('stage4d_ground_truth_path')
        self.path = Path()
        self.last = None
        self.pub = self.create_publisher(Path, '/stage4d/ground_truth_path', 10)
        self.create_subscription(Odometry, '/sim/ground_truth_odom', self.on_odom, 10)

    def on_odom(self, odom):
        pose = odom.pose.pose
        if self.last is not None:
            dx = pose.position.x - self.last.position.x
            dy = pose.position.y - self.last.position.y
            # Avoid repeating a stationary pose at the raw simulator rate.
            if math.hypot(dx, dy) < 0.015:
                return
        stamped = PoseStamped()
        stamped.header = odom.header
        stamped.pose = pose
        self.path.header = odom.header
        self.path.poses.append(stamped)
        self.path.poses = self.path.poses[-5000:]
        self.last = pose
        self.pub.publish(self.path)

def main():
    rclpy.init(); node = GroundTruthPath()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); try_shutdown()

if __name__ == '__main__': main()
