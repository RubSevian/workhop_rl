#!/usr/bin/env python3
"""Diagnostics-only Point-LIO trajectory; /path stays owned by localPlanner."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathMirror(Node):
    def __init__(self):
        super().__init__('stage4d_odometry_path')
        self.path = Path()
        self.pub = self.create_publisher(Path, '/point_lio/path', 10)
        self.create_subscription(Odometry, '/state_estimation', self.on_odom, 10)
    def on_odom(self, odom):
        pose = PoseStamped(); pose.header = odom.header; pose.pose = odom.pose.pose
        self.path.header = odom.header; self.path.poses.append(pose)
        self.path.poses = self.path.poses[-2000:]
        self.pub.publish(self.path)
def main():
    rclpy.init(); node = PathMirror()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); try_shutdown()
if __name__ == '__main__': main()
