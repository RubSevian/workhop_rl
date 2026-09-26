#!/usr/bin/env python3
"""Freshness/ownership gate for the complete Stage4D navigation chain."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class Stage4DReadiness(Node):
    def __init__(self):
        super().__init__('stage4d_readiness')
        self.last = {}
        self.pub = self.create_publisher(Bool, '/stage4d/navigation_ready', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        types = {'/state_estimation': Odometry, '/cloud_registered': PointCloud2, '/terrain_map': PointCloud2, '/terrain_map_ext': PointCloud2, '/pointlio_ready': Bool}
        for topic, typ in types.items():
            self.create_subscription(typ, topic, lambda m,t=topic: self.last.__setitem__(t,time.monotonic()), 1)
        self.timer = self.create_timer(0.5, self.tick)
    def tick(self):
        # ROS topic types are intentionally discovered through graph counts; callbacks below are
        # installed by typed subscriptions in the launch process.  Publisher ownership is checked
        # independently so readiness cannot be asserted by a stale message alone.
        required = ('/state_estimation','/cloud_registered','/terrain_map','/terrain_map_ext','/pointlio_ready')
        fresh = all(time.monotonic()-self.last.get(t,0) < 1.5 for t in required)
        owned = all(self.get_publishers_info_by_topic(t) for t in ('/way_point','/path','/cmd_vel','/navigation_active'))
        self.pub.publish(Bool(data=bool(fresh and owned)))
def main():
    rclpy.init(); n=Stage4DReadiness()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
