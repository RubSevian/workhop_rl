#!/usr/bin/env python3
"""FAR-owned emergency stop: malformed goal is rejected and FAR publishes false."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
def main():
 rclpy.init(); n=Node('stage4d_stop_navigation'); p=n.create_publisher(PointStamped,'/goal_point',10); m=PointStamped(); m.header.frame_id=''; p.publish(m); rclpy.spin_once(n,timeout_sec=.2); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
