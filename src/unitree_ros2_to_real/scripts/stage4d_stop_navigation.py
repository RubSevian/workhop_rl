#!/usr/bin/env python3
"""Navigation-only emergency stop: FAR disables navigation and a zero command refreshes RL."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TwistStamped

def main():
    rclpy.init(); node = Node('stage4d_stop_navigation')
    goal = node.create_publisher(PointStamped, '/goal_point', 10)
    cmd = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    invalid = PointStamped(); invalid.header.frame_id = ''
    zero = TwistStamped(); zero.header.stamp = node.get_clock().now().to_msg(); zero.header.frame_id = 'vehicle'
    for _ in range(3):
        goal.publish(invalid); cmd.publish(zero); rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
