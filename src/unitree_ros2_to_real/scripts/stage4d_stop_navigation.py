#!/usr/bin/env python3
"""Navigation-only Stage4D stop: cancel FAR, then refresh a safe zero command."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty


def main():
    rclpy.init()
    node = Node('stage4d_stop_navigation')
    cancel = node.create_publisher(Empty, '/navigation_cancel', 10)
    cmd = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    zero = TwistStamped()
    zero.header.frame_id = 'vehicle'
    # A small bounded refresh helps transient-local discovery without relying on
    # the former malformed /goal_point compatibility path.
    for _ in range(3):
        zero.header.stamp = node.get_clock().now().to_msg()
        cancel.publish(Empty())
        cmd.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
