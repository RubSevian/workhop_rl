#!/usr/bin/env python3
"""Manual Stage-4 publisher for the exact navigation-to-RL command contract."""

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


COMMANDS = {
    "zero": (0.0, 0.0, 0.0),
    "forward": (0.35, 0.0, 0.0),
    "backward": (-0.25, 0.0, 0.0),
    "left": (0.0, 0.20, 0.0),
    "right": (0.0, -0.20, 0.0),
    "yaw_left": (0.0, 0.0, 0.30),
    "yaw_right": (0.0, 0.0, -0.30),
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--activate", action="store_true", help="publish /navigation_active=true")
    args = parser.parse_args()
    if args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("--duration and --rate must be positive")

    rclpy.init()
    node = rclpy.create_node("manual_twist_test")
    cmd_pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)
    gate_pub = node.create_publisher(Bool, "/navigation_active", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if args.activate:
        gate_pub.publish(Bool(data=True))

    vx, vy, wz = COMMANDS[args.command]
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.angular.z = wz
            cmd_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / args.rate)
    finally:
        zero = TwistStamped()
        zero.header.stamp = node.get_clock().now().to_msg()
        zero.header.frame_id = "base_link"
        cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
