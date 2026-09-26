#!/usr/bin/env python3
"""Point-LIO odometry adapter; deliberately never subscribes to MuJoCo GT."""
import math
import time
import rclpy
import argparse
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class PointLioStateAdapter(Node):
    def __init__(self, output_frame="map"):
        super().__init__("pointlio_state_adapter")
        self.output_frame = output_frame
        self.pub = self.create_publisher(Odometry, "/state_estimation", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, "/pointlio_ready", qos)
        self.sub = self.create_subscription(Odometry, "/aft_mapped_to_init", self.callback, 10)
        self.last_stamp_ns = None
        self.last_valid_wall = 0.0
        self.timer = self.create_timer(0.1, self.watchdog)
        self.ready_pub.publish(Bool(data=False))

    @staticmethod
    def finite_odom(msg):
        values = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                  msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                  msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                  msg.twist.twist.linear.z, msg.twist.twist.angular.x,
                  msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        return all(math.isfinite(float(v)) for v in values)

    def callback(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if not self.finite_odom(msg) or (self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns):
            return
        self.last_stamp_ns = stamp_ns
        self.last_valid_wall = time.monotonic()
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame
        out.child_frame_id = msg.child_frame_id or "base"
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)
        self.ready_pub.publish(Bool(data=True))

    def watchdog(self):
        ready = self.last_valid_wall > 0.0 and time.monotonic() - self.last_valid_wall <= 0.5
        self.ready_pub.publish(Bool(data=ready))


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--frame", default="map")
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = PointLioStateAdapter(args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
