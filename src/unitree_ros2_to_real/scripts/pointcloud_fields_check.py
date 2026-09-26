#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Check(Node):
    def __init__(self):
        super().__init__("pointcloud_fields_check")
        self.sub = self.create_subscription(PointCloud2, "/unilidar/cloud", self.cb, 10)

    def cb(self, msg):
        print(f"frame_id={msg.header.frame_id} width={msg.width} height={msg.height} point_step={msg.point_step}")
        print("fields=" + ",".join(f"{f.name}:{f.datatype}@{f.offset}" for f in msg.fields))
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Check()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
