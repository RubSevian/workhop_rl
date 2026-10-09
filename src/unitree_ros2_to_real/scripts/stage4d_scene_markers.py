#!/usr/bin/env python3
"""Visualize named Stage4D MuJoCo environment geoms by parsing the scene XML."""
import argparse
import xml.etree.ElementTree as ET
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

class SceneMarkers(Node):
    def __init__(self, scene):
        super().__init__('stage4d_scene_markers')
        self.scene = scene
        self.pub = self.create_publisher(MarkerArray, '/stage4d/scene_markers', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(1.0, self.publish)
    def publish(self):
        try: root = ET.parse(self.scene).getroot()
        except (OSError, ET.ParseError) as exc:
            self.get_logger().error(f'Cannot parse Stage4D scene: {exc}'); return
        markers = MarkerArray(); index = 0
        for geom in root.findall('.//geom'):
            name = geom.get('name', '')
            if not (name.startswith('stage4d_landmark_') or name.startswith('stage4d_obstacle_') or name.startswith('stage4d_corridor_')): continue
            marker = Marker(); marker.header.frame_id = 'map'; marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'stage4d_scene'; marker.id = index; index += 1; marker.action = Marker.ADD
            marker.type = Marker.CYLINDER if geom.get('type') == 'cylinder' else Marker.CUBE
            pos = [float(x) for x in geom.get('pos', '0 0 0').split()]; size = [float(x) for x in geom.get('size', '1 1 1').split()]
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos
            marker.pose.orientation.w = 1.0
            if marker.type == Marker.CYLINDER: marker.scale.x = marker.scale.y = 2*size[0]; marker.scale.z = 2*size[1]
            else: marker.scale.x, marker.scale.y, marker.scale.z = 2*size[0], 2*size[1], 2*size[2]
            rgba = [float(x) for x in geom.get('rgba', '0.8 0.8 0.8 0.35').split()]
            marker.color.r, marker.color.g, marker.color.b = rgba[:3]; marker.color.a = 0.35
            markers.markers.append(marker)
        self.pub.publish(markers)
def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--scene', required=True); args, _ = parser.parse_known_args()
    rclpy.init(); node = SceneMarkers(args.scene)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); try_shutdown()
if __name__ == '__main__': main()
