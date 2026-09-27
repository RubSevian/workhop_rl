#!/usr/bin/env python3
"""Planner-estimate-only markers; these never influence navigation."""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker

DEFAULTS = {
    'vehicle_length': 0.30, 'vehicle_width': 0.70, 'adjacent_range': 3.0,
    'path_scale': 0.75, 'path_range': 3.0, 'correspondence_search_radius': 0.55,
    'candidate_paths_total': 0, 'candidate_paths_blocked': 0,
    'candidate_paths_scored': 0, 'selected_group_id': -1,
}

class PlannerVisualization(Node):
    def __init__(self):
        super().__init__('stage4d_planner_visualization')
        self.values = dict(DEFAULTS)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.footprint = self.create_publisher(Marker, '/stage4d/planner_footprint', retained)
        # Sole dynamic Point-LIO TF publisher: together with the existing
        # static map->camera_init, aft_mapped->sensor and sensor->vehicle links
        # it makes planner-frame markers visible in map without ground truth.
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/state_estimation', self.estimated_pose, 10)
        self.domain = self.create_publisher(Marker, '/stage4d/planner_search_domain', retained)
        self.text = self.create_publisher(Marker, '/stage4d/planner_status_marker', retained)
        self.create_subscription(DiagnosticArray, '/local_planner/status', self.status, retained)
        self.create_timer(0.2, self.publish)

    def estimated_pose(self, odometry):
        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.header.frame_id = 'camera_init'
        transform.child_frame_id = 'aft_mapped'
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf.sendTransform(transform)

    def status(self, message):
        for status in message.status:
            if status.name != 'local_planner':
                continue
            for item in status.values:
                try: self.values[item.key] = float(item.value)
                except ValueError: pass

    def marker(self, marker_type, namespace, marker_id=0):
        marker = Marker()
        marker.header.frame_id = 'vehicle'  # Planner estimate frame, deliberately not ground truth.
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace; marker.id = marker_id
        marker.type = marker_type; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish(self):
        length, width = self.values['vehicle_length'], self.values['vehicle_width']
        footprint = self.marker(Marker.CUBE, 'planner_footprint_parameters')
        footprint.pose.position.z = 0.015
        footprint.scale.x, footprint.scale.y, footprint.scale.z = length, width, 0.03
        footprint.color.r, footprint.color.g, footprint.color.b, footprint.color.a = 0.72, 0.18, 0.92, 0.42
        self.footprint.publish(footprint)

        domain = self.marker(Marker.CYLINDER, 'planner_search_domain')
        domain.pose.position.z = 0.006
        domain.scale.x = domain.scale.y = 2.0 * self.values['adjacent_range']; domain.scale.z = 0.012
        domain.color.r, domain.color.g, domain.color.b, domain.color.a = 0.65, 0.15, 0.95, 0.10
        self.domain.publish(domain)

        effective = self.values['correspondence_search_radius'] * self.values['path_scale']
        text = self.marker(Marker.TEXT_VIEW_FACING, 'planner_status')
        text.pose.position.z = 0.65; text.scale.z = 0.16
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 0.93, 0.20, 0.92
        text.text = (f'Planner footprint parameters: {length:.2f} x {width:.2f} m\n'
                     f'range: {self.values["adjacent_range"]:.2f} m  scale: {self.values["path_scale"]:.2f}\n'
                     f'corr radius: {self.values["correspondence_search_radius"]:.2f} canonical, ~{effective:.2f} effective\n'
                     f'candidates: {int(self.values["candidate_paths_total"])} total, '
                     f'{int(self.values["candidate_paths_blocked"])} blocked, {int(self.values["candidate_paths_scored"])} scored\n'
                     f'selected group: {int(self.values["selected_group_id"])}')
        self.text.publish(text)

def main():
    rclpy.init(); node = PlannerVisualization()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
