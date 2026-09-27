#!/usr/bin/env python3
"""Terrain statistics plus truthful, visualization-only planner terrain splits."""
import math
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class TerrainDiagnostics(Node):
    def __init__(self):
        super().__init__('stage4d_terrain_diagnostics')
        self.sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(DiagnosticArray, '/stage4d/terrain_diagnostics', retained)
        # RViz defaults to RELIABLE; source terrain input remains BEST_EFFORT.
        self.viz_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.local_obstacle_pub = self.create_publisher(PointCloud2, '/stage4d/localplanner_obstacles_viz', self.viz_qos)
        self.far_free_pub = self.create_publisher(PointCloud2, '/stage4d/far_free_viz', self.viz_qos)
        self.far_obstacle_pub = self.create_publisher(PointCloud2, '/stage4d/far_obstacles_viz', self.viz_qos)
        self.free_z = None
        self.local_obstacle_height = None
        self.latest = {}
        self.create_subscription(PointCloud2, '/terrain_map', lambda msg: self.cloud('/terrain_map', msg), self.sensor_qos)
        self.create_subscription(PointCloud2, '/terrain_map_ext', lambda msg: self.cloud('/terrain_map_ext', msg), self.sensor_qos)
        self.create_subscription(DiagnosticArray, '/far/planner_status', self.far_status, retained)
        self.create_subscription(DiagnosticArray, '/local_planner/status', self.local_status, retained)
        self.create_timer(1.0, self.publish)

    def far_status(self, message):
        for status in message.status:
            if status.name == 'far_planner':
                values = {item.key: item.value for item in status.values}
                try: self.free_z = float(values['far_terrain_free_z'])
                except (KeyError, ValueError): pass

    def local_status(self, message):
        for status in message.status:
            if status.name == 'local_planner':
                values = {item.key: item.value for item in status.values}
                try: self.local_obstacle_height = float(values['obstacle_height_threshold'])
                except (KeyError, ValueError): pass

    @staticmethod
    def make_cloud(header, points):
        # x/y/z-only PointCloud2 preserves the original frame/stamp and is for RViz only.
        return point_cloud2.create_cloud_xyz32(header, points)

    def cloud(self, topic, message):
        names = {field.name for field in message.fields}
        fields = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
        rows = []
        for row in point_cloud2.read_points(message, field_names=fields, skip_nans=True):
            # Humble may yield a zero-dimensional structured numpy record; use
            # field names rather than sequence slicing so both APIs work.
            try:
                xyz = (float(row['x']), float(row['y']), float(row['z']))
                raw_intensity = float(row['intensity']) if 'intensity' in fields else None
            except (IndexError, KeyError, TypeError):
                values = tuple(row)
                xyz = tuple(float(value) for value in values[:3])
                raw_intensity = float(values[3]) if len(values) == 4 else None
            if not all(math.isfinite(value) for value in xyz): continue
            intensity = raw_intensity if raw_intensity is not None and math.isfinite(raw_intensity) else None
            rows.append((xyz, intensity))
        self.latest[topic] = {'frame_id': message.header.frame_id or 'N/A',
                              'stamp': f'{message.header.stamp.sec}.{message.header.stamp.nanosec:09d}',
                              'points': rows}
        if topic == '/terrain_map' and self.local_obstacle_height is not None:
            obstacles = [xyz for xyz, intensity in rows if intensity is not None and intensity > self.local_obstacle_height]
            self.local_obstacle_pub.publish(self.make_cloud(message.header, obstacles))
        if topic == '/terrain_map_ext' and self.free_z is not None:
            free = [xyz for xyz, intensity in rows if intensity is not None and intensity < self.free_z]
            obstacles = [xyz for xyz, intensity in rows if intensity is not None and intensity >= self.free_z]
            self.far_free_pub.publish(self.make_cloud(message.header, free))
            self.far_obstacle_pub.publish(self.make_cloud(message.header, obstacles))

    @staticmethod
    def percentile(values, p):
        if not values: return 'N/A'
        values = sorted(values)
        return f'{values[min(len(values) - 1, round((len(values) - 1) * p))]:.6f}'

    def status(self, topic, data):
        status = DiagnosticStatus()
        status.name = f'stage4d_terrain{topic}'; status.hardware_id = 'stage4d'
        status.level = DiagnosticStatus.OK if data else DiagnosticStatus.WARN
        status.message = 'terrain statistics and visualization split available' if data else 'waiting for terrain topic'
        values = []
        def add(key, value): values.append(KeyValue(key=key, value=str(value)))
        if not data:
            for key in ('frame_id', 'stamp', 'point_count', 'intensity_lt_far_terrain_free_z',
                        'intensity_ge_far_terrain_free_z', 'localplanner_obstacle_height_threshold'):
                add(key, 'N/A')
            status.values = values; return status
        points = data['points']; xyzs = [point[0] for point in points]; ints = [point[1] for point in points if point[1] is not None]
        add('frame_id', data['frame_id']); add('stamp', data['stamp']); add('point_count', len(points))
        add('far_terrain_free_z', f'{self.free_z:.6f}' if self.free_z is not None else 'N/A')
        add('localplanner_obstacle_height_threshold', f'{self.local_obstacle_height:.6f}' if self.local_obstacle_height is not None else 'N/A')
        for index, label in enumerate(('x', 'y', 'z')):
            series = [xyz[index] for xyz in xyzs]
            add(label + '_min', f'{min(series):.6f}' if series else 'N/A'); add(label + '_max', f'{max(series):.6f}' if series else 'N/A')
        add('intensity_min', f'{min(ints):.6f}' if ints else 'N/A'); add('intensity_mean', f'{sum(ints)/len(ints):.6f}' if ints else 'N/A')
        add('intensity_p50', self.percentile(ints, .5)); add('intensity_p95', self.percentile(ints, .95)); add('intensity_max', f'{max(ints):.6f}' if ints else 'N/A')
        if ints and self.free_z is not None:
            free = sum(value < self.free_z for value in ints)
            add('intensity_lt_far_terrain_free_z', free); add('intensity_ge_far_terrain_free_z', len(ints) - free)
        else:
            add('intensity_lt_far_terrain_free_z', 'N/A'); add('intensity_ge_far_terrain_free_z', 'N/A')
        if ints and self.local_obstacle_height is not None:
            add('localplanner_obstacle_point_count', sum(value > self.local_obstacle_height for value in ints))
        else: add('localplanner_obstacle_point_count', 'N/A')
        status.values = values; return status

    def publish(self):
        message = DiagnosticArray(); message.header.stamp = self.get_clock().now().to_msg()
        message.status = [self.status('/terrain_map', self.latest.get('/terrain_map')),
                          self.status('/terrain_map_ext', self.latest.get('/terrain_map_ext'))]
        self.publisher.publish(message)

def main():
    rclpy.init(); node = TerrainDiagnostics()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
