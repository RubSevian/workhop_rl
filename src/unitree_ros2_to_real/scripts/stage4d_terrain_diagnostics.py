#!/usr/bin/env python3
"""Retained statistics for the terrain streams consumed by FAR in Stage4D."""
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
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        retained_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(DiagnosticArray, '/stage4d/terrain_diagnostics', retained_qos)
        self.free_z = None
        self.latest = {}
        self.create_subscription(PointCloud2, '/terrain_map', lambda msg: self.cloud('/terrain_map', msg), sensor_qos)
        self.create_subscription(PointCloud2, '/terrain_map_ext', lambda msg: self.cloud('/terrain_map_ext', msg), sensor_qos)
        self.create_subscription(DiagnosticArray, '/far/planner_status', self.far_status, retained_qos)
        self.create_timer(1.0, self.publish)

    def far_status(self, msg):
        for status in msg.status:
            if status.name != 'far_planner':
                continue
            values = {value.key: value.value for value in status.values}
            try:
                self.free_z = float(values['far_terrain_free_z'])
            except (KeyError, ValueError):
                pass

    def cloud(self, topic, msg):
        # FAR accepts x/y/z/intensity points.  If intensity is absent, retain
        # that fact instead of fabricating a free/obstacle classification.
        names = {field.name for field in msg.fields}
        rows = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                        skip_nans=True) if 'intensity' in names else \
               point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xs, ys, zs, intensities = [], [], [], []
        for row in rows:
            x, y, z = (float(row[0]), float(row[1]), float(row[2]))
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            xs.append(x); ys.append(y); zs.append(z)
            if len(row) == 4 and math.isfinite(float(row[3])):
                intensities.append(float(row[3]))
        self.latest[topic] = {
            'frame_id': msg.header.frame_id or 'N/A',
            'stamp': f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}',
            'xs': xs, 'ys': ys, 'zs': zs, 'intensities': intensities,
        }

    @staticmethod
    def percentile(values, p):
        if not values:
            return 'N/A'
        values = sorted(values)
        return f'{values[min(len(values) - 1, round((len(values) - 1) * p))]:.6f}'

    def status(self, topic, data):
        st = DiagnosticStatus()
        st.name = f'stage4d_terrain{topic}'
        st.hardware_id = 'stage4d'
        st.level = DiagnosticStatus.OK if data else DiagnosticStatus.WARN
        st.message = 'terrain statistics available' if data else 'waiting for terrain topic'
        values = []
        def add(key, value): values.append(KeyValue(key=key, value=str(value)))
        if not data:
            for key in ('frame_id', 'stamp', 'point_count', 'x_min', 'x_max', 'y_min', 'y_max',
                        'z_min', 'z_max', 'intensity_min', 'intensity_mean', 'intensity_p50',
                        'intensity_p95', 'intensity_max', 'intensity_lt_far_terrain_free_z',
                        'intensity_ge_far_terrain_free_z', 'free_ratio', 'obstacle_ratio'):
                add(key, 'N/A')
            st.values = values
            return st
        xs, ys, zs, ints = data['xs'], data['ys'], data['zs'], data['intensities']
        add('frame_id', data['frame_id']); add('stamp', data['stamp']); add('point_count', len(xs))
        for prefix, valueset in (('x', xs), ('y', ys), ('z', zs)):
            add(prefix + '_min', f'{min(valueset):.6f}' if valueset else 'N/A')
            add(prefix + '_max', f'{max(valueset):.6f}' if valueset else 'N/A')
        add('far_terrain_free_z', f'{self.free_z:.6f}' if self.free_z is not None else 'N/A')
        add('intensity_min', f'{min(ints):.6f}' if ints else 'N/A')
        add('intensity_mean', f'{sum(ints)/len(ints):.6f}' if ints else 'N/A')
        add('intensity_p50', self.percentile(ints, 0.50)); add('intensity_p95', self.percentile(ints, 0.95))
        add('intensity_max', f'{max(ints):.6f}' if ints else 'N/A')
        if ints and self.free_z is not None:
            free = sum(value < self.free_z for value in ints)
            obs = len(ints) - free
            add('intensity_lt_far_terrain_free_z', free); add('intensity_ge_far_terrain_free_z', obs)
            add('free_ratio', f'{free / len(ints):.6f}'); add('obstacle_ratio', f'{obs / len(ints):.6f}')
        else:
            add('intensity_lt_far_terrain_free_z', 'N/A'); add('intensity_ge_far_terrain_free_z', 'N/A')
            add('free_ratio', 'N/A'); add('obstacle_ratio', 'N/A')
        st.values = values
        return st

    def publish(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [self.status('/terrain_map', self.latest.get('/terrain_map')),
                      self.status('/terrain_map_ext', self.latest.get('/terrain_map_ext'))]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TerrainDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
