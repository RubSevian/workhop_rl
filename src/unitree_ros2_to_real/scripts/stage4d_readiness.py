#!/usr/bin/env python3
"""Stage4D gate: validate original-stack data and topic ownership before a goal."""
import json
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PointStamped, TwistStamped

class Readiness(Node):
    TOPICS = {
        '/utlidar/cloud': PointCloud2, '/utlidar/transformed_cloud': PointCloud2,
        '/utlidar/transformed_raw_imu': Imu, '/state_estimation': Odometry,
        '/registered_scan': PointCloud2, '/terrain_map': PointCloud2,
        '/terrain_map_ext': PointCloud2, '/runtime': None,
    }
    OWNERS = {
        '/utlidar/cloud': PointCloud2, '/utlidar/imu': Imu, '/state_estimation': Odometry,
        '/registered_scan': PointCloud2, '/terrain_map': PointCloud2, '/terrain_map_ext': PointCloud2,
        '/navigation_active': Bool, '/way_point': PointStamped, '/path': Path, '/cmd_vel': TwistStamped,
    }
    def __init__(self):
        super().__init__('stage4d_readiness')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pre = self.create_publisher(Bool, '/stage4d/navigation_ready', qos)
        self.post = self.create_publisher(Bool, '/stage4d/post_goal_ready', qos)
        self.detail = self.create_publisher(String, '/stage4d/readiness_diagnostics', qos)
        self.seen = {}; self.nonempty = {}; self.pointlio_ok = None; self.rl_ready = False; self.clock_value = None; self.clock_advanced = False
        self.nav_active = False; self.waypoint = False; self.path_len = 0; self.nonzero_command = False
        self.create_subscription(Clock, '/clock', self.clock, 10)
        self.create_subscription(String, '/mujoco/lidar_diagnostics', self.lidar_diag, 10)
        self.create_subscription(Float32, '/runtime', lambda msg: self.data('/runtime', msg), 10)
        for topic, typ in self.TOPICS.items():
            if typ: self.create_subscription(typ, topic, lambda msg, t=topic: self.data(t, msg), 10)
        self.create_subscription(Bool, '/pointlio_ready', self.pointlio, 10)
        self.create_subscription(Bool, '/stage4d/rl_ready', lambda msg: setattr(self, 'rl_ready', bool(msg.data)), qos)
        self.create_subscription(Bool, '/navigation_active', self.navigation, 10)
        self.create_subscription(PointStamped, '/way_point', self.waypoint_cb, 10)
        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)
        self.timer = self.create_timer(0.25, self.tick)
    def clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_value is not None and value < self.clock_value - 0.5:
            # Freshness of the previous simulation epoch is not readiness now.
            self.seen.clear(); self.nonempty.clear(); self.pointlio_ok = None
            self.clock_advanced = False; self.nav_active = False
            self.waypoint = False; self.path_len = 0; self.nonzero_command = False
            self.pre.publish(Bool(data=False)); self.post.publish(Bool(data=False))
        if self.clock_value is not None and value > self.clock_value: self.clock_advanced = True
        self.clock_value = value; self.seen['/clock'] = time.monotonic()
    def lidar_diag(self, msg):
        try:
            d = json.loads(msg.data); self.nonempty['/utlidar/cloud'] = d.get('valid_hits', 0) > 0
            self.seen['/utlidar/cloud'] = time.monotonic()
        except (TypeError, ValueError): self.nonempty['/utlidar/cloud'] = False
    def data(self, topic, msg):
        self.seen[topic] = time.monotonic()
        if hasattr(msg, 'width'): self.nonempty[topic] = msg.width * msg.height > 0
    def pointlio(self, msg): self.pointlio_ok = bool(msg.data)
    def navigation(self, msg): self.nav_active = bool(msg.data)
    def waypoint_cb(self, _): self.waypoint = True
    def path_cb(self, msg): self.path_len = len(msg.poses)
    def cmd_cb(self, msg):
        self.nonzero_command = self.nonzero_command or any(abs(v) > 1e-4 for v in (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z))
        self.seen['/cmd_vel'] = time.monotonic()
    def fresh(self, topic): return time.monotonic() - self.seen.get(topic, 0.0) < 1.5
    def tick(self):
        data_topics = tuple(self.TOPICS)
        raw_imu = self.fresh('/utlidar/transformed_raw_imu')
        values_ok = all(self.fresh(t) and self.nonempty.get(t, True) for t in data_topics if t != '/runtime') and raw_imu
        values_ok = values_ok and self.rl_ready
        owner_count = {topic: len(self.get_publishers_info_by_topic(topic)) for topic in self.OWNERS}
        owners_ok = all(n == 1 for n in owner_count.values())
        pre_ok = bool(self.clock_advanced and values_ok and owners_ok and self.fresh('/runtime'))
        post_ok = bool(pre_ok and self.nav_active and self.waypoint and self.path_len > 1 and self.fresh('/cmd_vel'))
        self.pre.publish(Bool(data=pre_ok)); self.post.publish(Bool(data=post_ok))
        missing = [topic for topic in data_topics if topic != '/runtime' and not self.fresh(topic)]
        if not raw_imu: missing.append('/utlidar/transformed_raw_imu')
        owner_conflicts = [topic for topic, count in owner_count.items() if count != 1]
        blockers = []
        if not self.clock_advanced: blockers.append('CLOCK_NOT_ADVANCING')
        if not self.fresh('/runtime'): blockers.append('WAITING_FOR_RUNTIME')
        if missing: blockers.append('WAITING_FOR:' + ','.join(missing))
        if not self.fresh('/state_estimation'): blockers.append('POINTLIO_NOT_READY')
        if not self.rl_ready: blockers.append('RL_STANDUP_NOT_COMPLETE')
        if owner_conflicts: blockers.append('TOPIC_OWNERS:' + ','.join(owner_conflicts))
        if pre_ok and not self.nav_active: blockers.append('WAITING_FOR_GOAL_OR_FAR_NAVIGATION')
        elif self.nav_active and self.path_len <= 1: blockers.append('WAITING_FOR_LOCAL_PATH')
        elif self.nav_active and not self.fresh('/cmd_vel'): blockers.append('WAITING_FOR_CMD_VEL')
        self.detail.publish(String(data=json.dumps({'pre_goal_ready': pre_ok, 'post_goal_ready': post_ok,
            'clock_advancing': self.clock_advanced, 'owners': owner_count, 'path_poses': self.path_len,
            'navigation_active': self.nav_active, 'nonzero_cmd_seen': self.nonzero_command,
            'missing_or_stale': missing, 'owner_conflicts': owner_conflicts, 'blockers': blockers}, sort_keys=True)))
def main():
    rclpy.init(); n = Readiness()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: n.destroy_node(); try_shutdown()
if __name__ == '__main__': main()
