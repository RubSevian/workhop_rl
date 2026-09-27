#!/usr/bin/env python3
"""Stage4D Phase-1 evaluation: Point-LIO IMU pose vs time-aligned MuJoCo GT IMU.

This node is diagnostics only. It never publishes commands, TF, poses, or
planner input. Run it separately for every baseline/candidate trajectory.
"""
import argparse
import json
import math
from collections import deque
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# Parsed/verified from go2_rars01.xml; this script intentionally evaluates the
# Point-LIO IMU state, not the MuJoCo base origin.
IMU_IN_BASE = (-0.02557, 0.0, 0.04232)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.e-9


def qnorm(q):
    n = math.sqrt(sum(x*x for x in q))
    return (0., 0., 0., 1.) if n == 0. else tuple(x/n for x in q)


def rotate(q, v):
    x, y, z, w = qnorm(q); vx, vy, vz = v
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def inverse_rotate(q, v):
    x, y, z, w = qnorm(q)
    return rotate((-x, -y, -z, w), v)


def rpy(q):
    x, y, z, w = qnorm(q)
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def wrapped_delta(a, b): return (a-b+math.pi) % (2*math.pi)-math.pi
def mag(v): return math.sqrt(sum(x*x for x in v))
def pctl(values, q):
    if not values: return None
    v = sorted(values); i = (len(v)-1)*q; lo, hi = int(i), math.ceil(i)
    return v[lo] if lo == hi else v[lo] + (v[hi]-v[lo])*(i-lo)
def avg(values): return sum(values)/len(values) if values else None


class Diagnostic(Node):
    def __init__(self, output, max_delta, duration):
        super().__init__('stage4d_localization_extrinsic_diagnostics')
        self.output, self.max_delta = Path(output), max_delta
        self.gt = deque(maxlen=4000)
        self.samples, self.first_lio, self.first_gt = [], None, None
        self.goal_original = self.goal_current = None
        self.goal_from_topic = None
        self.last_base = self.last_imu = self.last_lio = None
        self.pub = self.create_publisher(String, '/stage4d/localization_extrinsic_metrics', 10)
        self.create_subscription(Odometry, '/sim/ground_truth_odom', self.gt_cb, 100)
        self.create_subscription(Odometry, '/state_estimation', self.lio_cb, 100)
        self.create_subscription(PointStamped, '/goal_point', self.goal_cb, 10)
        self.create_subscription(DiagnosticArray, '/far/planner_status', self.far_cb, 10)
        self.create_timer(1., self.heartbeat)
        if duration > 0: self.create_timer(duration, self.stop_once)
        self.done = False

    def gt_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.gt.append((stamp_seconds(msg.header.stamp), (p.x,p.y,p.z), (q.x,q.y,q.z,q.w)))

    def goal_cb(self, msg):
        if msg.header.frame_id: self.goal_from_topic = (msg.point.x,msg.point.y,msg.point.z)

    def far_cb(self, msg):
        for status in msg.status:
            if status.name != 'far_planner': continue
            values = {item.key: item.value for item in status.values}
            def point(prefix):
                try: return tuple(float(values[f'{prefix}_{axis}']) for axis in 'xyz')
                except (KeyError, ValueError): return None
            self.goal_original, self.goal_current = point('goal_original'), point('goal_current')

    def lio_cb(self, msg):
        if not self.gt: return
        t = stamp_seconds(msg.header.stamp)
        gt_t, base, gt_q = min(self.gt, key=lambda record: abs(record[0]-t))
        delta_t = abs(t-gt_t)
        if delta_t > self.max_delta: return
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        lio, lio_q = (position.x,position.y,position.z), (orientation.x,orientation.y,orientation.z,orientation.w)
        gt_imu = tuple(base[i]+rotate(gt_q, IMU_IN_BASE)[i] for i in range(3))
        error = tuple(lio[i]-gt_imu[i] for i in range(3)); body_error = inverse_rotate(gt_q, error)
        if self.first_lio is None: self.first_lio, self.first_gt = lio, gt_imu
        rel = mag(tuple((lio[i]-self.first_lio[i])-(gt_imu[i]-self.first_gt[i]) for i in range(3)))
        lr, gr = rpy(lio_q), rpy(gt_q)
        self.samples.append({'dt':delta_t, 'xy':math.hypot(error[0],error[1]), 'xyz':mag(error),
            'body':body_error, 'relative':rel, 'yaw':math.degrees(wrapped_delta(lr[2],gr[2])),
            'roll':math.degrees(wrapped_delta(lr[0],gr[0])), 'pitch':math.degrees(wrapped_delta(lr[1],gr[1]))})
        self.last_base, self.last_imu, self.last_lio = base, gt_imu, lio

    @staticmethod
    def distance(a, b): return mag(tuple(a[i]-b[i] for i in range(3))) if a and b else None

    def result(self):
        if not self.samples: return {'samples': 0, 'max_timestamp_delta_s': 0.0}
        get = lambda key: [sample[key] for sample in self.samples]
        body = get('body'); original = self.goal_original or self.goal_from_topic
        result = {
          'samples':len(self.samples), 'mean_xy_error_m':avg(get('xy')), 'median_xy_error_m':pctl(get('xy'),.5),
          'p95_xy_error_m':pctl(get('xy'),.95), 'max_xy_error_m':max(get('xy')), 'final_xy_error_m':get('xy')[-1],
          'mean_xyz_error_m':avg(get('xyz')), 'p95_xyz_error_m':pctl(get('xyz'),.95), 'max_xyz_error_m':max(get('xyz')),
          'mean_body_error_x_m':avg([x[0] for x in body]), 'mean_body_error_y_m':avg([x[1] for x in body]),
          'mean_body_error_z_m':avg([x[2] for x in body]), 'mean_yaw_error_deg':avg(get('yaw')),
          'mean_roll_error_deg':avg(get('roll')), 'mean_pitch_error_deg':avg(get('pitch')),
          'final_relative_motion_error_m':get('relative')[-1], 'p95_timestamp_delta_s':pctl(get('dt'),.95),
          'max_timestamp_delta_s':max(get('dt')), 'final_gt_base_position':self.last_base,
          'final_gt_imu_position':self.last_imu, 'final_lio_position':self.last_lio,
          'goal_original_map_xyz':original, 'goal_current_map_xyz':self.goal_current,
          'goal_adjustment_distance_m':self.distance(original,self.goal_current),
          'final_lio_to_original_goal_m':self.distance(self.last_lio,original),
          'final_lio_to_current_goal_m':self.distance(self.last_lio,self.goal_current),
          'final_gt_base_to_original_goal_m':self.distance(self.last_base,original),
          'final_gt_base_to_current_goal_m':self.distance(self.last_base,self.goal_current),
          'final_gt_imu_to_original_goal_m':self.distance(self.last_imu,original),
          'final_gt_imu_to_current_goal_m':self.distance(self.last_imu,self.goal_current),
        }
        return result

    def heartbeat(self):
        payload = json.dumps(self.result(), sort_keys=True)
        self.pub.publish(String(data=payload))
        result = self.result()
        self.get_logger().info(
            f"Phase1 samples={result['samples']} mean_xy={result.get('mean_xy_error_m')} "
            f"p95_dt={result.get('p95_timestamp_delta_s')}")

    def finish(self):
        if self.done: return
        self.done = True; self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(self.result(), indent=2, sort_keys=True)+'\n')
        self.get_logger().info(f'Wrote {self.output}')

    def stop_once(self):
        self.finish(); rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=0., help='0 means run until Ctrl-C')
    parser.add_argument('--max-time-delta', type=float, default=.02)
    args, _ = parser.parse_known_args()
    rclpy.init(); node = Diagnostic(args.output, args.max_time_delta, args.duration)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.finish(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
