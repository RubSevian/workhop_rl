#!/usr/bin/env python3
"""Human-readable, observation-only explanation of Stage4D motion state."""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


def number(values, key):
    try:
        return float(values[key])
    except (KeyError, TypeError, ValueError):
        return None


class NavigationExplainer(Node):
    def __init__(self):
        super().__init__('stage4d_navigation_explainer')
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.text_pub = self.create_publisher(String, '/stage4d/navigation_explanation', retained)
        self.readiness = {}
        self.far = {}
        self.local = {}
        self.nav_active = False
        self.cmd = (0., 0., 0.)
        self.safe = (0., 0., 0.)
        self.cmd_seen = 0.
        self.safe_seen = 0.
        self.estimated_xy = None
        self.ground_truth_xy = None
        self.goal_xy = None
        self.create_subscription(Odometry, '/state_estimation', self.estimated_odom, 10)
        # Evaluation only: never used to generate a command.
        self.create_subscription(Odometry, '/sim/ground_truth_odom', self.ground_truth_odom, 10)
        self.create_subscription(PointStamped, '/goal_point', self.goal, 10)
        self.create_subscription(String, '/stage4d/readiness_diagnostics', self.readiness_cb, retained)
        self.create_subscription(DiagnosticArray, '/far/planner_status',
                                 lambda msg: self.diag(msg, 'far'), retained)
        self.create_subscription(DiagnosticArray, '/local_planner/status',
                                 lambda msg: self.diag(msg, 'local'), retained)
        self.create_subscription(Bool, '/navigation_active',
                                 lambda msg: setattr(self, 'nav_active', bool(msg.data)), retained)
        self.create_subscription(TwistStamped, '/cmd_vel', lambda msg: self.command(msg, False), 10)
        self.create_subscription(TwistStamped, '/rl/safe_command', lambda msg: self.command(msg, True), 10)
        self.create_timer(.25, self.publish)

    def estimated_odom(self, message):
        self.estimated_xy = (message.pose.pose.position.x, message.pose.pose.position.y)

    def ground_truth_odom(self, message):
        self.ground_truth_xy = (message.pose.pose.position.x, message.pose.pose.position.y)

    def goal(self, message):
        self.goal_xy = (message.point.x, message.point.y)

    def readiness_cb(self, message):
        try:
            self.readiness = json.loads(message.data)
        except (TypeError, ValueError):
            self.readiness = {'blockers': ['READINESS_DIAGNOSTIC_PARSE_ERROR']}

    def diag(self, message, destination):
        wanted = 'far_planner' if destination == 'far' else 'local_planner'
        for status in message.status:
            if status.name == wanted:
                values = {value.key: value.value for value in status.values}
                values.update({'code': values.get('reason_code', 'UNKNOWN'), 'text': status.message})
                setattr(self, destination, values)

    def command(self, message, safe):
        value = (message.twist.linear.x, message.twist.linear.y, message.twist.angular.z)
        if safe:
            self.safe, self.safe_seen = value, time.monotonic()
        else:
            self.cmd, self.cmd_seen = value, time.monotonic()

    @staticmethod
    def nonzero(command):
        return any(abs(value) > 1e-4 for value in command)

    def final_goal(self):
        if self.goal_xy is not None:
            return self.goal_xy
        try:
            return (float(self.far['goal_original_x']), float(self.far['goal_original_y']))
        except (KeyError, TypeError, ValueError):
            return None

    def reason(self):
        blockers = self.readiness.get('blockers', [])
        if blockers:
            return ' | '.join(blockers)
        if not self.nav_active:
            return 'WAITING_FOR_GOAL_OR_FAR_NAVIGATION'
        if self.local.get('code') not in ('PATH_FOUND', 'PATH_PUBLISHED'):
            return 'LOCAL_PLANNER:' + self.local.get('code', 'WAITING_FOR_STATUS')
        if not self.nonzero(self.cmd):
            return 'LOCAL_PATH_EXISTS_BUT_CMD_VEL_ZERO'
        if not self.nonzero(self.safe):
            return 'RL_WATCHDOG_OR_NAVIGATION_GATE_ZEROED_COMMAND'
        return 'COMMAND_REACHES_RL: robot should move unless physics/contact prevents it'

    def publish(self):
        goal = self.final_goal()
        localization_error = (
            math.hypot(self.estimated_xy[0] - self.ground_truth_xy[0],
                       self.estimated_xy[1] - self.ground_truth_xy[1])
            if self.estimated_xy is not None and self.ground_truth_xy is not None else None)
        estimated_remaining = (
            math.hypot(self.estimated_xy[0] - goal[0], self.estimated_xy[1] - goal[1])
            if self.estimated_xy is not None and goal is not None else None)
        actual_remaining = (
            math.hypot(self.ground_truth_xy[0] - goal[0], self.ground_truth_xy[1] - goal[1])
            if self.ground_truth_xy is not None and goal is not None else None)
        payload = {
            'reason': self.reason(),
            'navigation_active': self.nav_active,
            'goal_map_xy_m': list(goal) if goal is not None else None,
            'remaining_to_goal_m': {
                'pointlio_planner': estimated_remaining,
                'mujoco_actual': actual_remaining,
                'disagreement': (abs(estimated_remaining - actual_remaining)
                                 if estimated_remaining is not None and actual_remaining is not None else None),
            },
            # `/state_estimation` is an IMU pose while GT odom is a base pose;
            # this latest-message separation is deliberately not a localization
            # error.  Use stage4d_localization_extrinsic_diagnostics.py for the
            # time-aligned IMU-vs-IMU metric.
            'unsynchronized_pointlio_imu_to_gt_base_xy_separation_m': localization_error,
            'localization_audit_topic': '/stage4d/localization_extrinsic_metrics',
            'readiness': {
                'pre_goal_ready': self.readiness.get('pre_goal_ready'),
                'post_goal_ready': self.readiness.get('post_goal_ready'),
                'blockers': self.readiness.get('blockers', []),
            },
            'far': {
                'reason': self.far.get('code', 'WAITING_FOR_STATUS'),
                'distance_to_goal_m': number(self.far, 'distance_to_goal'),
                'goal_reached': self.far.get('goal_reached'),
                'planning_failed': self.far.get('planning_failed'),
                'path_size': self.far.get('path_size'),
                'goal_original_xyz': [number(self.far, f'goal_original_{axis}') for axis in 'xyz'],
                'goal_current_xyz': [number(self.far, f'goal_current_{axis}') for axis in 'xyz'],
                'goal_adjustment_distance_m': number(self.far, 'goal_adjustment_distance'),
            },
            'local_planner': {
                'reason': self.local.get('code', 'WAITING_FOR_STATUS'),
                'relative_goal_distance_m': number(self.local, 'relative_goal_distance'),
                'candidate_total': self.local.get('candidate_paths_total'),
                'candidate_blocked': self.local.get('candidate_paths_blocked'),
                'candidate_scored': self.local.get('candidate_paths_scored'),
            },
            'command': {
                'cmd_vel': dict(zip(('vx', 'vy', 'wz'), self.cmd)),
                'rl_safe_command': dict(zip(('vx', 'vy', 'wz'), self.safe)),
            },
        }
        self.text_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rclpy.init()
    node = NavigationExplainer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == '__main__':
    main()
