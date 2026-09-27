#!/usr/bin/env python3
"""Print the retained Stage4D navigation diagnostic as readable terminal text."""
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class Status(Node):
    def __init__(self):
        super().__init__('stage4d_navigation_status')
        self.done = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/stage4d/navigation_explanation', self.callback, qos)
        self.create_timer(5.0, self.timeout)

    @staticmethod
    def fmt(value):
        return 'n/a' if value is None else f'{float(value):.3f} m'

    def callback(self, message):
        try:
            state = json.loads(message.data)
        except ValueError:
            print('Stage4D diagnostic is not valid JSON', file=sys.stderr)
            self.done = True
            return
        remaining = state.get('remaining_to_goal_m', {})
        far = state.get('far', {})
        local = state.get('local_planner', {})
        command = state.get('command', {})
        ready = state.get('readiness', {})
        goal = state.get('goal_map_xy_m')
        print('Stage4D navigation status')
        print(f'  state: {state.get("reason")}')
        print(f'  goal map XY: {goal}')
        print(f'  FAR: {far.get("reason")}; distance={self.fmt(far.get("distance_to_goal_m"))}; '
              f'path_size={far.get("path_size")}; failed={far.get("planning_failed")}')
        print(f'  localPlanner: {local.get("reason")}; relative_goal={self.fmt(local.get("relative_goal_distance_m"))}; '
              f'blocked={local.get("candidate_blocked")}/{local.get("candidate_total")}; '
              f'scored={local.get("candidate_scored")}')
        print(f'  remaining: planner={self.fmt(remaining.get("pointlio_planner"))}; '
              f'MuJoCo={self.fmt(remaining.get("mujoco_actual"))}; '
              f'disagreement={self.fmt(remaining.get("disagreement"))}')
        print(f'  Point-LIO vs MuJoCo XY error: {self.fmt(state.get("pointlio_vs_ground_truth_xy_error_m"))}')
        print(f'  readiness: pre={ready.get("pre_goal_ready")}; post={ready.get("post_goal_ready")}; '
              f'blockers={ready.get("blockers")}')
        print(f'  command: cmd_vel={command.get("cmd_vel")}; safe={command.get("rl_safe_command")}')
        self.done = True

    def timeout(self):
        print('No retained Stage4D diagnostic: start Stage4D first.', file=sys.stderr)
        self.done = True


def main():
    rclpy.init()
    node = Status()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
