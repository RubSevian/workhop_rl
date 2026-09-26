#!/usr/bin/env python3
"""Deterministic Stage-4B ground-truth route publisher and evaluator.

The existing local_planner/pathFollower remains the command owner.  This node
only supplies a refreshed vehicle-frame path and records the resulting
ground-truth goal/path metrics, so planner-to-RL can be tested without a SLAM
source or a synthetic pose fed into navigation.
"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


def wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def segment_distance(x, y, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 1.0e-12:
        return math.hypot(x - ax, y - ay)
    u = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    return math.hypot(x - (ax + u * dx), y - (ay + u * dy))


class Stage4BNavigationTest(Node):
    def __init__(self, goal_x: float, goal_y: float, goal_yaw: float,
                 duration: float, report_path: str, stop_distance: float,
                 start_delay: float):
        super().__init__("stage4b_navigation_test")
        self.goal_x_local = goal_x
        self.goal_y_local = goal_y
        self.goal_yaw_local = goal_yaw
        self.duration = duration
        self.stop_distance = stop_distance
        self.start_delay = start_delay
        self.report_path = Path(report_path)
        self.odom = None
        self.start = None
        self.goal = None
        self.goal_yaw = None
        self.samples = []
        self.commands = []
        self.latest_command = None
        self.collision = {}
        self.arrival_time = None
        self.reached_goal = False
        self.finished = False
        self.navigation_started = None
        self.active = False

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.active_pub = self.create_publisher(Bool, "/navigation_active", qos)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.path_pub = self.create_publisher(NavPath, "/path", 10)
        self.odom_sub = self.create_subscription(Odometry, "/state_estimation", self.odom_cb, 10)
        self.cmd_sub = self.create_subscription(TwistStamped, "/cmd_vel", self.cmd_cb, 10)
        collision_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.collision_sub = self.create_subscription(
            String, "/mujoco/collision_diagnostics", self.collision_cb, collision_qos)
        self.timer = self.create_timer(0.1, self.tick)
        self.started_at = time.monotonic()

    def odom_cb(self, msg: Odometry):
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                  msg.pose.pose.position.z, msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            return
        self.odom = msg
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.start is None:
            self.start = (x, y, yaw)
            self.goal = (x + math.cos(yaw) * self.goal_x_local - math.sin(yaw) * self.goal_y_local,
                         y + math.sin(yaw) * self.goal_x_local + math.cos(yaw) * self.goal_y_local)
            self.goal_yaw = wrap_pi(yaw + self.goal_yaw_local)
            self.get_logger().info(
                f"Stage-4B goal: map=({self.goal[0]:.3f}, {self.goal[1]:.3f}), "
                f"yaw={self.goal_yaw:.3f}; navigation starts after {self.start_delay:.1f} s")

        if self.goal is not None:
            goal_error = math.hypot(x - self.goal[0], y - self.goal[1])
            cross_track = segment_distance(x, y, self.start[0], self.start[1],
                                           self.goal[0], self.goal[1])
            self.samples.append({
                "t": time.monotonic() - self.started_at,
                "goal_error_gt_m": goal_error,
                "goal_error_est_m": goal_error,
                "cross_track_m": cross_track,
                "x": x,
                "y": y,
                "yaw": yaw,
                "gt_vx_body": float(msg.twist.twist.linear.x),
                "gt_vy_body": float(msg.twist.twist.linear.y),
                "gt_wz": float(msg.twist.twist.angular.z),
                "cmd_vx": self.latest_command[0] if self.latest_command else None,
                "cmd_vy": self.latest_command[1] if self.latest_command else None,
                "cmd_wz": self.latest_command[2] if self.latest_command else None,
                "goal_vehicle_x": (math.cos(yaw) * (self.goal[0] - x) +
                                   math.sin(yaw) * (self.goal[1] - y)),
                "goal_vehicle_y": (-math.sin(yaw) * (self.goal[0] - x) +
                                   math.cos(yaw) * (self.goal[1] - y)),
            })
            if self.active and self.arrival_time is None and goal_error <= self.stop_distance:
                self.arrival_time = time.monotonic()
                self.reached_goal = True
                self.active = False
                self.active_pub.publish(Bool(data=False))
                self.get_logger().info(
                    f"Stage-4B arrival: goal_error_gt={goal_error:.3f} m; observing stop for 3 s")

    def cmd_cb(self, msg: TwistStamped):
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if all(math.isfinite(float(value)) for value in values):
            self.latest_command = values
            self.commands.append({"t": time.monotonic() - self.started_at,
                                  "vx": values[0], "vy": values[1], "wz": values[2]})

    def collision_cb(self, msg: String):
        try:
            self.collision = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning("Ignoring malformed MuJoCo collision diagnostics")

    def publish_route(self):
        if self.start is None or self.goal is None or not self.active:
            return
        now = self.get_clock().now().to_msg()
        goal = PointStamped()
        goal.header.stamp, goal.header.frame_id = now, "map"
        goal.point.x, goal.point.y = self.goal
        self.goal_pub.publish(goal)

        route = NavPath()
        route.header.stamp, route.header.frame_id = now, "vehicle"
        # pathFollower captures the current vehicle pose when it receives each
        # refreshed path. Transform the one fixed world goal into that current
        # vehicle frame before publishing, so the physical target never moves.
        x, y = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        dx, dy = self.goal[0] - x, self.goal[1] - y
        goal_x_vehicle = math.cos(yaw) * dx + math.sin(yaw) * dy
        goal_y_vehicle = -math.sin(yaw) * dx + math.cos(yaw) * dy
        for index in range(21):
            u = index / 20.0
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = goal_x_vehicle * u
            pose.pose.position.y = goal_y_vehicle * u
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        self.path_pub.publish(route)

    def tick(self):
        if self.start is not None and not self.active and self.arrival_time is None and \
                self.navigation_started is None and time.monotonic() - self.started_at >= self.start_delay:
            self.navigation_started = time.monotonic()
            self.active = True
            self.active_pub.publish(Bool(data=True))
            self.get_logger().info("Stage-4B navigation_active=true")
        self.publish_route()
        if self.start is None:
            return
        now = time.monotonic()
        if self.navigation_started is not None and self.arrival_time is None and \
                now - self.navigation_started > self.duration:
            self.active = False
            self.active_pub.publish(Bool(data=False))
            self.get_logger().warning("Stage-4B duration expired before arrival")
            self.arrival_time = now
        if self.arrival_time is not None and now - self.arrival_time >= 3.0:
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.active_pub.publish(Bool(data=False))
        errors = [sample["goal_error_gt_m"] for sample in self.samples]
        cross = [sample["cross_track_m"] for sample in self.samples]
        tracked = [sample for sample in self.samples if sample["cmd_vx"] is not None]
        def tracking_metric(command_key, gt_key, absolute=False):
            values = [sample[command_key] - sample[gt_key] for sample in tracked]
            if not values:
                return None
            if absolute:
                return sum(abs(value) for value in values) / len(values)
            return math.sqrt(sum(value * value for value in values) / len(values))
        post = [sample for sample in self.samples if self.arrival_time and
                self.started_at + sample["t"] >= self.arrival_time]
        final = self.samples[-1] if self.samples else {}
        metrics = {
            "goal_local_m": [self.goal_x_local, self.goal_y_local],
            "stopDisThre_m": self.stop_distance,
            "initial_goal_error_m": errors[0] if errors else None,
            "final_goal_error_gt_m": final.get("goal_error_gt_m"),
            "final_goal_error_est_m": final.get("goal_error_est_m"),
            "minimum_goal_error_gt_m": min(errors) if errors else None,
            "goal_error_gt_p50_m": sorted(errors)[len(errors) // 2] if errors else None,
            "goal_error_gt_p95_m": sorted(errors)[max(0, int(len(errors) * 0.95) - 1)] if errors else None,
            "cross_track_mean_m": sum(cross) / len(cross) if cross else None,
            "cross_track_rmse_m": math.sqrt(sum(value * value for value in cross) / len(cross)) if cross else None,
            "cross_track_p95_m": sorted(cross)[max(0, int(len(cross) * 0.95) - 1)] if cross else None,
            "completion_time_s": (self.arrival_time - self.navigation_started) if self.arrival_time and self.navigation_started else None,
            "arrived": self.reached_goal,
            "post_arrival_drift_xy_m": (max((math.hypot(s["x"] - final["x"], s["y"] - final["y"]) for s in post), default=0.0)),
            "post_arrival_yaw_drift_rad": (max((abs(wrap_pi(s["yaw"] - final["yaw"])) for s in post), default=0.0)),
            "cmd_max_abs_vx": max((abs(c["vx"]) for c in self.commands), default=0.0),
            "cmd_max_abs_vy": max((abs(c["vy"]) for c in self.commands), default=0.0),
            "cmd_max_abs_wz": max((abs(c["wz"]) for c in self.commands), default=0.0),
            "samples": len(self.samples),
            "commands": len(self.commands),
            "fixed_goal_world_m": list(self.goal) if self.goal else None,
            "fixed_start_world_m": list(self.start[:2]) if self.start else None,
            "non_floor_environment_contact_count": self.collision.get("non_floor_environment_contact_count", 0),
            "non_floor_environment_contact_steps": self.collision.get("non_floor_environment_contact_steps", 0),
            "unexpected_robot_floor_contact_count": self.collision.get("unexpected_robot_floor_contact_count", 0),
            "first_non_floor_contact_time_s": self.collision.get("first_non_floor_contact_time_s"),
            "first_non_floor_contact_geoms": self.collision.get("first_non_floor_contact_geoms"),
            "first_unexpected_floor_contact_geoms": self.collision.get("first_unexpected_floor_contact_geoms"),
            "vx_tracking_rmse": tracking_metric("cmd_vx", "gt_vx_body"),
            "vy_tracking_rmse": tracking_metric("cmd_vy", "gt_vy_body"),
            "wz_tracking_rmse": tracking_metric("cmd_wz", "gt_wz"),
            "vx_tracking_mae": tracking_metric("cmd_vx", "gt_vx_body", True),
            "vy_tracking_mae": tracking_metric("cmd_vy", "gt_vy_body", True),
            "wz_tracking_mae": tracking_metric("cmd_wz", "gt_wz", True),
        }
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        self.report_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"Stage-4B metrics: {json.dumps(metrics, sort_keys=True)}")
        self.timer.cancel()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=1.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--start-delay", type=float, default=12.0)
    parser.add_argument("--stop-distance", type=float, default=0.2)
    parser.add_argument("--report", default="/tmp/stage4b_navigation_metrics.json")
    # ROS 2 appends `--ros-args` to every launch-managed executable.  This
    # evaluator has no ROS parameters, so retain only the task arguments.
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Stage4BNavigationTest(args.goal_x, args.goal_y, args.goal_yaw,
                                 args.duration, args.report, args.stop_distance,
                                 args.start_delay)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
