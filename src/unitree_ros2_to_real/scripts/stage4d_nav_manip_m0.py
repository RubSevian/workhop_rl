#!/usr/bin/env python3
"""Simulation-only Stage4D NAV to RARS01 manipulation M0.

Uses transport-free RARS01 kinematics and trajectory helpers. No real
RARS controller, SDK, serial port, or hardware transport is imported.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

ARM_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_NAMES = ("gripper_left_joint", "gripper_right_joint")
JOINT_NAMES = ARM_NAMES + GRIPPER_NAMES
HOME_Q = np.array([0.0, 1.0, 1.0, -0.5, 0.0, 0.0])
REST_Q = np.zeros(6)
PREGRASP_Q = np.array([0.25, 1.05, 0.85, -0.4, 0.1, -0.2])
SMOKE_TARGET_Q = np.array([0.35, 1.2, 0.95, -0.35, 0.15, -0.35])
GRIPPER_OPEN_M = 0.040
GRIPPER_CLOSED_M = 0.005
TARGET_RATE_HZ = 20.0


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def pose_summary(transform):
    return {
        "xyz_m": transform[:3, 3].astype(float).tolist(),
        "rotation_matrix": transform[:3, :3].astype(float).tolist(),
    }


class NavManipM0(Node):
    """Sequential base settle, measured arm homing, Cartesian motion and return."""

    def __init__(self, args, kinematics, trajectory, target_pose):
        super().__init__("stage4d_nav_manip_m0")
        self.args = args
        self.kinematics = kinematics
        self.trajectory = trajectory
        self.target_pose = target_pose
        self.goal_publisher = self.create_publisher(PointStamped, "/goal_point", 10)
        self.arm_publisher = self.create_publisher(JointState, "/rars01/arm_target", 10)
        self.status_publisher = self.create_publisher(String, "/stage4d/nav_manip_m0_status", 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._m0_subscriptions = [
            self.create_subscription(Bool, "/stage4d/navigation_ready", self.on_ready, retained),
            self.create_subscription(Bool, "/far_reach_goal_status", self.on_goal_reached, 10),
            self.create_subscription(Bool, "/navigation_active", self.on_navigation_active, retained),
            self.create_subscription(Odometry, "/state_estimation", self.on_odometry, 10),
            self.create_subscription(JointState, "/go2/motor_state", self.on_arm_state, 10),
        ]
        self.timer = self.create_timer(1.0 / TARGET_RATE_HZ, self.tick)
        self.started_at = time.monotonic()
        self.state_started_at = self.started_at
        self.stage_started_at = self.started_at
        self.state = "IDLE"
        self.stage = ""
        self.navigation_ready = False
        self.navigation_active = False
        self.goal_sent = False
        self.goal_reached = False
        self.odometry = None
        self.odometry_at = 0.0
        self.arm_measurements = {}
        self.arm_measurements_at = 0.0
        self.settle_count = 0
        self.anchor = None
        self.max_anchor_xy_m = 0.0
        self.max_anchor_yaw_rad = 0.0
        self.plan = {}
        self.samples = deque()
        self.last_arm_target = None
        self.last_gripper_target = GRIPPER_OPEN_M
        self.result = {
            "schema": "stage4d_nav_manip_m0/v2",
            "simulation_only": True,
            "real_serial_opened": False,
            "base_goal": {
                "x": args.base_x,
                "y": args.base_y,
                "yaw_deg": args.base_yaw_deg,
            },
            "arm_target_base_link": pose_summary(target_pose),
            "states": [],
            "ik": {},
            "sim_grasp": False,
            "abort_reason": None,
        }
        self.transition("NAV_TO_BASE_GOAL")

    def transition(self, new_state, **details):
        previous = self.state
        self.state = new_state
        self.state_started_at = time.monotonic()
        self.result["states"].append({
            "at_s": round(self.state_started_at - self.started_at, 3),
            "from": previous,
            "to": new_state,
            **details,
        })
        self.publish_status(new_state)

    def publish_status(self, event):
        message = String()
        message.data = json.dumps({
            "state": self.state,
            "event": event,
            "navigation_active": self.navigation_active,
            "simulation_only": True,
        })
        self.status_publisher.publish(message)

    def on_ready(self, message):
        self.navigation_ready = bool(message.data)

    def on_goal_reached(self, message):
        if self.goal_sent:
            self.goal_reached = bool(message.data)

    def on_navigation_active(self, message):
        self.navigation_active = bool(message.data)

    def on_odometry(self, message):
        self.odometry = message
        self.odometry_at = time.monotonic()
        if self.state == "BASE_SETTLE":
            velocity = message.twist.twist
            moving = math.hypot(velocity.linear.x, velocity.linear.y)
            if moving <= self.args.settle_linear_mps and abs(velocity.angular.z) <= self.args.settle_angular_rps:
                self.settle_count += 1
            else:
                self.settle_count = 0
        if self.anchor is not None:
            x, y, heading = self.base_pose()
            self.max_anchor_xy_m = max(
                self.max_anchor_xy_m,
                math.hypot(x - self.anchor["x"], y - self.anchor["y"]),
            )
            yaw_delta = math.atan2(
                math.sin(heading - math.radians(self.anchor["yaw_deg"])),
                math.cos(heading - math.radians(self.anchor["yaw_deg"])),
            )
            self.max_anchor_yaw_rad = max(self.max_anchor_yaw_rad, abs(yaw_delta))

    def on_arm_state(self, message):
        self.arm_measurements = dict(zip(message.name, message.position))
        self.arm_measurements_at = time.monotonic()

    def base_pose(self):
        if self.odometry is None:
            return None
        position = self.odometry.pose.pose.position
        return (
            float(position.x),
            float(position.y),
            yaw_from_quaternion(self.odometry.pose.pose.orientation),
        )

    def measured_arm(self):
        if time.monotonic() - self.arm_measurements_at > 0.25:
            return None
        if not all(name in self.arm_measurements for name in JOINT_NAMES):
            return None
        measured = np.array([self.arm_measurements[name] for name in ARM_NAMES])
        if not np.all(np.isfinite(measured)):
            return None
        return measured

    def publish_goal(self):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.point.x = self.args.base_x
        message.point.y = self.args.base_y
        self.goal_publisher.publish(message)
        self.goal_sent = True
        self.result["goal_sent_at_s"] = round(time.monotonic() - self.started_at, 3)

    def publish_arm_target(self, joints, gripper):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = [float(value) for value in joints] + [float(gripper)] * 2
        self.arm_publisher.publish(message)
        self.last_arm_target = np.asarray(joints, dtype=float)
        self.last_gripper_target = gripper

    def precompute_paths(self):
        current = self.measured_arm()
        if current is None:
            return False
        lower = self.kinematics.lower_limits
        upper = self.kinematics.upper_limits
        if np.any(current < lower - 0.02) or np.any(current > upper + 0.02):
            self.abort("ARM_STATE_OUTSIDE_URDF_LIMITS")
            return False
        try:
            self.plan["initial_home"] = self.trajectory.minimum_jerk_samples(
                current, HOME_Q, self.args.arm_duration_s, TARGET_RATE_HZ, 0.05
            )
            for name, start, target in (
                ("pregrasp", HOME_Q, self.kinematics.forward(PREGRASP_Q)),
                ("target", PREGRASP_Q, self.target_pose),
            ):
                points = self.trajectory.track_cartesian_trajectory(
                    self.kinematics, start, target,
                    duration_s=self.args.arm_duration_s,
                    rate_hz=TARGET_RATE_HZ,
                    max_joint_step_rad=0.05,
                    joint_margin_rad=0.05,
                    position_tolerance_m=0.002,
                    rotation_tolerance_deg=2.0,
                )
                self.plan[name] = points
                self.result["ik"][name] = {
                    "success": True,
                    "samples": len(points),
                    "target": pose_summary(target),
                }
            target_joints = self.plan["target"][-1]
            home_pose = self.kinematics.forward(HOME_Q)
            self.plan["home"] = self.trajectory.track_cartesian_trajectory(
                self.kinematics, target_joints, home_pose,
                duration_s=self.args.arm_duration_s,
                rate_hz=TARGET_RATE_HZ,
                max_joint_step_rad=0.05,
                joint_margin_rad=0.05,
                position_tolerance_m=0.002,
                rotation_tolerance_deg=2.0,
            )
            self.result["ik"]["home"] = {
                "success": True,
                "samples": len(self.plan["home"]),
                "target": pose_summary(home_pose),
            }
            self.plan["rest"] = self.trajectory.minimum_jerk_samples(
                self.plan["home"][-1], REST_Q,
                self.args.arm_duration_s, TARGET_RATE_HZ, 0.05,
            )
        except Exception as error:
            self.abort("IK_UNREACHABLE:" + str(error))
            return False
        return True

    def begin_stage(self, stage):
        self.stage = stage
        self.samples = deque(self.plan[stage])
        self.stage_started_at = time.monotonic()
        self.result["states"].append({
            "at_s": round(self.stage_started_at - self.started_at, 3),
            "stage": stage,
        })

    def stream_stage(self):
        if self.navigation_active:
            self.abort("SIMULTANEOUS_NAVIGATION_AND_MANIPULATION")
            return False
        if self.samples:
            self.publish_arm_target(self.samples.popleft(), GRIPPER_OPEN_M)
            return False
        if self.last_arm_target is not None:
            self.publish_arm_target(self.last_arm_target, GRIPPER_OPEN_M)
        actual = self.measured_arm()
        if actual is not None and self.last_arm_target is not None:
            error = float(np.max(np.abs(actual - self.last_arm_target)))
            self.result["arm_joint_error_rad"] = error
            if error <= self.args.arm_joint_tolerance_rad:
                return True
        expected = len(self.plan[self.stage]) / TARGET_RATE_HZ
        if time.monotonic() - self.stage_started_at > expected + 6.0:
            self.abort("ARM_TRACKING_TIMEOUT:" + self.stage)
        return False

    def save_result(self):
        self.result["duration_s"] = round(time.monotonic() - self.started_at, 3)
        output = Path(self.args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.result, indent=2, sort_keys=True) + "\n")
        self.get_logger().info("M0 %s: %s" % (self.state, output))

    def abort(self, reason):
        if self.state in ("ABORT", "COMPLETE"):
            return
        self.result["complete"] = False
        self.result["abort_reason"] = reason
        self.transition("ABORT", reason=reason)
        self.save_result()

    def complete(self):
        pose = self.base_pose()
        if pose is not None:
            x, y, heading = pose
            yaw_delta = math.atan2(
                math.sin(heading - math.radians(self.args.base_yaw_deg)),
                math.cos(heading - math.radians(self.args.base_yaw_deg)),
            )
            self.result["final_base_pose"] = {
                "x": x, "y": y, "yaw_deg": math.degrees(heading)
            }
            self.result["final_xy_error_m"] = math.hypot(
                x - self.args.base_x, y - self.args.base_y
            )
            self.result["final_yaw_error_deg"] = abs(math.degrees(yaw_delta))
        self.result.update({
            "manip_anchor": self.anchor,
            "max_anchor_xy_error_m": self.max_anchor_xy_m,
            "max_anchor_yaw_error_deg": math.degrees(self.max_anchor_yaw_rad),
            "complete": True,
        })
        self.transition("COMPLETE")
        self.save_result()

    def tick(self):
        now = time.monotonic()
        if self.state in ("COMPLETE", "ABORT"):
            return
        if now - self.started_at > self.args.timeout_s:
            self.abort("TIMEOUT")
            return
        if self.state == "NAV_TO_BASE_GOAL":
            if not self.goal_sent and self.navigation_ready and self.goal_publisher.get_subscription_count():
                self.publish_goal()
            if self.goal_sent and self.goal_reached:
                self.transition("BASE_SETTLE")
            return
        if self.state == "BASE_SETTLE":
            if now - self.odometry_at <= 0.25 and self.settle_count >= self.args.settle_samples:
                x, y, heading = self.base_pose()
                self.anchor = {
                    "x": x, "y": y, "yaw_deg": math.degrees(heading)
                }
                self.transition("SAVE_MANIP_ANCHOR")
            return
        if self.state == "SAVE_MANIP_ANCHOR":
            if self.navigation_active:
                if now - self.state_started_at > 5.0:
                    self.abort("NAVIGATION_STILL_ACTIVE")
                return
            if self.measured_arm() is None:
                if now - self.state_started_at > 5.0:
                    self.abort("ARM_STATE_UNAVAILABLE")
                return
            if self.precompute_paths():
                self.begin_stage("initial_home")
                self.transition("ARM_PREGRASP")
            return
        if self.state == "ARM_PREGRASP":
            if not self.stream_stage():
                return
            if self.stage == "initial_home":
                self.begin_stage("pregrasp")
            else:
                self.begin_stage("target")
                self.transition("ARM_TARGET")
            return
        if self.state == "ARM_TARGET":
            if self.stream_stage():
                self.transition("SIM_GRASP")
            return
        if self.state == "SIM_GRASP":
            if self.navigation_active:
                self.abort("SIMULTANEOUS_NAVIGATION_AND_MANIPULATION")
                return
            self.publish_arm_target(self.plan["target"][-1], GRIPPER_CLOSED_M)
            if now - self.state_started_at < self.args.grasp_hold_s:
                return
            if time.monotonic() - self.arm_measurements_at > 0.25:
                self.abort("GRIPPER_STATE_UNAVAILABLE")
                return
            gripper_error = max(
                abs(self.arm_measurements[name] - GRIPPER_CLOSED_M)
                for name in GRIPPER_NAMES
            )
            self.result["gripper_error_m"] = gripper_error
            if gripper_error > 0.01:
                self.abort("GRIPPER_NOT_CLOSED")
                return
            self.result["sim_grasp"] = True
            self.publish_status("SIM_GRASP_SUCCESS")
            self.begin_stage("home")
            self.transition("ARM_HOME")
            return
        if self.state == "ARM_HOME" and self.stream_stage():
            if self.stage == "home":
                self.begin_stage("rest")
            else:
                self.complete()


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-x", type=float, required=True)
    parser.add_argument("--base-y", type=float, required=True)
    parser.add_argument("--base-yaw-deg", type=float, default=0.0)
    parser.add_argument(
        "--arm-target", type=float, nargs=6, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="TCP pose in arm base_link, metres and degrees; default is a known reachable FK smoke target",
    )
    parser.add_argument(
        "--output", default="stage4d_runs/nav_manip_m0_%s.json" % time.strftime("%Y%m%d_%H%M%S")
    )
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--arm-duration-s", type=float, default=2.0)
    parser.add_argument("--grasp-hold-s", type=float, default=1.0)
    parser.add_argument("--settle-linear-mps", type=float, default=0.03)
    parser.add_argument("--settle-angular-rps", type=float, default=0.05)
    parser.add_argument("--settle-samples", type=int, default=10)
    parser.add_argument("--arm-joint-tolerance-rad", type=float, default=0.05)
    parser.add_argument(
        "--graspnet-root",
        default=os.environ.get("RARS01_GRASPNET_ROOT", "/home/ruben/go2_diploma_sim2sim/repos/rars01_graspnet"),
    )
    parser.add_argument(
        "--urdf", default="/home/ruben/go2_diploma_sim2sim/repos/rars01_description/urdf/rars01_control.urdf"
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    root = Path(args.graspnet_root)
    urdf = Path(args.urdf)
    if not (root / "rars01_graspnet" / "ik.py").is_file():
        raise SystemExit("Transport-free RARS01 IK package not found: %s" % root)
    if not urdf.is_file():
        raise SystemExit("RARS01 control URDF not found: %s" % urdf)
    sys.path.insert(0, str(root))
    from scipy.spatial.transform import Rotation
    from rars01_graspnet.kinematics import RarsKinematics
    from rars01_graspnet import trajectory

    kinematics = RarsKinematics(urdf, "base_link", "End_link")
    for label, joints in (("HOME", HOME_Q), ("PREGRASP", PREGRASP_Q), ("SMOKE_TARGET", SMOKE_TARGET_Q)):
        if np.any(joints <= kinematics.lower_limits) or np.any(joints >= kinematics.upper_limits):
            raise SystemExit("%s violates existing URDF joint limits" % label)
    target_pose = kinematics.forward(SMOKE_TARGET_Q)
    if args.arm_target is not None:
        x, y, z, roll, pitch, heading = args.arm_target
        target_pose = np.eye(4)
        target_pose[:3, 3] = [x, y, z]
        target_pose[:3, :3] = Rotation.from_euler(
            "xyz", [roll, pitch, heading], degrees=True
        ).as_matrix()
    rclpy.init()
    node = NavManipM0(args, kinematics, trajectory, target_pose)
    try:
        while rclpy.ok() and node.state not in ("COMPLETE", "ABORT"):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.abort("INTERRUPTED")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.state == "COMPLETE" else 2


if __name__ == "__main__":
    sys.exit(main())
