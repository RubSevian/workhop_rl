#!/usr/bin/env python3
"""Simulation-only M0.1 manual target executor for the Stage4D MuJoCo viewer.

The viewer publishes a world-frame target; this node owns base-settle gating,
world-to-RARS01 conversion, IK, arm trajectory and safe cancellation. It never
imports an SDK or opens a physical serial device.
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
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String

ARM_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_NAMES = ("gripper_left_joint", "gripper_right_joint")
JOINT_NAMES = ARM_NAMES + GRIPPER_NAMES
HOME_Q = np.array([0.0, 1.0, 1.0, -0.5, 0.0, 0.0])
REST_Q = np.zeros(6)
GRIPPER_OPEN_M = 0.040
GRIPPER_CLOSED_M = 0.005
TARGET_RATE_HZ = 20.0
# MJCF: base -> arm_mount_link (-.03,0,.058) -> RARS base_link (.074304,0,.0145).
RARS_BASE_OFFSET_IN_GO2 = np.array([0.044304, 0.0, 0.0725])


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ManualManipTarget(Node):
    """Manual MuJoCo target state machine with navigation and settle interlocks."""

    def __init__(self, args, kinematics, trajectory, rotation):
        super().__init__("stage4d_manual_manip_target")
        self.args, self.kinematics, self.trajectory, self.rotation = args, kinematics, trajectory, rotation
        self.arm_pub = self.create_publisher(JointState, "/rars01/arm_target", 10)
        self.status_pub = self.create_publisher(String, "/stage4d/manual_manip_status", 10)
        self._subscriptions = [
            self.create_subscription(PoseStamped, "/stage4d/manual_manip_target", self.on_target, 10),
            self.create_subscription(Empty, "/stage4d/manual_manip_cancel", self.on_cancel, 10),
            self.create_subscription(Bool, "/navigation_active", self.on_navigation_active, 10),
            self.create_subscription(Odometry, "/sim/ground_truth_odom", self.on_odometry, 10),
            self.create_subscription(JointState, "/go2/motor_state", self.on_motor_state, 10),
        ]
        self.timer = self.create_timer(1.0 / TARGET_RATE_HZ, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.state = "MANUAL_IDLE"
        self.navigation_active = False
        self.odometry = None
        self.odometry_at = 0.0
        self.settle_started_at = None
        self.motor = {}
        self.motor_at = 0.0
        self.target_world = None
        self.pending_target = None
        self.run_started_at = None
        self.target_arm = None
        self.anchor = None
        self.max_anchor_xy_m = 0.0
        self.max_anchor_yaw_rad = 0.0
        self.samples = deque()
        self.stage_started_at = time.monotonic()
        self.last_arm_target = None
        self.last_error = ""
        self.cancel_requested = False
        self.result = self.new_result()
        self.run_started_at = time.monotonic()
        self.publish_status()

    def new_result(self):
        return {"schema": "stage4d_manual_manip_m0_1/v1", "simulation_only": True,
                "real_serial_opened": False, "states": [], "complete": False,
                "cancelled": False, "ik": {}, "abort_reason": None}

    def transition(self, state, detail=""):
        self.state = state
        self.stage_started_at = time.monotonic()
        self.result["states"].append({"at_s": round(self.stage_started_at - (self.run_started_at or self.stage_started_at), 3), "state": state, "detail": detail})
        self.publish_status()

    def publish_status(self):
        settle_s = 0.0 if self.settle_started_at is None else time.monotonic() - self.settle_started_at
        fields = [self.state,
                  "NAV=" + ("ACTIVE" if self.navigation_active else "IDLE"),
                  "SETTLE=%.2f/%.2f" % (settle_s, self.args.settle_hold_s),
                  "IK=" + ("OK" if self.target_arm is not None else "-"),
                  "ARM_ERR=" + ("%.3f" % self.arm_error() if self.arm_error() is not None else "-"),
                  "DRIFT=" + ("%.3f m/%.1f deg" % (self.max_anchor_xy_m, math.degrees(self.max_anchor_yaw_rad)))]
        if self.target_arm is not None:
            fields.append("TARGET_ARM=%.2f %.2f %.2f" % tuple(self.target_arm[:3, 3]))
        if self.last_error:
            fields.append(self.last_error)
        msg = String(); msg.data = "; ".join(fields); self.status_pub.publish(msg)

    def on_navigation_active(self, message):
        self.navigation_active = bool(message.data)
        if self.navigation_active and self.state not in ("MANUAL_IDLE", "MANUAL_TARGET_SET"):
            self.request_cancel("NAV ACTIVE -> RETURN HOME")

    def on_odometry(self, message):
        self.odometry, self.odometry_at = message, time.monotonic()
        linear = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        yaw_rate = abs(math.degrees(message.twist.twist.angular.z))
        if linear <= self.args.settle_linear_mps and yaw_rate <= self.args.max_yaw_rate_deg_s:
            if self.settle_started_at is None:
                self.settle_started_at = self.odometry_at
        else:
            self.settle_started_at = None
        if self.anchor is not None:
            x, y, yaw = self.base_pose()
            self.max_anchor_xy_m = max(self.max_anchor_xy_m, math.hypot(x - self.anchor[0], y - self.anchor[1]))
            delta = math.atan2(math.sin(yaw - self.anchor[2]), math.cos(yaw - self.anchor[2]))
            self.max_anchor_yaw_rad = max(self.max_anchor_yaw_rad, abs(delta))

    def on_motor_state(self, message):
        self.motor = dict(zip(message.name, message.position))
        self.motor_at = time.monotonic()

    def on_target(self, message):
        if self.navigation_active and not self.args.allow_manual_manip_during_navigation:
            self.last_error = "MANUAL TARGET REJECTED — NAV ACTIVE"
            self.publish_status()
            return
        if self.state not in ("MANUAL_IDLE", "MANUAL_TARGET_SET"):
            self.pending_target = message
            self.request_cancel("TARGET REPLACED -> RETURN HOME")
            return
        self.result = self.new_result()
        self.run_started_at = time.monotonic()
        self.target_world = np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z])
        self.target_arm = None
        self.anchor = None
        self.max_anchor_xy_m = self.max_anchor_yaw_rad = 0.0
        self.last_error = ""
        self.transition("MANUAL_TARGET_SET", "await sustained base settle")

    def on_cancel(self, _message):
        self.pending_target = None
        if self.state in ("MANUAL_IDLE", "MANUAL_TARGET_SET"):
            self.clear_to_idle("IDLE CLEAR")
        else:
            self.request_cancel("CANCEL -> RETURN HOME")
    def base_pose(self):
        if self.odometry is None:
            return None
        p, q = self.odometry.pose.pose.position, self.odometry.pose.pose.orientation
        return float(p.x), float(p.y), yaw_from_quaternion(q)

    def base_rotation(self):
        q = self.odometry.pose.pose.orientation
        return self.rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    def base_settled(self):
        return (not self.navigation_active and self.odometry is not None and
                time.monotonic() - self.odometry_at <= self.args.max_pose_age_s and
                self.settle_started_at is not None and
                time.monotonic() - self.settle_started_at >= self.args.settle_hold_s)

    def measured_arm(self):
        if time.monotonic() - self.motor_at > 0.25 or not all(name in self.motor for name in JOINT_NAMES):
            return None
        return np.array([self.motor[name] for name in ARM_NAMES])

    def arm_error(self):
        actual = self.measured_arm()
        if actual is None or self.last_arm_target is None:
            return None
        return float(np.max(np.abs(actual - self.last_arm_target)))

    def world_to_arm_target(self):
        x, y, _ = self.base_pose()
        p_base_world = np.array([x, y, self.odometry.pose.pose.position.z])
        p_arm = self.base_rotation().T @ (self.target_world - p_base_world) - RARS_BASE_OFFSET_IN_GO2
        target = np.eye(4)
        target[:3, :3] = self.kinematics.forward(HOME_Q)[:3, :3]
        target[:3, 3] = p_arm
        return target

    def build_plan(self):
        current = self.measured_arm()
        if current is None:
            raise RuntimeError("ARM_STATE_UNAVAILABLE")
        self.target_arm = self.world_to_arm_target()
        pregrasp = self.target_arm.copy()
        pregrasp[:3, 3] -= 0.06 * pregrasp[:3, 0]
        # Check the picked point with the fixed M0.1 TCP orientation before
        # planning a trajectory. A floor click can be visually close but place
        # the target below the arm workspace or outside its joint limits.
        reach = self.trajectory.solve_pose_ik(
            self.kinematics, self.target_arm, HOME_Q, random_starts=8,
            joint_margin_rad=0.05, position_tolerance_m=0.002,
            rotation_tolerance_deg=2.0)
        if not reach.success:
            x, y, z = self.target_arm[:3, 3]
            raise RuntimeError(
                "TARGET_POSE_UNREACHABLE arm=(%.2f,%.2f,%.2f)m; "
                "best error=%.1fmm/%.1fdeg (fixed TCP orientation)" %
                (x, y, z, reach.position_error_m * 1000.0, reach.rotation_error_deg))
        initial = self.trajectory.minimum_jerk_samples(current, HOME_Q, self.args.arm_duration_s, TARGET_RATE_HZ, 0.05)
        pre = self.trajectory.track_cartesian_trajectory(self.kinematics, HOME_Q, pregrasp,
            duration_s=self.args.arm_duration_s, rate_hz=TARGET_RATE_HZ, max_joint_step_rad=0.05,
            joint_margin_rad=0.05, position_tolerance_m=0.002, rotation_tolerance_deg=2.0)
        target = self.trajectory.track_cartesian_trajectory(self.kinematics, pre[-1], self.target_arm,
            duration_s=self.args.arm_duration_s, rate_hz=TARGET_RATE_HZ, max_joint_step_rad=0.05,
            joint_margin_rad=0.05, position_tolerance_m=0.002, rotation_tolerance_deg=2.0)
        home = self.trajectory.track_cartesian_trajectory(self.kinematics, target[-1], self.kinematics.forward(HOME_Q),
            duration_s=self.args.arm_duration_s, rate_hz=TARGET_RATE_HZ, max_joint_step_rad=0.05,
            joint_margin_rad=0.05, position_tolerance_m=0.002, rotation_tolerance_deg=2.0)
        rest = self.trajectory.minimum_jerk_samples(home[-1], REST_Q, self.args.arm_duration_s, TARGET_RATE_HZ, 0.05)
        return {"initial": initial, "pregrasp": pre, "target": target, "home": home, "rest": rest}

    def publish_arm(self, joints, gripper):
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES); msg.position = [float(v) for v in joints] + [gripper, gripper]
        self.arm_pub.publish(msg); self.last_arm_target = np.asarray(joints, dtype=float)

    def begin(self, name, gripper=GRIPPER_OPEN_M):
        self.stage_name, self.stage_gripper = name, gripper
        self.samples = deque(self.plan[name])
        self.stage_started_at = time.monotonic()
        self.transition("MANUAL_" + name.upper())

    def stream(self):
        if self.samples:
            self.publish_arm(self.samples.popleft(), self.stage_gripper)
            return False
        if self.last_arm_target is not None:
            self.publish_arm(self.last_arm_target, self.stage_gripper)
        error = self.arm_error()
        if error is not None and error <= self.args.arm_joint_tolerance_rad:
            return True
        if time.monotonic() - self.stage_started_at > self.args.arm_duration_s + 6.0:
            self.request_cancel("ARM_TRACKING_TIMEOUT -> RETURN HOME")
        return False

    def request_cancel(self, reason):
        current = self.measured_arm()
        self.cancel_requested = True
        self.last_error = reason
        if current is None:
            self.clear_to_idle("CANCELLED: ARM STATE UNAVAILABLE")
            return
        self.plan = {
            "home": self.trajectory.minimum_jerk_samples(current, HOME_Q, self.args.arm_duration_s, TARGET_RATE_HZ, 0.05),
            "rest": self.trajectory.minimum_jerk_samples(HOME_Q, REST_Q, self.args.arm_duration_s, TARGET_RATE_HZ, 0.05),
        }
        self.begin("home")

    def clear_to_idle(self, detail):
        self.result["complete"] = detail == "MANUAL_COMPLETE"
        if detail == "MANUAL_IK_FAILED":
            self.result["abort_reason"] = self.last_error
        self.result["cancelled"] = self.cancel_requested
        self.result["max_anchor_xy_error_m"] = self.max_anchor_xy_m
        self.result["max_anchor_yaw_error_deg"] = math.degrees(self.max_anchor_yaw_rad)
        self.result["detail"] = detail
        self.result["target_world"] = None if self.target_world is None else self.target_world.tolist()
        self.result["target_arm"] = None if self.target_arm is None else self.target_arm[:3, 3].tolist()
        self.result["duration_s"] = 0.0 if self.run_started_at is None else time.monotonic() - self.run_started_at
        path = Path(self.args.output); path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.result, indent=2, sort_keys=True) + "\n")
        pending_target = self.pending_target
        self.pending_target = None
        self.target_world = self.target_arm = self.anchor = None
        self.last_arm_target = None
        self.cancel_requested = False
        self.transition("MANUAL_IDLE", detail)
        if pending_target is not None:
            self.on_target(pending_target)

    def tick(self):
        if self.state == "MANUAL_IDLE":
            return
        if self.navigation_active and not self.args.allow_manual_manip_during_navigation:
            self.request_cancel("NAV ACTIVE -> RETURN HOME")
            return
        if self.state == "MANUAL_TARGET_SET":
            if not self.base_settled():
                self.publish_status()
                return
            self.anchor = self.base_pose()
            self.transition("MANUAL_IK_CHECK")
            return
        if self.state == "MANUAL_IK_CHECK":
            try:
                self.plan = self.build_plan()
                self.result["ik"] = {"success": True, "target_arm": self.target_arm[:3, 3].tolist()}
            except Exception as error:
                self.last_error = "MANUAL_IK_FAILED: " + str(error)
                self.result["ik"] = {"success": False, "error": str(error)}
                self.transition("MANUAL_IK_FAILED", str(error))
                self.clear_to_idle("MANUAL_IK_FAILED")
                return
            self.begin("initial")
            return
        if self.state == "MANUAL_INITIAL" and self.stream():
            self.begin("pregrasp"); return
        if self.state == "MANUAL_PREGRASP" and self.stream():
            self.begin("target"); return
        if self.state == "MANUAL_TARGET" and self.stream():
            self.transition("MANUAL_SIM_GRASP"); return
        if self.state == "MANUAL_SIM_GRASP":
            self.publish_arm(self.plan["target"][-1], GRIPPER_CLOSED_M)
            if time.monotonic() - self.stage_started_at >= self.args.grasp_hold_s:
                self.begin("home")
            return
        if self.state == "MANUAL_HOME" and self.stream():
            self.begin("rest"); return
        if self.state == "MANUAL_REST" and self.stream():
            self.clear_to_idle("MANUAL_COMPLETE" if not self.cancel_requested else "MANUAL_CANCELLED_HOME")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", default="stage4d_runs/manual_m0_1.json")
    p.add_argument("--arm-duration-s", type=float, default=2.0)
    p.add_argument("--grasp-hold-s", type=float, default=1.0)
    p.add_argument("--settle-hold-s", type=float, default=0.75)
    p.add_argument("--settle-linear-mps", type=float, default=0.03)
    p.add_argument("--max-yaw-rate-deg-s", type=float, default=3.0)
    p.add_argument("--max-pose-age-s", type=float, default=0.20)
    p.add_argument("--arm-joint-tolerance-rad", type=float, default=0.05)
    p.add_argument("--allow-manual-manip-during-navigation", action="store_true")
    p.add_argument("--graspnet-root", default=os.environ.get("RARS01_GRASPNET_ROOT", "/home/ruben/go2_diploma_sim2sim/repos/rars01_graspnet"))
    p.add_argument("--urdf", default="/home/ruben/go2_diploma_sim2sim/repos/rars01_description/urdf/rars01_control.urdf")
    return p.parse_known_args()[0]


def main():
    args = parse_args()
    if not (Path(args.graspnet_root) / "rars01_graspnet" / "ik.py").is_file() or not Path(args.urdf).is_file():
        raise SystemExit("M0.1 needs the existing transport-free rars01_graspnet package and control URDF")
    sys.path.insert(0, args.graspnet_root)
    from scipy.spatial.transform import Rotation
    from rars01_graspnet.kinematics import RarsKinematics
    from rars01_graspnet import trajectory
    rclpy.init()
    node = ManualManipTarget(args, RarsKinematics(args.urdf, "base_link", "End_link"), trajectory, Rotation)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.request_cancel("PROCESS INTERRUPTED")
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
