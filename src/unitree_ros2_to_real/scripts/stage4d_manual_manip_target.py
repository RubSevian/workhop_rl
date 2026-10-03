#!/usr/bin/env python3
"""Simulation-only M0.1 manual target executor for the Stage4D MuJoCo viewer.

The viewer publishes a world-frame target; this node owns base-settle gating,
world-to-RARS01 conversion, IK, arm trajectory and safe cancellation. It never
imports an SDK or opens a physical serial device.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from stage4d_manual_grasp_geometry import (
    floor_clearance, gripper_from_project_config, message_pose_matrix,
    stamp_seconds, static_mount_contract, transform_error,
)
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String, Float64MultiArray

ARM_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_NAMES = ("gripper_left_joint", "gripper_right_joint")
JOINT_NAMES = ARM_NAMES + GRIPPER_NAMES
GRIPPER_OPEN_M = 0.040
GRIPPER_CLOSED_M = 0.005
TARGET_RATE_HZ = 100.0
SHADOW_CHECK_TIMEOUT_S = 90.0  # Full 100 Hz path is checked before any arm command.


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ManualManipTarget(Node):
    """Manual MuJoCo target state machine with navigation and settle interlocks."""

    def __init__(self, args, kinematics, trajectory, geometry, mount_contract):
        super().__init__("stage4d_manual_manip_target")
        self.args, self.kinematics, self.trajectory = args, kinematics, trajectory
        self.sensor_group = ReentrantCallbackGroup()
        self.geometry, self.mount_contract, self.mount_transform = geometry, mount_contract[0], mount_contract[1]
        self.arm_base_pose = None
        self.arm_base_at = 0.0
        self.target_stamp = None
        self.target_frame = None
        self.world_arm = None
        self.arm_pub = self.create_publisher(JointState, "/rars01/arm_target", 10)
        self.payload_pub = self.create_publisher(Bool, "/stage4d/payload_attach", 10)
        self.payload_status = {}
        self.payload_group = MutuallyExclusiveCallbackGroup()
        self.payload_reset_pending = False
        self.payload_anchor = None
        self.payload_hold_start = None
        self.payload_csv = None
        self.payload_last_stamp = None
        self.payload_last_flush = time.monotonic()
        if args.payload_enabled:
            path = Path(args.payload_csv)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.payload_csv = path.open('w', newline='', buffering=65536)
            self.payload_writer = csv.writer(self.payload_csv)
            self.payload_writer.writerow(("sim_time_s", "payload_attached", "payload_mass_kg",
                "base_x_m", "base_y_m", "base_roll_rad", "base_pitch_rad", "base_yaw_rad",
                "anchor_x_m", "anchor_y_m", "anchor_yaw_rad", "base_xy_drift_m",
                "base_yaw_drift_rad", "arm_tracking_max_rad"))
        self.status_pub = self.create_publisher(String, "/stage4d/manual_manip_status", 10)
        self.collision_pub = self.create_publisher(Float64MultiArray, "/stage4d/arm_trajectory_check_request", 10)
        self.collision_request_id = None
        self.collision_response = None
        self._subscriptions = [
            self.create_subscription(String, "/stage4d/payload_status", self.on_payload_status,
                                     rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL),
                                     callback_group=self.payload_group),
            self.create_subscription(PoseStamped, "/stage4d/manual_manip_target", self.on_target, 10),
            self.create_subscription(Empty, "/stage4d/manual_manip_cancel", self.on_cancel, 10),
            self.create_subscription(Bool, "/navigation_active", self.on_navigation_active, 10),
            self.create_subscription(Odometry, "/sim/ground_truth_odom", self.on_odometry, 10, callback_group=self.sensor_group),
            self.create_subscription(PoseStamped, "/sim/rars_base_pose", self.on_arm_base_pose, 10, callback_group=self.sensor_group),
            self.create_subscription(JointState, "/go2/motor_state", self.on_motor_state, 10, callback_group=self.sensor_group),
            self.create_subscription(String, "/stage4d/arm_trajectory_check_result", self.on_collision_result, 10),
        ]
        self.timer = self.create_timer(1.0 / TARGET_RATE_HZ, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.payload_timer = self.create_timer(0.02, self.record_payload,
            clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=self.payload_group)
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
        self.cancel_home_pending = False
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

    def on_collision_result(self, message):
        try:
            response = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if response.get("id") == self.collision_request_id:
            self.collision_response = response

    def on_payload_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        previous = self.payload_status
        self.payload_status = status
        if status.get("reset_count", 0) != previous.get("reset_count", 0):
            self.payload_anchor = None
            self.payload_reset_pending = True
        if status.get("payload_attached") and not previous.get("payload_attached"):
            self.payload_anchor = self.base_pose()

    def record_payload(self):
        if self.payload_csv is None or self.odometry is None:
            return
        stamp = self.payload_status.get("sim_time_s")
        if stamp is None or stamp == self.payload_last_stamp:
            return
        self.payload_last_stamp = stamp
        x, y, yaw = self.base_pose()
        q = self.odometry.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        anchor = self.payload_anchor or (x, y, yaw)
        yaw_drift = math.atan2(math.sin(yaw-anchor[2]), math.cos(yaw-anchor[2]))
        error = self.arm_error()
        self.payload_writer.writerow((stamp, int(self.payload_status.get("payload_attached", False)),
            self.payload_status.get("payload_mass_kg"), x, y, roll, pitch, yaw,
            *anchor, math.hypot(x-anchor[0], y-anchor[1]), yaw_drift,
            float('nan') if error is None else error))
        if time.monotonic() - self.payload_last_flush >= 1.0:
            self.payload_csv.flush()
            self.payload_last_flush = time.monotonic()
    def on_arm_base_pose(self, message):
        self.arm_base_pose, self.arm_base_at = message, time.monotonic()
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
        self.target_stamp = stamp_seconds(message.header.stamp)
        self.target_frame = message.header.frame_id
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
        if self.target_frame != "map" or self.odometry is None or self.arm_base_pose is None:
            raise RuntimeError("FRAME_CONTRACT_FAILED: target/map or authoritative pose missing")
        if time.monotonic() - self.arm_base_at > self.args.max_pose_age_s:
            raise RuntimeError("FRAME_CONTRACT_FAILED: stale RARS base pose")
        if self.mount_contract["state"] == "FAILED":
            raise RuntimeError("STATIC_MOUNT_CONTRACT_FAILED")
        go2 = message_pose_matrix(self.odometry.pose.pose)
        arm = message_pose_matrix(self.arm_base_pose.pose)
        reconstructed = go2 @ self.mount_transform
        distance, angle = transform_error(reconstructed, arm)
        stamps = [self.target_stamp, stamp_seconds(self.odometry.header.stamp),
                  stamp_seconds(self.arm_base_pose.header.stamp)]
        skew_ms = (max(stamps) - min(stamps)) * 1000.0
        state = ("FAILED" if distance > self.args.frame_fail_translation_m or angle > self.args.frame_fail_rotation_deg
                 else "WARNING" if distance > self.args.frame_warn_translation_m or angle > self.args.frame_warn_rotation_deg
                 else "GOOD")
        self.result["frame"] = {"state": state, "static_mount": self.mount_contract,
            "target_world_xyz_m": self.target_world.tolist(), "go2_world_xyz_m": go2[:3, 3].tolist(),
            "go2_world_quaternion_xyzw": [getattr(self.odometry.pose.pose.orientation, key) for key in ("x", "y", "z", "w")],
            "rars_world_xyz_m": arm[:3, 3].tolist(), "reconstructed_rars_world_xyz_m": reconstructed[:3, 3].tolist(),
            "translation_difference_mm": distance * 1000.0, "rotation_difference_deg": angle,
            "target_stamp_s": stamps[0], "go2_stamp_s": stamps[1], "rars_stamp_s": stamps[2],
            "max_stamp_skew_ms": skew_ms}
        if state == "FAILED":
            raise RuntimeError("FRAME_CONTRACT_FAILED %.1fmm/%.2fdeg skew=%.1fms" % (distance * 1000, angle, skew_ms))
        if state == "WARNING":
            self.get_logger().warn("FRAME_CONTRACT_WARNING %.1fmm/%.2fdeg skew=%.1fms" % (distance * 1000, angle, skew_ms))
        self.world_arm = arm
        center = np.linalg.inv(arm) @ np.r_[self.target_world, 1.0]
        self.result["frame"]["grasp_center_arm_xyz_m"] = center[:3].tolist()
        return center[:3]

    def build_plan(self):
        current = self.measured_arm()
        if current is None:
            raise RuntimeError("ARM_STATE_UNAVAILABLE")
        center = self.world_to_arm_target()
        from stage4d_real_grasp_planner import plan_manual_ik, plan_real_grasp
        diagnostics = {}
        self.result["planning_diagnostics"] = diagnostics
        planner = plan_manual_ik if self.args.manual_target_mode == "ik_only" else plan_real_grasp
        stages, metadata = planner(
            center, current, self.world_arm, self.geometry,
            self.args.virtual_grasp_width_m, self.args.urdf,
            self.real_config, self.args.floor_z_m, self.args.floor_margin_m, diagnostics=diagnostics)
        self.target_arm = metadata.pop("target_tcp")
        orientation = metadata.pop("orientation")
        if self.args.manual_target_mode == "ik_only":
            self.result["manual_target"] = {
                "mode": "ik_only", "selected_orientation": orientation,
                "target_arm_xyz_m": center.tolist(),
                "end_link_arm_xyz_m": self.target_arm[:3, 3].tolist()}
        else:
            self.result["grasp"] = {
                "selected_orientation": orientation,
                "grasp_center_arm_xyz_m": center.tolist(),
                "grasp_tcp_arm_xyz_m": self.target_arm[:3, 3].tolist(),
                "virtual_grasp_width_m": self.args.virtual_grasp_width_m}
        self.result["floor_clearance"] = metadata.pop("floor_clearance")
        metadata["ready_joints"] = metadata["ready_joints"].tolist()
        report_key = ("manual_ik_trajectory" if self.args.manual_target_mode == "ik_only"
                      else "real_grasp_trajectory")
        self.result[report_key] = metadata
        return stages

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
        if time.monotonic() - self.stage_started_at > max(self.args.arm_duration_s, len(self.plan[self.stage_name]) / TARGET_RATE_HZ) + 6.0:
            self.request_cancel("ARM_TRACKING_TIMEOUT -> RETURN HOME")
        return False

    def request_cancel(self, reason):
        self.cancel_requested = True
        self.last_error = reason
        if self.state in ("MANUAL_TARGET_SET", "MANUAL_IK_CHECK", "MANUAL_COLLISION_CHECK"):
            self.clear_to_idle("MANUAL_CANCELLED_NO_ARM_COMMAND")
            return
        current = self.measured_arm()
        if (current is None or self.arm_base_pose is None or
                time.monotonic() - self.arm_base_at > self.args.max_pose_age_s):
            self.clear_to_idle("CANCELLED: ARM/BASE STATE UNAVAILABLE")
            return
        from rars01_graspnet.real_grasp_trajectory import joint_motion_samples
        from stage4d_real_grasp_planner import OfflineArm
        config = self.real_config["robot"]
        rate = float(config["rars01"]["command_rate_hz"])
        limits = np.asarray(config["rars01"]["position_velocity_limits_rad_s"][:6])
        home_q = np.asarray(config["rars01"]["home_joints_rad"], dtype=float).reshape(6)
        arm = OfflineArm(self.args.urdf, current)
        try:
            home, duration = joint_motion_samples(current, home_q,
                float(config["ready_pose"]["duration"]), 1.0/rate, limits)
            floor_clearance({"home": home}, arm, self.geometry,
                message_pose_matrix(self.arm_base_pose.pose), self.args.floor_z_m,
                self.args.floor_margin_m, self.args.virtual_grasp_width_m)
        except (RuntimeError, ValueError) as error:
            self.last_error = "CANCEL_HOME_UNSAFE: " + str(error)
            self.clear_to_idle("MANUAL_CANCELLED_NO_UNVALIDATED_HOME")
            return
        self.plan = {"home": home}
        self.cancel_home_pending = True
        self.world_arm = message_pose_matrix(self.arm_base_pose.pose)
        self.result["cancel_home"] = {"duration_s": duration, "samples": len(home)}
        self.collision_request_id = int(time.monotonic_ns() % 1000000000)
        self.collision_response = None
        request = Float64MultiArray()
        data = []
        for joints in home:
            data.extend([float(value) for value in joints])
            data.extend([self.stage_gripper, self.stage_gripper])
        request.data = [float(self.collision_request_id), float(len(home))] + data
        if self.collision_pub.get_subscription_count() == 0:
            self.clear_to_idle("MANUAL_CANCELLED_SHADOW_UNAVAILABLE")
            return
        self.collision_pub.publish(request)
        self.transition("MANUAL_COLLISION_CHECK", "validate cancel HOME before command")

    def clear_to_idle(self, detail):
        self.result["complete"] = detail == "MANUAL_COMPLETE"
        if detail != "MANUAL_COMPLETE":
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
        self.collision_request_id = None
        self.collision_response = None
        self.target_world = self.target_arm = self.anchor = None
        self.last_arm_target = None
        self.cancel_requested = False
        self.cancel_home_pending = False
        self.transition("MANUAL_IDLE", detail)
        if pending_target is not None:
            self.on_target(pending_target)

    def tick(self):
        if self.payload_reset_pending:
            self.payload_reset_pending = False
            self.pending_target = None
            if self.args.payload_enabled and self.state != "MANUAL_IDLE":
                self.last_error = "PAYLOAD_SIMULATOR_RESET"
                self.clear_to_idle("PAYLOAD_SIMULATOR_RESET")
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
            samples = []
            stages = (("initial", "target", "home") if self.args.manual_target_mode == "ik_only"
                      else ("initial", "pregrasp", "target", "retreat", "home"))
            for stage in stages:
                gripper = (GRIPPER_OPEN_M if self.args.manual_target_mode == "ik_only" or
                           stage in ("initial", "pregrasp", "target") else GRIPPER_CLOSED_M)
                for joints in self.plan[stage]:
                    samples.extend([float(v) for v in joints])
                    samples.extend([gripper, gripper])
                if stage == "target" and self.args.manual_target_mode != "ik_only":
                    samples.extend([float(v) for v in self.plan["target"][-1]])
                    samples.extend([GRIPPER_CLOSED_M, GRIPPER_CLOSED_M])
            self.collision_request_id = int(time.monotonic_ns() % 1000000000)
            self.collision_response = None
            request = Float64MultiArray()
            request.data = [float(self.collision_request_id), float(len(samples) // 8)] + samples
            if self.collision_pub.get_subscription_count() == 0:
                self.last_error = "SHADOW_CHECKER_UNAVAILABLE"
                self.transition("MANUAL_COLLISION_REJECTED", self.last_error)
                self.clear_to_idle("MANUAL_COLLISION_REJECTED")
                return
            self.collision_pub.publish(request)
            self.transition("MANUAL_COLLISION_CHECK")
            return
        if self.state == "MANUAL_COLLISION_CHECK":
            if self.collision_response is not None:
                self.result["collision"] = self.collision_response
                if not self.collision_response.get("ok", False):
                    self.last_error = self.collision_response.get("reason", "COLLISION_REJECTED")
                    self.transition("MANUAL_COLLISION_REJECTED", self.last_error)
                    self.clear_to_idle("MANUAL_COLLISION_REJECTED")
                    return
                measured = self.measured_arm()
                if (not self.base_settled() or self.arm_base_pose is None or
                        time.monotonic() - self.arm_base_at > self.args.max_pose_age_s or
                        measured is None):
                    self.last_error = "START_STATE_STALE_AFTER_IK"
                    self.transition("MANUAL_COLLISION_REJECTED", self.last_error)
                    self.clear_to_idle("MANUAL_COLLISION_REJECTED")
                    return
                shift_m, turn_deg = transform_error(
                    self.world_arm, message_pose_matrix(self.arm_base_pose.pose))
                first_stage = "home" if self.cancel_home_pending else "initial"
                joint_shift = float(np.max(np.abs(measured - self.plan[first_stage][0])))
                frame_warning = (shift_m > self.args.frame_warn_translation_m or
                                 turn_deg > self.args.frame_warn_rotation_deg)
                self.result["start_state_recheck"] = {
                    "base_translation_m": shift_m, "base_rotation_deg": turn_deg,
                    "arm_joint_shift_rad": joint_shift,
                    "frame_state": "WARNING" if frame_warning else "GOOD"}
                if frame_warning:
                    self.get_logger().warn("START_FRAME_WARNING %.1fmm/%.2fdeg" %
                                           (shift_m * 1000.0, turn_deg))
                if (shift_m > self.args.frame_fail_translation_m or
                        turn_deg > self.args.frame_fail_rotation_deg or
                        joint_shift > self.args.arm_joint_tolerance_rad):
                    self.last_error = "START_STATE_CHANGED_AFTER_IK"
                    self.transition("MANUAL_COLLISION_REJECTED", self.last_error)
                    self.clear_to_idle("MANUAL_COLLISION_REJECTED")
                    return
                self.begin(first_stage, self.stage_gripper if self.cancel_home_pending else GRIPPER_OPEN_M)
                self.cancel_home_pending = False
                return
            if time.monotonic() - self.stage_started_at > SHADOW_CHECK_TIMEOUT_S:
                self.last_error = "SHADOW_CHECK_TIMEOUT"
                self.transition("MANUAL_COLLISION_REJECTED", self.last_error)
                self.clear_to_idle("MANUAL_COLLISION_REJECTED")
            return
        if self.state == "MANUAL_INITIAL" and self.stream():
            self.begin("target" if self.args.manual_target_mode == "ik_only" else "pregrasp"); return
        if self.state == "MANUAL_PREGRASP" and self.stream():
            self.begin("target"); return
        if self.state == "MANUAL_TARGET" and self.stream():
            if self.args.payload_enabled:
                self.payload_pub.publish(Bool(data=True))
                self.result["payload_attach_request"] = {"count": 1,
                    "sim_time_s": self.payload_status.get("sim_time_s")}
                self.transition("MANUAL_PAYLOAD_ATTACH")
            elif self.args.manual_target_mode == "ik_only":
                self.begin("home", GRIPPER_OPEN_M)
            else:
                self.transition("MANUAL_SIM_GRASP")
            return
        if self.state == "MANUAL_PAYLOAD_ATTACH":
            self.publish_arm(self.plan["target"][-1], self.stage_gripper)
            if self.payload_status.get("error"):
                self.request_cancel("PAYLOAD_ATTACH_FAILED: " + self.payload_status["error"])
            elif self.payload_status.get("payload_attached"):
                self.result["payload"] = dict(self.payload_status)
                self.payload_hold_start = self.payload_status["sim_time_s"]
                self.transition("MANUAL_PAYLOAD_HOLD")
            elif time.monotonic() - self.stage_started_at > 5.0:
                self.request_cancel("PAYLOAD_ATTACH_TIMEOUT")
            return
        if self.state == "MANUAL_PAYLOAD_HOLD":
            self.publish_arm(self.plan["target"][-1], self.stage_gripper)
            if self.payload_status.get("sim_time_s", 0) - self.payload_hold_start >= self.args.payload_hold_s:
                if self.args.manual_target_mode == "ik_only":
                    self.begin("home", GRIPPER_OPEN_M)
                else:
                    self.transition("MANUAL_SIM_GRASP")
            return
        if self.state == "MANUAL_SIM_GRASP":
            self.publish_arm(self.plan["target"][-1], GRIPPER_CLOSED_M)
            if time.monotonic() - self.stage_started_at >= self.args.grasp_hold_s:
                self.begin("retreat", GRIPPER_CLOSED_M)
            return
        if self.state == "MANUAL_RETREAT" and self.stream():
            self.begin("home", GRIPPER_CLOSED_M); return
        if self.state == "MANUAL_HOME" and self.stream():
            if self.cancel_requested:
                self.clear_to_idle("MANUAL_CANCELLED_HOME")
            elif (self.max_anchor_xy_m > self.args.frame_fail_translation_m or
                  math.degrees(self.max_anchor_yaw_rad) > self.args.frame_fail_rotation_deg):
                self.last_error = "BASE_DRIFT_EXCEEDED"
                self.clear_to_idle("MANUAL_BASE_DRIFT_EXCEEDED")
            else:
                self.clear_to_idle("MANUAL_COMPLETE")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", default=os.environ.get('STAGE4D_MANUAL_OUTPUT'))
    p.add_argument("--arm-duration-s", type=float, default=2.0)
    p.add_argument("--grasp-hold-s", type=float, default=1.0)
    p.add_argument("--settle-hold-s", type=float, default=0.75)
    p.add_argument("--settle-linear-mps", type=float, default=0.03)
    p.add_argument("--max-yaw-rate-deg-s", type=float, default=3.0)
    p.add_argument("--max-pose-age-s", type=float, default=0.20)
    p.add_argument("--arm-joint-tolerance-rad", type=float, default=0.05)
    p.add_argument("--allow-manual-manip-during-navigation", action="store_true")
    p.add_argument("--virtual-grasp-width-m", type=float, default=float(os.getenv("STAGE4D_VIRTUAL_GRASP_WIDTH_M", "0.04")))
    p.add_argument("--manual-target-mode", choices=("ik_only", "simulated_grasp"), default="ik_only")
    p.add_argument("--payload-enabled", choices=('true', 'false'), default='false')
    p.add_argument("--payload-hold-s", type=float, default=2.0)
    p.add_argument("--payload-csv", default=os.environ.get('STAGE4D_PAYLOAD_CSV', 'stage4d_runs/payload.csv'))
    p.add_argument("--pregrasp-distance-m", type=float, default=0.08)
    p.add_argument("--floor-z-m", type=float, default=0.0)
    p.add_argument("--floor-margin-m", type=float, default=float(os.getenv("STAGE4D_FLOOR_MARGIN_M", "0.0008")))
    p.add_argument("--frame-warn-translation-m", type=float, default=float(os.getenv("STAGE4D_FRAME_WARN_TRANSLATION_M", "0.005")))
    p.add_argument("--frame-warn-rotation-deg", type=float, default=float(os.getenv("STAGE4D_FRAME_WARN_ROTATION_DEG", "0.5")))
    p.add_argument("--frame-fail-translation-m", type=float, default=float(os.getenv("STAGE4D_FRAME_FAIL_TRANSLATION_M", "0.020")))
    p.add_argument("--frame-fail-rotation-deg", type=float, default=float(os.getenv("STAGE4D_FRAME_FAIL_ROTATION_DEG", "2.0")))
    p.add_argument("--mjcf", default=os.environ.get("STAGE4D_MJCF_PATH", "/home/ruben/go2_diploma_sim2sim/repos/workhop_rl/src/unitree_mujoco/unitree_robots/go2_rars01/go2_rars01.xml"))
    p.add_argument("--graspnet-root", default=os.environ.get("RARS01_GRASPNET_ROOT", "/home/ruben/go2_diploma_sim2sim/repos/rars01_graspnet"))
    p.add_argument("--urdf", default=None, help="optional assertion of grasp default.yaml URDF path")
    args = p.parse_known_args()[0]
    args.payload_enabled = args.payload_enabled == 'true'
    if args.output is None:
        args.output = (str(Path(args.payload_csv).with_name('manual_payload.json'))
                       if args.payload_enabled else 'stage4d_runs/manual_m0_1.json')
    if not math.isfinite(args.payload_hold_s) or args.payload_hold_s < 2:
        p.error('--payload-hold-s must be finite and >= 2 seconds')
    return args


def main():
    args = parse_args()
    if not (Path(args.graspnet_root) / "rars01_graspnet" / "ik.py").is_file():
        raise SystemExit("M0.1 needs the existing transport-free rars01_graspnet package")
        raise SystemExit("M0.1 needs the existing transport-free rars01_graspnet package and control URDF")
    sys.path.insert(0, args.graspnet_root)
    from rars01_graspnet.config import load_config, robot_kinematics_config
    config = load_config()
    canonical_urdf = robot_kinematics_config(config)[0]
    if args.urdf is not None and Path(args.urdf).resolve() != canonical_urdf:
        raise SystemExit("Stage4D URDF override differs from grasp default.yaml")
    if not canonical_urdf.is_file():
        raise SystemExit("RARS01 control URDF from grasp default.yaml is missing")
    args.urdf = str(canonical_urdf)
    from rars01_graspnet.gripper_geometry import RarsGripperGeometry
    geometry = gripper_from_project_config(config, RarsGripperGeometry)
    mount_contract = static_mount_contract(args.mjcf)
    rclpy.init()
    node = ManualManipTarget(args, None, None, geometry, mount_contract)
    node.real_config = config
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node.payload_csv is not None:
            node.payload_csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
