#!/usr/bin/env python3
"""Simulation adapter for the unchanged real RARS grasp IK and 100 Hz motion math."""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pinocchio as pin
from rars01_graspnet.grasp_selection import joint_limit_cost, motion_cost
from rars01_graspnet.ik import pregrasp_transform, tcp_target_from_grasp
from rars01_graspnet.limit_contract import arm_limit_contract, command_positions_within_limits
from rars01_graspnet.pinocchio_math import compute_fk, pad_q_for_model
from rars01_graspnet.real_grasp_ik import IkChecker, _plan_cartesian_segment
from rars01_graspnet.real_grasp_trajectory import joint_motion_samples, waypoint_motion_samples
from stage4d_manual_grasp_geometry import floor_clearance, grasp_orientations
from utils.transforms import mat4_to_pose6d


class OfflineArm:
    """Kinematics only; deliberately has no SDK, serial port or motor methods."""
    def __init__(self, urdf, joints):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.groups = {"arm": SimpleNamespace(num_joints=6)}
        self.joints = np.asarray(joints, dtype=np.float64).reshape(6)

    def load_kinematic_model(self):
        return self.model

    def get_state(self, request_feedback=False):
        del request_feedback
        return self.joints.copy(), None, None

    def forward(self, joints):
        return compute_fk(self.model, pad_q_for_model(self.model, joints, 6),
                          frame_name="End_link")[2]


def _joint_limit_diagnostic(samples, lower, upper):
    """Describe the first violating sample without changing limits or samples."""
    values = np.asarray(samples, dtype=float)
    violations = np.maximum(lower - values, values - upper)
    failed = np.argwhere(~np.isfinite(values) | (violations > 1e-9))
    if not len(failed):
        return None
    sample_index, joint_index = map(int, failed[0])
    value = float(values[sample_index, joint_index])
    return {"sample_index": sample_index, "joint": f"joint{joint_index + 1}",
            "value_rad": value, "lower_rad": float(lower[joint_index]),
            "upper_rad": float(upper[joint_index]),
            "violation_rad": float(violations[sample_index, joint_index])}


def plan_real_grasp(center_arm, measured, world_arm, geometry, grasp_width_m,
                    urdf, config, floor_z_m, floor_margin_m, diagnostics=None):
    """Return the executable real-driver samples or exact rejection reasons."""
    arm = OfflineArm(urdf, measured)
    lower, upper, command_epsilon, limit_source = arm_limit_contract(config, arm.model)
    arm.model.lowerPositionLimit[:6] = lower
    arm.model.upperPositionLimit[:6] = upper
    grasp_cfg = config["grasp_pipeline"]["grasp"]
    cart_cfg = grasp_cfg["cartesian_ik"]
    motion_cfg = config["robot"]["motion"]
    ready_cfg = config["robot"]["ready_pose"]
    home = np.asarray(config["robot"]["rars01"]["home_joints_rad"], dtype=float).reshape(6)
    velocity_limits = np.asarray(config["robot"]["rars01"]["position_velocity_limits_rad_s"][:6], dtype=float)
    dt = 1.0 / float(config["robot"]["rars01"]["command_rate_hz"])
    if abs(dt - 0.01) > 1e-9:
        raise ValueError("REAL_COMMAND_RATE_NOT_100HZ")
    checker = IkChecker(arm, retry_count=int(grasp_cfg["ik_retry_count"]),
        position_tolerance_m=float(cart_cfg["ik_position_tolerance_m"]),
        orientation_tolerance_rad=np.deg2rad(float(cart_cfg["ik_orientation_tolerance_deg"])))
    if diagnostics is not None:
        diagnostics.update({"measured_joints_rad": np.asarray(measured, dtype=float).tolist(),
            "joint_lower_rad": lower.tolist(), "joint_upper_rad": upper.tolist(),
            "limit_source": limit_source,
            "grasp_center_arm_xyz_m": np.asarray(center_arm, dtype=float).tolist(),
            "ik_position_tolerance_m": float(cart_cfg["ik_position_tolerance_m"]),
            "ik_orientation_tolerance_deg": float(cart_cfg["ik_orientation_tolerance_deg"]),
            "candidates": []})
        diagnostics["measured_limit_violation"] = _joint_limit_diagnostic(
            np.asarray(measured, dtype=float).reshape(1, 6), lower, upper)
    try:
        command_start = command_positions_within_limits(
            np.asarray(measured, dtype=float), lower, upper, command_epsilon)
    except RuntimeError as error:
        if diagnostics is not None:
            diagnostics["start_command_rejection"] = str(error)
        raise
    if diagnostics is not None:
        diagnostics["initial_command_joints_rad"] = command_start.tolist()
        diagnostics["initial_command_clamp_rad"] = (command_start - measured).tolist()
    ready_pose = tuple(float(ready_cfg[key]) for key in ("x", "y", "z", "roll", "pitch")) + (0.0,)
    ready = checker.solve(*ready_pose, reference_joints=command_start)
    if diagnostics is not None:
        diagnostics["ready_target_pose6d"] = list(ready_pose)
        diagnostics["ready_ik"] = {"success": bool(ready.success), "solver_error": float(ready.error),
            "fk_position_error_m": float(ready.position_error_m),
            "fk_orientation_error_deg": float(np.rad2deg(ready.orientation_error_rad)),
            "joints_rad": ready.joints.tolist(),
            "limit_violation": _joint_limit_diagnostic(ready.joints.reshape(1, 6), lower, upper)}
    if not ready.success:
        raise RuntimeError("REAL_READY_IK_FAILED")
    opening = geometry.solve_opening(grasp_width_m)
    candidates, rejects = [], []
    candidate_diagnostics = {}
    for name, rotation in grasp_orientations():
        grasp = np.eye(4); grasp[:3, :3] = rotation; grasp[:3, 3] = center_arm
        pre = pregrasp_transform(grasp, float(grasp_cfg["pregrasp_offset_m"]))
        retreat = pregrasp_transform(grasp, float(grasp_cfg["pregrasp_offset_m"]))
        grasp_tcp = tcp_target_from_grasp(grasp, opening.T_grasp_End_link)
        pre_tcp = tcp_target_from_grasp(pre, opening.T_grasp_End_link)
        retreat_tcp = tcp_target_from_grasp(retreat, opening.T_grasp_End_link)
        pre6, grasp6, retreat6 = map(mat4_to_pose6d, (pre_tcp, grasp_tcp, retreat_tcp))
        candidate_diagnostic = {"orientation": name, "pregrasp_tcp_pose6d": list(pre6),
            "grasp_tcp_pose6d": list(grasp6), "retreat_tcp_pose6d": list(retreat6)}
        candidate_diagnostics[name] = candidate_diagnostic
        if diagnostics is not None:
            diagnostics["candidates"].append(candidate_diagnostic)
        try:
            pre_solution = checker.solve(*pre6, reference_joints=ready.joints)
            candidate_diagnostic["pregrasp_ik"] = {"success": bool(pre_solution.success),
                "solver_error": float(pre_solution.error),
                "fk_position_error_m": float(pre_solution.position_error_m),
                "fk_orientation_error_deg": float(np.rad2deg(pre_solution.orientation_error_rad)),
                "joints_rad": pre_solution.joints.tolist(),
                "limit_violation": _joint_limit_diagnostic(pre_solution.joints.reshape(1, 6), lower, upper)}
            if not pre_solution.success:
                raise RuntimeError("REAL_PREGRASP_IK_FAILED")
            approach = _plan_cartesian_segment(checker, pre6, grasp6, pre_solution.joints,
                                               cart_cfg, "approach")
            if not approach:
                raise RuntimeError("REAL_APPROACH_WAYPOINT_REJECTED")
            retreat_path = _plan_cartesian_segment(checker, grasp6, retreat6,
                                                   approach[-1], cart_cfg, "retreat")
            if not retreat_path:
                raise RuntimeError("REAL_RETREAT_WAYPOINT_REJECTED")
            chain = np.asarray([command_start, ready.joints, pre_solution.joints,
                               *approach, *retreat_path])
            weights = grasp_cfg["candidate_selection"]
            score = (float(weights["weight_joint"]) * joint_limit_cost(chain, lower, upper)
                     + float(weights["weight_motion"]) * motion_cost(
                         chain, lower, upper, float(weights["motion_normalization"])))
            candidates.append((score, name, grasp_tcp, pre_solution.joints,
                               approach, retreat_path))
        except RuntimeError as error:
            candidate_diagnostic["rejection"] = str(error)
            rejects.append({"orientation": name, "reason": str(error)})
    if not candidates:
        raise RuntimeError("NO_REAL_IK_CANDIDATE: " + str(rejects))
    candidates.sort(key=lambda item: item[0])
    for score, name, grasp_tcp, pre_q, approach, retreat_path in candidates:
        candidate_diagnostic = candidate_diagnostics[name]
        try:
            stages = {}
            durations = {}
            stages["initial"], durations["initial"] = joint_motion_samples(
                command_start, ready.joints, float(ready_cfg["duration"]), dt, velocity_limits)
            stages["pregrasp"], durations["pregrasp"] = joint_motion_samples(
                ready.joints, pre_q, float(motion_cfg["pregrasp_duration_s"]), dt, velocity_limits)
            stages["target"], durations["target"] = waypoint_motion_samples(
                pre_q, approach, float(motion_cfg["grasp_duration_s"]), dt, velocity_limits)
            stages["retreat"], durations["retreat"] = waypoint_motion_samples(
                approach[-1], retreat_path, float(motion_cfg["retreat_duration_s"]), dt, velocity_limits)
            stages["home"], durations["home"] = joint_motion_samples(
                retreat_path[-1], home, float(ready_cfg["duration"]), dt, velocity_limits)
            for stage, samples in stages.items():
                values = np.asarray(samples)
                limit_diagnostic = _joint_limit_diagnostic(values, lower, upper)
                if limit_diagnostic is not None:
                    candidate_diagnostic["stage_limit_violation"] = {"stage": stage, **limit_diagnostic}
                if not np.all(np.isfinite(values)) or np.any(values < lower - 1e-9) or np.any(values > upper + 1e-9):
                    raise RuntimeError("JOINT_LIMIT_REJECTED stage=" + stage)
                if np.max(np.abs(np.diff(values, axis=0))) / dt > np.max(velocity_limits) + 1e-8:
                    raise RuntimeError("VELOCITY_LIMIT_REJECTED stage=" + stage)
            clearance = floor_clearance(stages, arm, geometry, world_arm,
                                        floor_z_m, floor_margin_m, grasp_width_m)
            all_samples = np.concatenate([np.asarray(stages[key]) for key in stages])
            peak_velocity = np.max(np.abs(np.diff(all_samples, axis=0)), axis=0) / dt
            if np.any(peak_velocity > velocity_limits + 1e-8):
                raise RuntimeError("VELOCITY_LIMIT_REJECTED stage_boundary")
            return stages, {"orientation": name, "cost": score,
                "target_tcp": grasp_tcp, "ready_joints": ready.joints,
                "approach_waypoints": len(approach), "retreat_waypoints": len(retreat_path),
                "max_waypoint_step_rad": max(float(np.max(np.abs(q2-q1))) for q1,q2 in zip(
                    [pre_q,*approach[:-1]], approach)),
                "durations_s": durations, "sample_count": len(all_samples),
                "command_rate_hz": 1.0/dt, "max_velocity_rad_s": peak_velocity.tolist(),
                "velocity_limits_rad_s": velocity_limits.tolist(),
                "floor_clearance": clearance, "candidate_rejections": rejects}
        except (RuntimeError, ValueError) as error:
            candidate_diagnostic["rejection"] = str(error)
            rejects.append({"orientation": name, "reason": str(error)})
    raise RuntimeError("NO_SAFE_REAL_TRAJECTORY: " + str(rejects))
