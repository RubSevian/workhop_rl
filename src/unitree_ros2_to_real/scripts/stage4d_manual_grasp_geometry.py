#!/usr/bin/env python3
"""Transport-free geometry and safety checks for Stage4D manual grasp targets."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

MOUNT_OFFSET = np.array([0.044304, 0.0, 0.0725], dtype=float)


def pose_matrix(position, quaternion_xyzw):
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    transform[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    return transform


def message_pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return pose_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def transform_error(expected, actual):
    delta = np.linalg.inv(expected) @ actual
    distance = float(np.linalg.norm(delta[:3, 3]))
    angle = float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude()))
    return distance, angle


def static_mount_contract(model_xml):
    """Check the fixed MJCF chain independently of ROS timing."""
    root = ET.parse(str(model_xml)).getroot()
    mount = root.find(".//body[@name='arm_mount_link']")
    base = None if mount is None else mount.find("body[@name='base_link']")
    if mount is None or base is None:
        raise ValueError("STATIC_MOUNT_CONTRACT_FAILED: fixed RARS bodies absent")

    def fixed_transform(body):
        translation = [float(value) for value in body.get("pos", "0 0 0").split()]
        quat_wxyz = [float(value) for value in body.get("quat", "1 0 0 0").split()]
        if len(translation) != 3 or len(quat_wxyz) != 4:
            raise ValueError("STATIC_MOUNT_CONTRACT_FAILED: malformed fixed body pose")
        return pose_matrix(translation, quat_wxyz[1:] + quat_wxyz[:1])

    actual = fixed_transform(mount) @ fixed_transform(base)
    expected = np.eye(4)
    expected[:3, 3] = MOUNT_OFFSET
    distance, angle = transform_error(expected, actual)
    state = ("FAILED" if distance > 0.010 or angle > 1.0 else
             "WARNING" if distance > 0.001 or angle > 0.1 else "GOOD")
    return {"state": state, "translation_error_m": distance,
            "rotation_error_deg": angle,
            "expected_translation_m": MOUNT_OFFSET.tolist(),
            "model_translation_m": actual[:3, 3].tolist(),
            "model_rotation_xyzw": Rotation.from_matrix(actual[:3, :3]).as_quat().tolist()}, actual


def gripper_from_project_config(config, geometry_type):
    """Use the same parameters as GraspDriver, without importing its SDK."""
    robot = config["robot"]
    rars = robot["rars01"]
    gripper = robot["gripper"]["rars01"]
    return geometry_type(
        linkage_radius_m=gripper["linkage_radius_m"],
        connecting_rod_length_m=gripper["connecting_rod_length_m"],
        carriage_width_m=gripper["carriage_width_m"],
        maximum_width_m=rars["max_grasp_width_m"],
        jaw_center_End_link_m=gripper["jaw_center_End_link_m"],
        jaw_depth_m=gripper["jaw_depth_m"],
        jaw_height_m=gripper["jaw_height_m"],
        R_grasp_End_link=gripper["R_grasp_End_link"],
        motor_angle_limit_rad=abs(gripper["angle_open"]),
    )


def grasp_orientations():
    """+X approaches downward; final branch is GraspNet's supported Rx(pi)."""
    top = Rotation.from_euler("y", 90, degrees=True).as_matrix()
    for name, yaw in (("top", 0), ("top_yaw_plus90", 90), ("top_yaw_minus90", -90)):
        yield name, Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ top
    yield "top_parallel_flip", top @ np.diag([1.0, -1.0, -1.0])


@dataclass
class GraspCandidate:
    name: str
    grasp_center: np.ndarray
    pregrasp_tcp: np.ndarray
    grasp_tcp: np.ndarray
    pre_joints: np.ndarray
    grasp_joints: np.ndarray
    score: float


def select_grasp(kinematics, geometry, center_arm, seed, grasp_width_m, *,
                 pregrasp_distance_m=0.08, random_starts=32, return_all=False):
    from rars01_graspnet.ik import pregrasp_transform, solve_pose_ik, tcp_target_from_grasp

    opening = geometry.solve_opening(grasp_width_m)
    # Necessary (not sufficient) reach bound from the unchanged URDF chain.
    # It avoids minutes of doomed 32-start optimization for far-away clicks.
    max_tcp_radius = sum(float(np.linalg.norm(joint.T_origin[:3, 3]))
                         for joint in kinematics.chain)
    checked = []
    feasible = []
    for name, orientation in grasp_orientations():
        grasp = np.eye(4)
        grasp[:3, :3] = orientation
        grasp[:3, 3] = center_arm
        pregrasp = pregrasp_transform(grasp, pregrasp_distance_m)
        pre_tcp = tcp_target_from_grasp(pregrasp, opening.T_grasp_End_link)
        grasp_tcp = tcp_target_from_grasp(grasp, opening.T_grasp_End_link)
        if max(np.linalg.norm(pre_tcp[:3, 3]), np.linalg.norm(grasp_tcp[:3, 3])) > max_tcp_radius + 1e-6:
            checked.append({"candidate": name, "pregrasp_ik": False,
                            "grasp_ik": False, "reason": "GEOMETRIC_REACH_BOUND",
                            "max_tcp_radius_m": max_tcp_radius})
            continue
        common = dict(joint_margin_rad=0.05, random_starts=random_starts,
                      position_tolerance_m=0.002, rotation_tolerance_deg=2.0)
        pre = solve_pose_ik(kinematics, pre_tcp, seed, **common)
        target = solve_pose_ik(kinematics, grasp_tcp,
                               pre.joints if pre.success else seed, **common)
        record = {"candidate": name, "pregrasp_ik": pre.success,
                  "grasp_ik": target.success,
                  "pregrasp_error_mm": pre.position_error_m * 1000,
                  "grasp_error_mm": target.position_error_m * 1000,
                  "grasp_error_deg": target.rotation_error_deg}
        checked.append(record)
        if pre.success and target.success:
            scale = np.maximum(kinematics.upper_limits - kinematics.lower_limits, 0.01)
            cost = float(np.linalg.norm((pre.joints - seed) / scale) +
                         np.linalg.norm((target.joints - pre.joints) / scale))
            feasible.append(GraspCandidate(name, grasp, pre_tcp, grasp_tcp,
                                            pre.joints, target.joints, cost))
    feasible.sort(key=lambda candidate: candidate.score)
    return (feasible if return_all else feasible[0] if feasible else None), checked


def floor_clearance(plan, kinematics, geometry, world_arm, floor_z, margin_m,
                    gripper_width_m):
    """Check every commanded sample, including both jaws and End_link."""
    jaw_points = geometry.jaw_collision_points(gripper_width_m, include_max_open=True)
    minimum = math.inf
    minimum_sample = None
    index = 0
    for stage, samples in plan.items():
        for q in samples:
            tcp_world = world_arm @ kinematics.forward(q)
            points = tcp_world[:3, :3] @ jaw_points.T + tcp_world[:3, 3:4]
            end_z = float(tcp_world[2, 3])
            jaw_z = float(np.min(points[2]))
            component = "End_link" if end_z <= jaw_z else "jaw"
            height = min(end_z, jaw_z)
            if height < minimum:
                minimum, minimum_sample = height, {"stage": stage, "index": index,
                                                   "component": component}
            if height < floor_z + margin_m:
                raise RuntimeError("FLOOR_CLEARANCE_REJECTED stage=%s sample=%d component=%s min_z=%.4f required=%.4f" %
                                   (stage, index, component, height, floor_z + margin_m))
            index += 1
    return {"minimum_world_z_m": minimum, "minimum_sample": minimum_sample,
            "floor_z_m": floor_z, "required_z_m": floor_z + margin_m}
