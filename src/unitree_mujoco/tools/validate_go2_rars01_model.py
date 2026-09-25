#!/usr/bin/env python3
"""Validate mesh conversion and training-URDF dynamics independently."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np


LEG_DDS = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
ARM = tuple(f"joint{index}" for index in range(1, 7))
GRIPPER = ("gripper_left_joint", "gripper_right_joint")
MOVABLE = LEG_DDS + ARM + GRIPPER


def vector(text: str | None, default: tuple[float, ...]) -> np.ndarray:
    return np.asarray(default if text is None else [float(value) for value in text.split()])


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)))
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)))
    return rz @ ry @ rx


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    output = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(output, np.asarray(quaternion, dtype=np.float64))
    return output.reshape(3, 3)


def require_close(label: str, actual: np.ndarray | float, expected: np.ndarray | float,
                  atol: float = 1.0e-8, rtol: float = 5.0e-6) -> None:
    if not np.allclose(actual, expected, atol=atol, rtol=rtol):
        raise AssertionError(f"{label}: actual={actual!r}, expected={expected!r}")


def actuator_name(joint_name: str) -> str:
    if joint_name in LEG_DDS:
        return joint_name.removesuffix("_joint")
    if joint_name in GRIPPER:
        return joint_name.removesuffix("_joint") + "_motor"
    return f"{joint_name}_motor"


def validate_meshes(model_root: Path) -> dict[str, int]:
    report_path = model_root / "MESH_CONVERSION_REPORT.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    records = report["records"]
    if report.get("status") != "PASS" or len(records) != 11:
        raise AssertionError("conversion report must contain 11 passing unique meshes")
    for record in records:
        if record["status"] != "PASS" or record["source_faces"] != record["output_faces"]:
            raise AssertionError(f"mesh parity failed: {record['source']}")
        if record["relative_area_error"] > report["relative_area_tolerance"]:
            raise AssertionError(f"area parity failed: {record['source']}")
    mount = next(record for record in records if Path(record["source"]).stem == "arm_mount_link")
    if mount["source_faces"] <= 200_000:
        raise AssertionError("arm_mount_link regression mesh is no longer above the STL limit")

    runtime_files = [model_root / "go2_rars01.xml", *sorted((model_root / "generated").glob("*.urdf"))]
    for path in runtime_files:
        for mesh in ET.parse(path).getroot().findall(".//mesh"):
            reference = mesh.get("filename", mesh.get("file", ""))
            if reference.lower().endswith(".stl"):
                raise AssertionError(f"runtime STL reference in {path}: {reference}")
            if reference.startswith("/") or "/home/" in reference:
                raise AssertionError(f"absolute runtime path in {path}: {reference}")
    stl_assets = list((model_root / "assets" / "rars01").glob("*.[sS][tT][lL]"))
    if stl_assets:
        raise AssertionError(f"generated RARS01 asset directory still contains STL: {stl_assets}")
    return {"unique_meshes": len(records), "mount_faces": int(mount["source_faces"])}


def source_inertia(inertial: ET.Element) -> tuple[float, np.ndarray, np.ndarray]:
    origin = inertial.find("origin")
    xyz = vector(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
    rpy = vector(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
    mass = float(inertial.find("mass").get("value"))
    values = inertial.find("inertia")
    matrix = np.asarray((
        (float(values.get("ixx")), float(values.get("ixy")), float(values.get("ixz"))),
        (float(values.get("ixy")), float(values.get("iyy")), float(values.get("iyz"))),
        (float(values.get("ixz")), float(values.get("iyz")), float(values.get("izz"))),
    ))
    rotation = rpy_matrix(rpy)
    return mass, xyz, rotation @ matrix @ rotation.T


def validate_dynamics(urdf_path: Path, scene_path: Path) -> mujoco.MjModel:
    urdf_root = ET.parse(urdf_path).getroot()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    if abs(model.opt.timestep - 0.005) > 1.0e-12:
        raise AssertionError(f"timestep is {model.opt.timestep}, expected 0.005")
    expected_dimensions = {"nq": 27, "nv": 26, "nu": 20}
    for field, expected in expected_dimensions.items():
        if getattr(model, field) != expected:
            raise AssertionError(f"{field}={getattr(model, field)}, expected {expected}")

    # Inertia is compared as a reconstructed full tensor in the link frame;
    # MuJoCo is free to diagonalize it and rotate the inertial frame.
    for link in urdf_root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        name = link.get("name")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise AssertionError(f"missing body for inertial link {name}")
        mass, com, tensor = source_inertia(inertial)
        rotation = quaternion_matrix(model.body_iquat[body_id])
        reconstructed = rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T
        require_close(f"{name} mass", model.body_mass[body_id], mass)
        require_close(f"{name} COM", model.body_ipos[body_id], com)
        require_close(f"{name} inertia", reconstructed, tensor, atol=2.0e-8, rtol=5.0e-5)

    for joint in urdf_root.findall("joint"):
        name = joint.get("name")
        child = joint.find("child").get("link")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, child)
        if body_id < 0:
            raise AssertionError(f"missing child body {child} for joint {name}")
        origin = joint.find("origin")
        xyz = vector(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
        rpy = vector(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
        require_close(f"{name} origin xyz", model.body_pos[body_id], xyz)
        require_close(
            f"{name} origin rotation",
            quaternion_matrix(model.body_quat[body_id]),
            rpy_matrix(rpy),
        )
        if joint.get("type") == "fixed":
            continue
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise AssertionError(f"missing movable joint {name}")
        axis = vector(joint.find("axis").get("xyz"), (1.0, 0.0, 0.0))
        require_close(f"{name} axis", model.jnt_axis[joint_id], axis)
        limit = joint.find("limit")
        expected_range = np.asarray((float(limit.get("lower")), float(limit.get("upper"))))
        require_close(f"{name} range", model.jnt_range[joint_id], expected_range)

    for index, joint_name in enumerate(MOVABLE):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        expected_type = (
            mujoco.mjtJoint.mjJNT_SLIDE if joint_name in GRIPPER else mujoco.mjtJoint.mjJNT_HINGE
        )
        if joint_id < 0 or model.jnt_type[joint_id] != expected_type:
            raise AssertionError(f"wrong or missing joint {joint_name}")
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name(joint_name)
        )
        if actuator_id != index:
            raise AssertionError(
                f"actuator order mismatch for {joint_name}: {actuator_id}, expected {index}"
            )

    # Sensor compatibility ABI: leg q/qdot/force, then legacy IMU/frame.
    expected_legacy = [
        *(f"{actuator_name(name)}_pos" for name in LEG_DDS),
        *(f"{actuator_name(name)}_vel" for name in LEG_DDS),
        *(f"{actuator_name(name)}_torque" for name in LEG_DDS),
        "imu_quat", "imu_gyro", "imu_acc", "frame_pos", "frame_vel",
    ]
    for sensor_id, expected_name in enumerate(expected_legacy):
        actual_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id)
        if actual_name != expected_name:
            raise AssertionError(
                f"legacy sensor order mismatch at {sensor_id}: {actual_name}, expected {expected_name}"
            )
    return model


def main() -> int:
    tool_dir = Path(__file__).resolve().parent
    workhop_root = tool_dir.parents[2]
    default_description = workhop_root.parent / "rars01_description"
    default_model = tool_dir.parent / "unitree_robots" / "go2_rars01"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description-root", type=Path, default=default_description)
    parser.add_argument("--model-root", type=Path, default=default_model)
    args = parser.parse_args()

    mesh = validate_meshes(args.model_root)
    print(
        "MESH CONVERSION: PASS "
        f"({mesh['unique_meshes']} meshes, arm_mount_link={mesh['mount_faces']} faces)"
    )
    model = validate_dynamics(
        args.description_root / "urdf" / "go2_arm_dynamic_train.urdf",
        args.model_root / "scene.xml",
    )
    print(
        "DYNAMICS PARITY: PASS "
        f"(nbody={model.nbody}, njnt={model.njnt}, nq={model.nq}, nv={model.nv}, "
        f"nu={model.nu}, nsensor={model.nsensor}, timestep={model.opt.timestep})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
