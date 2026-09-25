#!/usr/bin/env python3
"""Run a five-second headless Go2 stand + RARS01 home-hold smoke test."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np


LEGS = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
LEG_TARGETS = np.asarray(
    (-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
     -0.1, 0.8, -1.5, 0.1, 0.8, -1.5)
)
RARS01 = tuple(f"joint{index}" for index in range(1, 7)) + (
    "gripper_left_joint", "gripper_right_joint"
)
RARS01_ACTUATORS = tuple(f"joint{index}_motor" for index in range(1, 7)) + (
    "gripper_left_motor", "gripper_right_motor"
)
RARS01_KP = np.asarray((20, 20, 20, 6, 6, 6, 20, 20), dtype=np.float64)
RARS01_KD = np.asarray((1, 1, 1, 0.4, 0.4, 0.4, 0.2, 0.2), dtype=np.float64)


def ids(model: mujoco.MjModel, object_type: mujoco.mjtObj, names: tuple[str, ...]) -> list[int]:
    result = [mujoco.mj_name2id(model, object_type, name) for name in names]
    if any(value < 0 for value in result):
        missing = [name for name, value in zip(names, result) if value < 0]
        raise AssertionError(f"missing named MuJoCo objects: {missing}")
    return result


def main() -> int:
    default_scene = (
        Path(__file__).resolve().parents[1]
        / "unitree_robots" / "go2_rars01" / "scene.xml"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=default_scene)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--max-arm-error", type=float, default=0.7)
    parser.add_argument("--soft-limit-tolerance", type=float, default=0.15)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    if abs(model.opt.timestep - 0.005) > 1.0e-12:
        raise AssertionError(f"unexpected timestep {model.opt.timestep}")
    mujoco.mj_resetDataKeyframe(model, data, 0)

    leg_joints = ids(model, mujoco.mjtObj.mjOBJ_JOINT, LEGS)
    leg_actuators = ids(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR,
        tuple(name.removesuffix("_joint") for name in LEGS),
    )
    rars_joints = ids(model, mujoco.mjtObj.mjOBJ_JOINT, RARS01)
    rars_actuators = ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, RARS01_ACTUATORS)
    if leg_actuators != list(range(12)) or rars_actuators != list(range(12, 20)):
        raise AssertionError("12/6/2 actuator ownership/order mismatch")

    samples = int(round(args.seconds / model.opt.timestep))
    base_heights: list[float] = []
    contacts: list[int] = []
    max_qvel = 0.0
    max_arm_error = 0.0
    max_arm_velocity = 0.0
    max_arm_torque = 0.0
    limit_violation = 0.0

    for _ in range(samples):
        for target, joint_id, actuator_id in zip(LEG_TARGETS, leg_joints, leg_actuators):
            q = data.qpos[model.jnt_qposadr[joint_id]]
            velocity = data.qvel[model.jnt_dofadr[joint_id]]
            torque = 25.0 * (target - q) - velocity
            lower, upper = model.actuator_ctrlrange[actuator_id]
            data.ctrl[actuator_id] = np.clip(torque, lower, upper)
        for index, (joint_id, actuator_id) in enumerate(zip(rars_joints, rars_actuators)):
            q = data.qpos[model.jnt_qposadr[joint_id]]
            velocity = data.qvel[model.jnt_dofadr[joint_id]]
            torque = -RARS01_KP[index] * q - RARS01_KD[index] * velocity
            lower, upper = model.actuator_ctrlrange[actuator_id]
            data.ctrl[actuator_id] = np.clip(torque, lower, upper)

        mujoco.mj_step(model, data)
        if not (
            np.isfinite(data.qpos).all()
            and np.isfinite(data.qvel).all()
            and np.isfinite(data.ctrl).all()
        ):
            raise AssertionError("NaN/Inf encountered during home hold")
        base_heights.append(float(data.qpos[2]))
        contacts.append(int(data.ncon))
        max_qvel = max(max_qvel, float(np.max(np.abs(data.qvel))))
        arm_q = np.asarray([data.qpos[model.jnt_qposadr[j]] for j in rars_joints[:6]])
        arm_v = np.asarray([data.qvel[model.jnt_dofadr[j]] for j in rars_joints[:6]])
        max_arm_error = max(max_arm_error, float(np.max(np.abs(arm_q))))
        max_arm_velocity = max(max_arm_velocity, float(np.max(np.abs(arm_v))))
        max_arm_torque = max(max_arm_torque, float(np.max(np.abs(data.ctrl[12:18]))))
        for joint_id in leg_joints + rars_joints:
            if not model.jnt_limited[joint_id]:
                continue
            q = data.qpos[model.jnt_qposadr[joint_id]]
            lower, upper = model.jnt_range[joint_id]
            limit_violation = max(limit_violation, lower - q, q - upper)

    if max_qvel >= 100.0:
        raise AssertionError(f"velocity explosion: max |qvel|={max_qvel}")
    # Joint4 has kp=6 and intentionally no gravity feed-forward, matching the
    # training controller.  Its bounded static PD sag is reported, not hidden.
    if max_arm_error >= args.max_arm_error:
        raise AssertionError(f"arm left home neighborhood: error={max_arm_error}")
    if limit_violation > args.soft_limit_tolerance:
        raise AssertionError(f"joint limit violation={limit_violation}")

    print(
        "HOME HOLD: PASS\n"
        f"duration_s={samples * model.opt.timestep:.3f}\n"
        f"base_height_min={min(base_heights):.6f}\n"
        f"base_height_max={max(base_heights):.6f}\n"
        f"max_abs_qvel={max_qvel:.6f}\n"
        f"max_arm_position_error={max_arm_error:.6f}\n"
        f"max_arm_velocity={max_arm_velocity:.6f}\n"
        f"max_arm_torque={max_arm_torque:.6f}\n"
        f"max_soft_limit_violation={limit_violation:.6f}\n"
        f"contacts_min={min(contacts)} contacts_max={max(contacts)} "
        f"contacts_mean={np.mean(contacts):.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
