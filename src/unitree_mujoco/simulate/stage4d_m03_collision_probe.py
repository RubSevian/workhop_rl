#!/usr/bin/env python3
"""Offline pinned-URDF triangle BVH versus MuJoCo mesh distance probe.

Read-only diagnostic: no ROS publishers, arm commands, serial, or model edits.
Run with repos/rars01_graspnet/.venv/bin/python from the workspace root.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

import coal
import mujoco
import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[5]
DESCRIPTION = ROOT / "repos/rars01_description"
SCENE = ROOT / "repos/workhop_rl/src/unitree_mujoco/unitree_robots/go2_rars01/scene_stage4d.xml"
EXPECTED_COMMIT = "2df2bf4f9a28437124b4eedf6e1777c2f497b709"
ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
GRIPPER_JOINTS = ("gripper_left_joint", "gripper_right_joint")
PAIRS = (
    ("link1_m0", "link2_m2"),
    ("link1_m0", "link_4_m4"),
    ("link2_m1", "link_4_m4"),
    ("link2_m1", "link_3_m3"),
    ("link2_m2", "link_4_m4"),
)


def models():
    head = subprocess.check_output(
        ["git", "-C", str(DESCRIPTION), "rev-parse", "HEAD"], text=True
    ).strip()
    if head != EXPECTED_COMMIT:
        raise RuntimeError(f"description HEAD {head} != pinned {EXPECTED_COMMIT}")
    urdf = DESCRIPTION / "urdf/go2_arm_dynamic_base.urdf"
    model = pin.buildModelFromUrdf(str(urdf), pin.JointModelFreeFlyer())
    geometry = pin.buildGeomFromUrdf(
        model, str(urdf), pin.GeometryType.COLLISION,
        package_dirs=[str(ROOT / "repos")],
    )
    mj_model = mujoco.MjModel.from_xml_path(str(SCENE))
    return model, geometry, mj_model


def pin_state(model, mj_model, arm):
    q = pin.neutral(model)
    q[:3] = [0.0, 0.0, 0.42]
    joint_names = list(ARM_JOINTS + GRIPPER_JOINTS)
    values = list(map(float, arm)) + [0.04, 0.04]
    for joint_id in range(1, model.njoints):
        name = model.names[joint_id]
        if name in joint_names:
            value = values[joint_names.index(name)]
        elif name == "root_joint":
            continue
        else:
            mj_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if mj_id < 0:
                raise RuntimeError(f"missing MuJoCo joint {name}")
            value = float(mj_model.qpos0[mj_model.jnt_qposadr[mj_id]])
        if model.nqs[joint_id] != 1:
            raise RuntimeError(f"unexpected joint dimension {name}")
        q[model.idx_qs[joint_id]] = value
    return q


def mujoco_state(mj_model, arm):
    data = mujoco.MjData(mj_model)
    data.qpos[:] = mj_model.qpos0
    free_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, "base_freejoint")
    free = mj_model.jnt_qposadr[free_id]
    data.qpos[free:free + 7] = [0, 0, 0.42, 1, 0, 0, 0]
    for name, value in zip(ARM_JOINTS + GRIPPER_JOINTS, list(arm) + [0.04, 0.04]):
        joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"missing MuJoCo joint {name}")
        data.qpos[mj_model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(mj_model, data)
    return data


def compare(model, geometry, mj_model, arm):
    data = model.createData()
    geometry_data = pin.GeometryData(geometry)
    q = pin_state(model, mj_model, arm)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.updateGeometryPlacements(model, data, geometry, geometry_data)
    mj_data = mujoco_state(mj_model, arm)
    by_frame = {model.frames[obj.parentFrame].name: i
                for i, obj in enumerate(geometry.geometryObjects)}
    by_body = {}
    for geom_id in range(mj_model.ngeom):
        body = mujoco.mj_id2name(
            mj_model, mujoco.mjtObj.mjOBJ_BODY, mj_model.geom_bodyid[geom_id]
        )
        if body not in by_body or mj_model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH:
            by_body[body] = geom_id
    rows = []
    for first, second in PAIRS:
        a, b = by_frame[first], by_frame[second]
        ga, gb = geometry.geometryObjects[a], geometry.geometryObjects[b]
        ta, tb = geometry_data.oMg[a], geometry_data.oMg[b]
        exact = coal.distance(
            ga.geometry, coal.Transform3s(ta.rotation, ta.translation),
            gb.geometry, coal.Transform3s(tb.rotation, tb.translation),
            coal.DistanceRequest(), coal.DistanceResult(),
        )
        old = mujoco.mj_geomDistance(
            mj_model, mj_data, by_body[first], by_body[second], 1.0, None
        )
        rows.append({"pair": [first, second], "coal_triangle_m": exact,
                     "mujoco_mesh_m": old, "difference_m": exact - old})
    return rows



def concavity_probe():
    """A sphere sits in an empty U-cavity, but inside its convex hull."""
    import struct

    vertices = coal.StdVec_Vec3s()
    triangles = coal.StdVec_Triangle()
    stl_vertices = []
    stl_faces = []
    boxes = (((-1, -1, -.1), (-.7, 1, .1)),
             ((.7, -1, -.1), (1, 1, .1)),
             ((-1, -1, -.1), (1, -.7, .1)))
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    for low, high in boxes:
        x0, y0, z0 = low
        x1, y1, z1 = high
        offset = len(stl_vertices)
        corners = ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                   (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))
        for corner in corners:
            vertices.append(np.asarray(corner, dtype=float))
            stl_vertices.append(corner)
        for a, b, c, d in faces:
            for i, j, k in ((a, b, c), (a, c, d)):
                triangle = (offset + i, offset + j, offset + k)
                triangles.append(coal.Triangle(*triangle))
                stl_faces.append(triangle)
    mesh = coal.BVHModelOBBRSS()
    mesh.beginModel(len(triangles), len(vertices))
    mesh.addSubModel(vertices, triangles)
    mesh.endModel()
    exact = coal.distance(mesh, coal.Transform3s(), coal.Sphere(.1),
                          coal.Transform3s(), coal.DistanceRequest(), coal.DistanceResult())
    binary_stl = bytearray(80) + struct.pack("<I", len(stl_faces))
    for face in stl_faces:
        binary_stl += struct.pack("<12fH", 0, 0, 0,
                                  *stl_vertices[face[0]], *stl_vertices[face[1]],
                                  *stl_vertices[face[2]], 0)
    xml = ('<mujoco><asset><mesh name="u" file="u.stl"/></asset><worldbody>'
           '<geom name="u" type="mesh" mesh="u"/>'
           '<geom name="ball" type="sphere" size="0.1" pos="0 0 0"/>'
           '</worldbody></mujoco>')
    model = mujoco.MjModel.from_xml_string(xml, assets={"u.stl": bytes(binary_stl)})
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    old = mujoco.mj_geomDistance(model, data, 0, 1, 2.0, None)
    return {"coal_triangle_m": exact, "mujoco_mesh_m": old}
def main():
    model, geometry, mj_model = models()
    configurations = {
        "HOME": [0.0] * 6,
        "READY_V1_APPROX": [-0.0011, 0.972, 1.3737, -1.1017, -0.0009, -0.0007],
    }
    result = {"description_commit": EXPECTED_COMMIT,
              "coal_version": coal.__version__, "mujoco_version": mujoco.__version__,
              "concavity_probe": concavity_probe(),
              "full_model_nq": model.nq, "collision_geoms": geometry.ngeoms,
              "states": {name: {"q": q, "pairs": compare(model, geometry, mj_model, q)}
                         for name, q in configurations.items()}}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
