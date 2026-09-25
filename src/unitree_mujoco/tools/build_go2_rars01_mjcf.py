#!/usr/bin/env python3
"""Reproducibly build the training-parity Go2+RARS01 MuJoCo model."""

from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing
import re
import subprocess
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco

from convert_urdf_meshes_to_obj import convert_urdf


LEG_DDS = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
ARM = tuple(f"joint{index}" for index in range(1, 7))
GRIPPER = ("gripper_left_joint", "gripper_right_joint")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_value(repository: Path, *arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repository), *arguments], text=True
    ).strip()


def joint_map(root: ET.Element) -> dict[str, ET.Element]:
    return {joint.get("name", ""): joint for joint in root.findall("joint")}


def joint_effort(joint: ET.Element) -> float:
    limit = joint.find("limit")
    if limit is None or limit.get("effort") is None:
        raise ValueError(f"joint {joint.get('name')} has no effort limit")
    return float(limit.get("effort", "nan"))


def generate_compile_urdf(train_obj: Path, output: Path) -> dict[str, ET.Element]:
    tree = ET.parse(train_obj)
    root = tree.getroot()
    joints = joint_map(root)
    for name in ARM + GRIPPER:
        joint = joints.get(name)
        if joint is None or joint.get("type") not in ("revolute", "prismatic"):
            raise ValueError(f"missing movable RARS01 joint: {name}")

    # Strip only visual elements whose mesh is a Go2 DAE.  Collision geometry,
    # inertials, frames, joints, and the RARS01 OBJ visuals remain untouched.
    parent_by_child = {child: parent for parent in root.iter() for child in parent}
    removed = 0
    for visual in list(root.findall(".//visual")):
        mesh = visual.find("./geometry/mesh")
        if mesh is None or not mesh.get("filename", "").lower().endswith(".dae"):
            continue
        parent_by_child[visual].remove(visual)
        removed += 1
    if removed != 17:
        raise ValueError(f"expected to strip 17 Go2 DAE visuals, stripped {removed}")

    for existing in list(root.findall("mujoco")):
        root.remove(existing)
    mujoco_extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        mujoco_extension,
        "compiler",
        {
            "fusestatic": "false",
            "discardvisual": "false",
            # MuJoCo's URDF importer keeps only mesh basenames; meshdir makes
            # that behavior explicit while remaining relative/portable.
            "meshdir": "../assets/rars01",
        },
    )
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return joints


def add_stock_go2_visuals(root: ET.Element, stock_path: Path) -> None:
    stock_root = ET.parse(stock_path).getroot()
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    stock_asset = stock_root.find("asset")
    if stock_asset is None:
        raise ValueError(f"stock Go2 model has no asset section: {stock_path}")
    for node in stock_asset:
        if node.tag not in ("material", "mesh"):
            continue
        copy = ET.fromstring(ET.tostring(node, encoding="unicode"))
        if copy.tag == "mesh":
            copy.set("file", "../go2/assets/" + copy.get("file", ""))
        asset.append(copy)

    generated_bodies = {
        body.get("name", ""): body for body in root.findall(".//body") if body.get("name")
    }
    # Go2's physical root is `base` in the combined URDF; `base_link` belongs
    # to RARS01 and must never receive the stock quadruped shell.
    body_target = {"base_link": "base"}
    for stock_body in stock_root.findall(".//body"):
        target = generated_bodies.get(
            body_target.get(stock_body.get("name", ""), stock_body.get("name", ""))
        )
        if target is None:
            continue
        for geom in stock_body.findall("geom"):
            if not geom.get("mesh"):
                continue
            visual = ET.fromstring(ET.tostring(geom, encoding="unicode"))
            visual.attrib.pop("class", None)
            visual.set("type", "mesh")
            visual.set("contype", "0")
            visual.set("conaffinity", "0")
            visual.set("group", "2")
            target.append(visual)


def _actuator_name(joint_name: str) -> str:
    if joint_name in LEG_DDS:
        return joint_name[: -len("_joint")]
    if joint_name in GRIPPER:
        return joint_name[: -len("_joint")] + "_motor"
    return f"{joint_name}_motor"


def add_runtime_layers(
    raw_xml: Path, final_xml: Path, joints: dict[str, ET.Element], stock_go2: Path
) -> None:
    tree = ET.parse(raw_xml)
    root = tree.getroot()
    root.set("model", "go2_rars01")
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
    asset = root.find("asset")
    if asset is None:
        raise ValueError("compiled URDF has no mesh asset section")
    for mesh in asset.findall("mesh"):
        if mesh.get("name") in {
            "arm_mount_link", "base_link", "link1_m0", "link2_m1", "link2_m2",
            "link_3_m3", "link_4_m4", "link_5_m5", "End_link",
            "gripper_left", "gripper_right",
        }:
            mesh.set("file", "assets/rars01/" + Path(mesh.get("file", "")).name)

    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        insert_at = 1 if root.find("compiler") is not None else 0
        root.insert(insert_at, option)
    option.set("timestep", "0.005")
    option.set("cone", "elliptic")
    option.set("impratio", "100")

    add_stock_go2_visuals(root, stock_go2)
    base = next((b for b in root.findall(".//body") if b.get("name") == "base"), None)
    if base is None:
        raise ValueError("compiled URDF has no Go2 base body named 'base'")
    # Keep physical collision primitives separately toggleable in the viewer.
    # Visual meshes set group=1 (RARS01) or group=2 (Go2) explicitly; inherited
    # group=3 therefore applies only to robot collision geometry.  Scene floor
    # and environment remain in group=0.
    main_default = root.find("default")
    if main_default is None:
        main_default = ET.Element("default")
        asset_index = list(root).index(asset)
        root.insert(asset_index, main_default)
    collision_default = next(
        (item for item in main_default.findall("default") if item.get("class") == "robot_collision"),
        None,
    )
    if collision_default is None:
        collision_default = ET.SubElement(main_default, "default", {"class": "robot_collision"})
        ET.SubElement(collision_default, "geom", {"group": "3"})
    base.set("childclass", "robot_collision")
    base.insert(0, ET.Element("freejoint", {"name": "base_freejoint"}))
    ET.SubElement(base, "site", {"name": "imu", "pos": "-0.02557 0 0.04232"})

    for section_name in ("actuator", "sensor", "keyframe"):
        existing = root.find(section_name)
        if existing is not None:
            root.remove(existing)

    actuator = ET.SubElement(root, "actuator")
    for name in LEG_DDS + ARM + GRIPPER:
        limit = joint_effort(joints[name])
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": _actuator_name(name),
                "joint": name,
                "ctrlrange": f"{-limit:.12g} {limit:.12g}",
            },
        )

    sensor = ET.SubElement(root, "sensor")
    # The first 36 scalar sensors are a compatibility ABI for the Unitree
    # bridge.  IMU/frame offsets immediately follow and are never displaced by
    # RARS01 measurements.
    for suffix, tag in (
        ("pos", "jointpos"), ("vel", "jointvel"), ("torque", "jointactuatorfrc")
    ):
        for name in LEG_DDS:
            ET.SubElement(
                sensor, tag,
                {"name": f"{_actuator_name(name)}_{suffix}", "joint": name},
            )
    ET.SubElement(sensor, "framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu"})
    ET.SubElement(sensor, "accelerometer", {"name": "imu_acc", "site": "imu"})
    ET.SubElement(sensor, "framepos", {"name": "frame_pos", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "framelinvel", {"name": "frame_vel", "objtype": "site", "objname": "imu"})
    for suffix, tag in (
        ("pos", "jointpos"), ("vel", "jointvel"), ("torque", "jointactuatorfrc")
    ):
        for name in ARM + GRIPPER:
            ET.SubElement(sensor, tag, {"name": f"{name}_{suffix}", "joint": name})

    # URDF order is FL, FR, RL, RR.  This keyframe is qpos order, independent
    # of the DDS actuator order above.
    # At the policy-default leg angles the four feet touch the plane at
    # base_z=0.323502569... .  Keep a 2.6 um solver-level overlap so reset
    # begins on the floor rather than falling from the former 0.4 m pose.
    # The normal stand-up controller remains responsible for raising it.
    home_qpos = [
        0, 0, 0.3235, 1, 0, 0, 0,
        0.1, 0.8, -1.5,
        -0.1, 0.8, -1.5,
        0.1, 0.8, -1.5,
        -0.1, 0.8, -1.5,
        *([0] * 8),
    ]
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(
        keyframe,
        "key",
        {
            "name": "home",
            "qpos": " ".join(str(value) for value in home_qpos),
            "ctrl": " ".join("0" for _ in range(20)),
        },
    )
    ET.indent(tree, space="  ")
    tree.write(final_xml, encoding="utf-8", xml_declaration=True)


def write_scene(output_root: Path, stock_scene: Path) -> None:
    text = stock_scene.read_text(encoding="utf-8")
    text = text.replace('file="go2.xml"', 'file="go2_rars01.xml"')
    # Robot collision geoms use group 3.  Keep the visible floor explicitly in
    # group 0 so hiding group 3 never removes the ground from the viewer.
    text = text.replace('<geom name="floor" ', '<geom name="floor" group="0" ')
    # The checked-in heightfield images are optional stock demo assets; the
    # Phase-3 smoke scene uses the plane while keeping lighting and skybox.
    text = re.sub(r"\s*<hfield[^>]*/>", "", text)
    text = re.sub(r"\s*<geom type=\"hfield\"[^>]*/>", "", text)
    (output_root / "scene.xml").write_text(text, encoding="utf-8")


def write_manifest(
    output_root: Path,
    description_root: Path,
    workhop_root: Path,
    train_urdf: Path,
    base_urdf: Path,
    stock_go2: Path,
    final_xml: Path,
) -> None:
    text = f'''model:
  name: go2_rars01

sources:
  rars01_description:
    repository: "https://github.com/RubSevian/rars01_description.git"
    branch: "{git_value(description_root, "branch", "--show-current")}"
    commit: "{git_value(description_root, "rev-parse", "HEAD")}"
    train_urdf: "urdf/go2_arm_dynamic_train.urdf"
    base_urdf: "urdf/go2_arm_dynamic_base.urdf"
    train_urdf_sha256: "{sha256(train_urdf)}"
    base_urdf_sha256: "{sha256(base_urdf)}"

  stock_go2_visuals:
    repository: "https://github.com/RubSevian/workhop_rl.git"
    branch: "{git_value(workhop_root, "branch", "--show-current")}"
    source_model: "src/unitree_mujoco/unitree_robots/go2/go2.xml"
    source_model_sha256: "{sha256(stock_go2)}"

mesh_pipeline:
  rars01_source_format: "STL"
  rars01_runtime_format: "OBJ"
  decimation: false
  conversion_report: "MESH_CONVERSION_REPORT.json"

runtime:
  profile: "train_obj"
  timestep: 0.005
  leg_actuators: 12
  arm_actuators: 6
  gripper_actuators: 2

generated:
  mjcf: "go2_rars01.xml"
  mjcf_sha256: "{sha256(final_xml)}"
'''
    (output_root / "SOURCE_MANIFEST.yaml").write_text(text, encoding="utf-8")


def run_conversion_isolated(
    urdf: Path, description_root: Path, output_root: Path, profile: str
) -> None:
    """Run peak-memory mesh indexing in a disposable process on POSIX."""
    if "fork" not in multiprocessing.get_all_start_methods():
        convert_urdf(urdf, description_root, output_root, profile)
        return
    process = multiprocessing.get_context("fork").Process(
        target=convert_urdf,
        args=(urdf, description_root, output_root, profile),
    )
    process.start()
    process.join()
    if process.exitcode != 0:
        raise RuntimeError(f"{profile} converter exited with status {process.exitcode}")


def compile_urdf_to_mjcf(urdf: Path, output: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(urdf))
    mujoco.mj_saveLastXML(str(output), model)


def compile_urdf_isolated(urdf: Path, output: Path) -> None:
    if "fork" not in multiprocessing.get_all_start_methods():
        compile_urdf_to_mjcf(urdf, output)
        return
    process = multiprocessing.get_context("fork").Process(
        target=compile_urdf_to_mjcf, args=(urdf, output)
    )
    process.start()
    process.join()
    if process.exitcode != 0:
        raise RuntimeError(f"MuJoCo URDF compiler exited with status {process.exitcode}")


def build(description_root: Path, output_root: Path) -> mujoco.MjModel:
    description_root = description_root.resolve()
    output_root = output_root.resolve()
    workhop_root = Path(__file__).resolve().parents[3]
    robot_root = Path(__file__).resolve().parents[1] / "unitree_robots"
    train_urdf = description_root / "urdf" / "go2_arm_dynamic_train.urdf"
    base_urdf = description_root / "urdf" / "go2_arm_dynamic_base.urdf"
    stock_go2 = robot_root / "go2" / "go2.xml"
    output_root.mkdir(parents=True, exist_ok=True)

    run_conversion_isolated(train_urdf, description_root, output_root, "train_obj")
    run_conversion_isolated(base_urdf, description_root, output_root, "base_obj")
    train_derivative = output_root / "generated" / "go2_arm_dynamic_train_obj.urdf"
    base_derivative = output_root / "generated" / "go2_arm_dynamic_base_obj.urdf"
    report_path = output_root / "MESH_CONVERSION_REPORT.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["profiles"] = ["train_obj", "base_obj"]
    report["derivative_urdfs"] = [
        train_derivative.relative_to(output_root).as_posix(),
        base_derivative.relative_to(output_root).as_posix(),
    ]
    report.pop("profile", None)
    report.pop("derivative_urdf", None)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    compile_urdf = output_root / "generated" / "go2_arm_dynamic_train_mujoco.urdf"
    joints = generate_compile_urdf(train_derivative, compile_urdf)
    with tempfile.TemporaryDirectory(prefix="go2_rars01_") as temporary_string:
        raw_xml = Path(temporary_string) / "compiled.xml"
        compile_urdf_isolated(compile_urdf, raw_xml)
        final_xml = output_root / "go2_rars01.xml"
        add_runtime_layers(raw_xml, final_xml, joints, stock_go2)

    final_model = mujoco.MjModel.from_xml_path(str(final_xml))
    write_scene(output_root, robot_root / "go2" / "scene_terrain.xml")
    # `final_model` already validates every robot asset.  Loading scene.xml at
    # the same time would hold two >1 GB high-poly model instances; the model
    # validator loads the scene separately after this process exits.
    write_manifest(
        output_root, description_root, workhop_root, train_urdf, base_urdf, stock_go2, final_xml
    )
    return final_model


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    model = build(args.description_root, args.output_root)
    print(
        "generated go2_rars01.xml "
        f"(nbody={model.nbody}, njnt={model.njnt}, nq={model.nq}, "
        f"nv={model.nv}, nu={model.nu}, nsensor={model.nsensor})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
