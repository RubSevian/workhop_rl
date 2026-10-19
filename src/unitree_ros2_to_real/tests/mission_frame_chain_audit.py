#!/usr/bin/env python3
"""Offline frame algebra audit. No ROS initialization, camera, SDK or commands.

Derives a cloud-consistent candidate from current source/config/URDF. This is
not a physical extrinsic calibration and never changes the mission profile.
"""
import ast
import hashlib
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT/'repos/workhop_rl/src/unitree_ros2_to_real/scripts'))
from mission.transforms import fixed_joint, rigid, xyzrpy, EyeInHand


def main():
    nav_path = ROOT/'repos/autonomy_nav_go2/src/utilities/go2_navigation_bridge/config/real_navigation.yaml'
    urdf = ROOT/'repos/rars01_description/urdf/go2_arm_dynamic_base.urdf'
    relay_path = ROOT/'repos/autonomy_nav_go2/src/utilities/transform_sensors/transform_sensors/transform_everything.py'
    nav = yaml.safe_load(nav_path.read_text())
    sensor, lio = nav['sensors'], nav['nodes']['laserMapping']
    assert lio['mapping.extrinsic_est_en'] is False, 'Dynamic extrinsics require a different audit'
    assert lio['preprocess.lidar_type'] == 5
    # Execute only the two pure quaternion helpers, never the ROS module/Node.
    tree = ast.parse(relay_path.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('quaternion_from_euler', 'quaternion_matrix')]
    assert len(functions) == 2
    namespace = {'np': np}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(relay_path), 'exec'), namespace)
    processed_raw = np.eye(4)
    processed_raw[:3, :3] = namespace['quaternion_matrix'](
        namespace['quaternion_from_euler'](0, sensor['cloud_pitch_rad'], 0))
    processed_raw[2, 3] = -sensor['cam_offset_m']
    state_processed = np.eye(4)
    state_processed[:3, :3] = np.asarray(lio['mapping.extrinsic_R']).reshape(3, 3)
    state_processed[:3, 3] = lio['mapping.extrinsic_T']
    state_vehicle = np.eye(4)
    state_vehicle[:2, 3] = [-nav['robot']['sensor_offset_x_m'], -nav['robot']['sensor_offset_y_m']]
    body_raw = fixed_joint(urdf, 'radar_joint')
    body_mount = fixed_joint(urdf, 'go2_to_rars01_mount')
    mount_arm = fixed_joint(urdf, 'arm_mount_to_base_link')
    vehicle_body = rigid(np.linalg.inv(state_vehicle) @ state_processed @ processed_raw @ np.linalg.inv(body_raw))
    vehicle_mount = rigid(vehicle_body @ body_mount)
    vehicle_arm = rigid(vehicle_mount @ mount_arm)

    rng = np.random.default_rng(17002026)
    raw = np.c_[rng.uniform(-2, 2, (128, 3)), np.ones(128)].T
    # Coordinate route A uses URDF directly. Route B uses the actual relay
    # quaternion helpers, Point-LIO extrinsics and the NAV static offset.
    body_points = body_raw @ raw
    vehicle_points = np.linalg.inv(state_vehicle) @ state_processed @ processed_raw @ raw
    residual = float(np.max(np.abs(np.linalg.inv(vehicle_body) @ vehicle_points - body_points)))
    assert residual < 1e-12
    naive_error = float(np.max(np.linalg.norm(vehicle_points[:3] - body_points[:3], axis=0)))
    world_residual = 0.
    for _ in range(32):
        world_state = xyzrpy(dict(xyz_m=rng.uniform(-3, 3, 3), rpy_rad=rng.uniform(-1, 1, 3)))
        world_body = world_state @ state_vehicle @ vehicle_body
        route_a = world_state @ state_processed @ processed_raw @ raw
        route_b = world_body @ body_points
        world_residual = max(world_residual, float(np.max(np.abs(route_a-route_b))))
        world_arm = world_state @ state_vehicle @ vehicle_arm
        local_grasp = xyzrpy(dict(xyz_m=rng.uniform(-.5, .5, 3), rpy_rad=rng.uniform(-1, 1, 3)))
        camera = xyzrpy(dict(xyz_m=[.1, -.03, .05], rpy_rad=[.1, -.2, .3]))
        anchored = EyeInHand.anchor(camera, world_arm, local_grasp)
        np.testing.assert_allclose(EyeInHand.rebase(world_arm, anchored), camera @ local_grasp, atol=1e-12)
    assert world_residual < 1e-12

    source_paths = [nav_path, urdf, relay_path,
        ROOT/'repos/autonomy_nav_go2/src/slam/point_lio_unilidar/src/Estimator.cpp',
        ROOT/'repos/autonomy_nav_go2/src/slam/point_lio_unilidar/src/laserMapping.cpp',
        ROOT/'repos/autonomy_nav_go2/src/utilities/go2_navigation_bridge/launch/real_navigation.launch.py',
        ROOT/'repos/workhop_rl/src/unitree_ros2_to_real/scripts/grasp_mission.py',
        ROOT/'repos/workhop_rl/src/unitree_ros2_to_real/scripts/grasp_perception.py',
        ROOT/'repos/workhop_rl/src/unitree_ros2_to_real/scripts/mission/planning.py']
    lines = ['# NAV → arm: offline frame-chain evidence', '',
        '17.10.2026. Source/URDF consistency only; no physical calibration or actuator commands.', '',
        '`T_A_B` maps coordinates from B to A: `p_A = T_A_B @ p_B`.',
        '`I` = Point-LIO state/aft_mapped; `P` = processed cloud named body; `L` = raw radar.', '',
        '`T_V_B = inverse(T_I_V) @ T_I_P @ T_P_L @ inverse(T_B_L)`.',
        '`T_V_mount = T_V_B @ T_B_mount`.', '',
        '| Transform | XYZ m | RPY rad |', '|---|---|---|']
    for name, t in [('body→raw radar', body_raw), ('processed→raw radar', processed_raw),
                    ('state→processed', state_processed), ('state→vehicle', state_vehicle),
                    ('vehicle→body (candidate)', vehicle_body), ('body→mount', body_mount),
                    ('vehicle→mount (candidate)', vehicle_mount), ('vehicle→arm base_link (candidate)', vehicle_arm)]:
        lines.append(f'| {name} | `{t[:3,3].tolist()}` | `{Rotation.from_matrix(t[:3,:3]).as_euler("xyz").tolist()}` |')
    candidate = dict(vehicle_to_mount=dict(xyz_m=vehicle_mount[:3, 3].tolist(),
        rpy_rad=Rotation.from_matrix(vehicle_mount[:3, :3]).as_euler('xyz').tolist()))
    lines += ['', 'Candidate YAML (not installed or physically validated):', '', '```yaml',
        yaml.safe_dump(candidate, sort_keys=False).strip(), '```', '',
        f'128 raw-point routes: max component residual {residual:.3g} m.',
        f'32 world poses × 128 points: max component residual {world_residual:.3g} m.',
        '32 eye-in-hand anchor/rebase roundtrips: PASS at atol 1e-12.',
        f'Incorrect vehicle=body assumption: max marker error {naive_error*1000:.6f} mm.', '',
        'LocalPlanner/pathFollower use yaw-only sensor offsets; TF/FAR use full SO(3):', '',
        '| Example pitch | Full-TF vs yaw-only XYZ difference mm |', '|---|---:|']
    offset = state_vehicle[:3, 3]
    for degrees in (1, 5, 10):
        delta = Rotation.from_euler('y', degrees, degrees=True).apply(offset)-offset
        lines.append(f'| {degrees}° | {np.linalg.norm(delta)*1000:.6f} |')
    lines += ['', 'The derivation assumes raw-cloud coordinates correspond to URDF radar, and the fixed URDF mount describes the actual assembly. It aligns geometry with the current registered-cloud pipeline; it does not prove physical IMU placement, SLAM accuracy or calibration.', '',
        'URDF trunk imu is not identified with the lidar IMU used by this real NAV stack. The simulation-only 0.31 m residual correction must not be applied to real NAV on that assumption.', '',
        '## Source hashes', '']
    for path in source_paths:
        lines.append(f'- `{path.relative_to(ROOT)}`: `{hashlib.sha256(path.read_bytes()).hexdigest()}`')
    output = ROOT/'docs/test_statistics/2026_10_17/NAV_ARM_FRAME_CHAIN_EVIDENCE.md'
    output.write_text('\n'.join(lines)+'\n')
    print(output)
    print(yaml.safe_dump(candidate, sort_keys=False))
    print(f'PASS offline algebra: local residual={residual:.3g} m, world residual={world_residual:.3g} m')


if __name__ == '__main__':
    main()
