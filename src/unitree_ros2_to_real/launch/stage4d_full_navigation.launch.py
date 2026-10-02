import math
import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _manual_grasp_settings():
    default_path = Path(__file__).resolve().parent.parent / 'config' / 'stage4d_manual_grasp.yaml'
    path = Path(os.environ.get('STAGE4D_MANUAL_GRASP_CONFIG', str(default_path)))
    settings = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(settings, dict):
        raise ValueError(f'Invalid Stage4D manual grasp config: {path}')
    names = ('target_diameter_m', 'virtual_grasp_width_m', 'floor_z_m',
             'floor_margin_m', 'arm_collision_margin_m', 'frame_warn_translation_m',
             'frame_warn_rotation_deg', 'frame_fail_translation_m', 'frame_fail_rotation_deg')
    for name in names:
        value = float(settings[name])
        if not math.isfinite(value) or (name != 'floor_z_m' and value < 0):
            raise ValueError(f'Invalid {name} in {path}: {value}')
        settings[name] = value
    settings['arm_self_collision_monitor_only'] = settings.get('arm_self_collision_monitor_only', False)
    if not isinstance(settings['arm_self_collision_monitor_only'], bool):
        raise ValueError(f'Invalid arm_self_collision_monitor_only in {path}')
    if settings['target_diameter_m'] <= 0 or settings['virtual_grasp_width_m'] <= 0:
        raise ValueError(f'Target diameter and grasp width must be positive in {path}')
    if settings.get('manual_target_mode') not in ('ik_only', 'simulated_grasp'):
        raise ValueError(f'Invalid manual_target_mode in {path}')
    return settings


def generate_launch_description():
    manual = _manual_grasp_settings()
    point_lio = FindPackageShare('point_lio_unilidar')
    far = FindPackageShare('far_planner')
    local = FindPackageShare('local_planner')
    robot_urdf = PathJoinSubstitution([FindPackageShare('unitree_legged_real'), 'config', 'go2_rars01_stage4d_rviz.urdf'])
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('policy_path'),
        DeclareLaunchArgument('rl_config_path'),
        DeclareLaunchArgument('mujoco_config', default_value='config_go2_rars01_stage4d.yaml'),
        DeclareLaunchArgument('rars01_arm_sim_config', default_value=PathJoinSubstitution(
            [FindPackageShare('unitree_mujoco'), 'config', 'rars01_arm_sim.yaml'])),
        SetEnvironmentVariable('STAGE4D_RARS01_ARM_SIM_CONFIG', LaunchConfiguration('rars01_arm_sim_config')),
        DeclareLaunchArgument('pointlio_config', default_value=PathJoinSubstitution([point_lio, 'config', 'utlidar_sim.yaml'])),
        DeclareLaunchArgument('far_config', default_value=PathJoinSubstitution([far, 'config', 'sim_pointlio.yaml'])),
        DeclareLaunchArgument('sim_imu_calibration', default_value=PathJoinSubstitution([FindPackageShare('unitree_legged_real'), 'config', 'stage4d_sim_imu_calibration.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_mujoco_hud', default_value=EnvironmentVariable('STAGE4D_MUJOCO_HUD', default_value='true')),
        SetEnvironmentVariable('STAGE4D_MUJOCO_HUD', LaunchConfiguration('enable_mujoco_hud')),
        DeclareLaunchArgument('enable_manual_manip_target', default_value=EnvironmentVariable('STAGE4D_ENABLE_MANUAL_MANIP_TARGET', default_value='true')),
        DeclareLaunchArgument('arm_collision_margin_m', default_value=str(manual['arm_collision_margin_m'])),
        SetEnvironmentVariable('STAGE4D_MANUAL_TARGET_DIAMETER_M', str(manual['target_diameter_m'])),
        DeclareLaunchArgument('enable_planner_visualization', default_value=EnvironmentVariable('STAGE4D_ENABLE_PLANNER_VISUALIZATION', default_value='true')),
        DeclareLaunchArgument('enable_navigation_explainer', default_value=EnvironmentVariable('STAGE4D_ENABLE_NAVIGATION_EXPLAINER', default_value='true')),
        DeclareLaunchArgument('enable_full_navigation_evaluator', default_value=EnvironmentVariable('STAGE4D_ENABLE_FULL_NAVIGATION_EVALUATOR', default_value='true')),
        DeclareLaunchArgument('enable_rviz_robot_visualization', default_value=EnvironmentVariable('STAGE4D_ENABLE_RVIZ_ROBOT_VISUALIZATION', default_value='true')),
        DeclareLaunchArgument('enable_far', default_value='true'),
        DeclareLaunchArgument('enable_local_planner', default_value='true'),
        DeclareLaunchArgument('far_converge_distance', default_value=EnvironmentVariable('STAGE4D_FAR_CONVERGE_DISTANCE', default_value='0.25')),
        DeclareLaunchArgument('goal_close_dis', default_value=EnvironmentVariable('STAGE4D_GOAL_CLOSE_DIS', default_value='0.40')),
        Node(package='unitree_mujoco', executable='unitree_mujoco', output='screen',
             arguments=['--config', LaunchConfiguration('mujoco_config')]),
        Node(package='unitree_legged_real', executable='mujoco_sim', output='screen', parameters=[{
             'policy_path': LaunchConfiguration('policy_path'), 'rl_config_path': LaunchConfiguration('rl_config_path'),
             'initial_navigation_active': False, 'auto_start_rl': True,
             'rars01_arm_sim_config': LaunchConfiguration('rars01_arm_sim_config')}]),
        # Unchanged original raw-sensor boundary: MuJoCo publishes /utlidar/*;
        # transform_sensors remains the sole raw->body conversion.
        Node(package='transform_sensors', executable='transform_everything', name='transform_everything', output='screen',
             parameters=[{'calibration_path': LaunchConfiguration('sim_imu_calibration'), 'preserve_sensor_stamp': True}]),
        Node(package='point_lio_unilidar', executable='pointlio_mapping', name='laserMapping', output='screen',
             parameters=[LaunchConfiguration('pointlio_config'), {'prop_at_freq_of_imu': True, 'check_satu': True,
                 'init_map_size': 10, 'point_filter_num': 1, 'space_down_sample': True,
                 'filter_size_surf': 0.1, 'filter_size_map': 0.1, 'cube_side_length': 1000.0}],
             remappings=[('/cloud_registered', '/registered_scan'), ('/aft_mapped_to_init', '/state_estimation'),
                         ('/path', '/point_lio/path')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_camera_init', output='screen',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='aft_mapped_to_sensor', output='screen',
             arguments=['0', '0', '0', '0', '0', '0', 'aft_mapped', 'sensor']),
        # Point-LIO's body-frame diagnostic cloud uses the same estimated pose
        # as aft_mapped; this is not a second localization transform.
        Node(package='tf2_ros', executable='static_transform_publisher', name='aft_mapped_to_body', output='screen',
             arguments=['0', '0', '0', '0', '0', '0', 'aft_mapped', 'body']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='sensor_to_vehicle', output='screen',
             # sensor is the IMU origin; child vehicle is the Go2 base origin.
             arguments=['0.02557', '0', '-0.04232', '0', '0', '0', 'sensor', 'vehicle']),
        Node(package='terrain_analysis', executable='terrainAnalysis', name='terrainAnalysis', output='screen',
             parameters=[{'worldFrame': 'map'}]),
        Node(package='terrain_analysis_ext', executable='terrainAnalysisExt', name='terrainAnalysisExt', output='screen',
             parameters=[{'worldFrame': 'map', 'checkTerrainConn': False}]),
        Node(package='far_planner', executable='far_planner', name='far_planner', output='screen', condition=IfCondition(LaunchConfiguration('enable_far')),
             parameters=[LaunchConfiguration('far_config'), {'g_planner/converge_distance': LaunchConfiguration('far_converge_distance')}],
             remappings=[('/odom_world', '/state_estimation'), ('/terrain_cloud', '/terrain_map_ext'),
                         ('/scan_cloud', '/terrain_map'), ('/terrain_local_cloud', '/registered_scan')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('graph_decoder'), 'launch', 'decoder.launch']))),
        # The upstream local_planner launch is authoritative for both nodes.
        # These are the explicit Stage4D simulation/autonomy overrides only.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([local, 'launch', 'local_planner.launch'])),
            condition=IfCondition(LaunchConfiguration('enable_local_planner')),
            launch_arguments={
                # /state_estimation is IMU-centred; original localPlanner algebra subtracts
                # sensorOffset in the body yaw direction, so -0.02557 recovers base XY.
                'sensorOffsetX': '-0.02557', 'sensorOffsetY': '0.0', 'cameraOffsetZ': '0.0',
                'autonomyMode': 'true', 'autonomySpeed': '0.35', 'maxSpeed': '0.35',
                'is_real_robot': 'false', 'sendSportCommand': 'false',
                'odomTimeoutSec': '0.5', 'pathTimeoutSec': '0.5', 'allowStaticPath': 'false',
                'stage4dGoalCloseDis': LaunchConfiguration('goal_close_dis'),
                # Stage4D already owns this identity transform; retain the
                # original sensor->camera publisher because no equivalent exists.
                'publishSensorToVehicleTf': 'false', 'publishSensorToCameraTf': 'true',
            }.items()),
        # Mode B: external replay is the sole /path owner; no FAR/localPlanner.
        Node(package='local_planner', executable='pathFollower', name='pathFollower', output='screen',
             condition=UnlessCondition(LaunchConfiguration('enable_local_planner')), parameters=[{
                 'sensorOffsetX': -0.02557, 'sensorOffsetY': 0.0, 'twoWayDrive': False,
                 'maxSpeed': 0.35, 'autonomyMode': True, 'autonomySpeed': 0.35,
                 'goalCloseDis': LaunchConfiguration('goal_close_dis'), 'is_real_robot': False, 'sendSportCommand': False,
                 'odomTimeoutSec': 0.5, 'pathTimeoutSec': 0.5, 'allowStaticPath': False,
                 'followerMotionModel': 'holonomic'}]),
        Node(package='unitree_legged_real', executable='stage4d_readiness.py', output='screen'),
        Node(package='unitree_legged_real', executable='stage4d_terrain_diagnostics.py', output='screen'),
        # Visualization is isolated under sim_visual_* and never supplies navigation TF.
        Node(package='unitree_legged_real', executable='stage4d_robot_state_bridge.py', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='stage4d_robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(Command(['cat ', robot_urdf]), value_type=str)}],
             remappings=[('joint_states', '/stage4d/joint_states')]),
        # Full RobotModel and the BLUE GT path are visualization/evaluation only.
        Node(package='unitree_legged_real', executable='stage4d_rviz_robot.py', output='screen', condition=IfCondition(LaunchConfiguration('enable_rviz_robot_visualization'))),
        Node(package='unitree_legged_real', executable='stage4d_planner_visualization.py', output='screen', condition=IfCondition(LaunchConfiguration('enable_planner_visualization'))),
        Node(package='unitree_legged_real', executable='stage4d_navigation_explainer.py', output='screen', condition=IfCondition(LaunchConfiguration('enable_navigation_explainer'))),
        Node(package='unitree_legged_real', executable='stage4d_scene_markers.py', output='screen', arguments=['--scene', PathJoinSubstitution([FindPackageShare('unitree_mujoco'), 'scene', 'scene_stage4d.xml'])]),
        # M0.2 simulation-only terminal executor; dormant until a Left-Alt target is set in MuJoCo.
        Node(package='unitree_mujoco', executable='stage4d_arm_shadow_checker', output='screen',
             parameters=[{'margin_m': LaunchConfiguration('arm_collision_margin_m'),
                          'monitor_self_collisions': bool(manual['arm_self_collision_monitor_only'])}],
             condition=IfCondition(LaunchConfiguration('enable_manual_manip_target'))),
        Node(package='unitree_legged_real', executable='stage4d_manual_manip_target.py', output='screen',
             arguments=['--manual-target-mode', manual['manual_target_mode'],
                        '--virtual-grasp-width-m', str(manual['virtual_grasp_width_m']),
                        '--floor-z-m', str(manual['floor_z_m']),
                        '--floor-margin-m', str(manual['floor_margin_m']),
                        '--frame-warn-translation-m', str(manual['frame_warn_translation_m']),
                        '--frame-warn-rotation-deg', str(manual['frame_warn_rotation_deg']),
                        '--frame-fail-translation-m', str(manual['frame_fail_translation_m']),
                        '--frame-fail-rotation-deg', str(manual['frame_fail_rotation_deg'])],
             additional_env={'PATH': [EnvironmentVariable('RARS01_GRASPNET_ROOT', default_value='/home/ruben/go2_diploma_sim2sim/repos/rars01_graspnet'), '/.venv/bin:', EnvironmentVariable('PATH')]},
             condition=IfCondition(LaunchConfiguration('enable_manual_manip_target'))),
        Node(package='unitree_legged_real', executable='stage4d_full_navigation_evaluator.py', output='screen', condition=IfCondition(LaunchConfiguration('enable_full_navigation_evaluator'))),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('unitree_legged_real'), 'config', 'stage4d_full_navigation.rviz'])]),
    ])
