from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    point_lio = FindPackageShare('point_lio_unilidar')
    far = FindPackageShare('far_planner')
    local = FindPackageShare('local_planner')
    robot_urdf = PathJoinSubstitution([FindPackageShare('unitree_legged_real'), 'config', 'go2_rars01_stage4d_rviz.urdf'])
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('policy_path'),
        DeclareLaunchArgument('rl_config_path'),
        DeclareLaunchArgument('mujoco_config', default_value='config_go2_rars01_stage4d.yaml'),
        DeclareLaunchArgument('pointlio_config', default_value=PathJoinSubstitution([point_lio, 'config', 'utlidar_sim.yaml'])),
        DeclareLaunchArgument('far_config', default_value=PathJoinSubstitution([far, 'config', 'sim_pointlio.yaml'])),
        DeclareLaunchArgument('sim_imu_calibration', default_value=PathJoinSubstitution([FindPackageShare('unitree_legged_real'), 'config', 'stage4d_sim_imu_calibration.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='unitree_mujoco', executable='unitree_mujoco', output='screen',
             arguments=['--config', LaunchConfiguration('mujoco_config')]),
        Node(package='unitree_legged_real', executable='mujoco_sim', output='screen', parameters=[{
             'policy_path': LaunchConfiguration('policy_path'), 'rl_config_path': LaunchConfiguration('rl_config_path'),
             'initial_navigation_active': False, 'auto_start_rl': True}]),
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
             arguments=['0', '0', '0', '0', '0', '0', 'sensor', 'vehicle']),
        Node(package='terrain_analysis', executable='terrainAnalysis', name='terrainAnalysis', output='screen',
             parameters=[{'worldFrame': 'map'}]),
        Node(package='terrain_analysis_ext', executable='terrainAnalysisExt', name='terrainAnalysisExt', output='screen',
             parameters=[{'worldFrame': 'map', 'checkTerrainConn': False}]),
        Node(package='far_planner', executable='far_planner', name='far_planner', output='screen',
             parameters=[LaunchConfiguration('far_config')],
             remappings=[('/odom_world', '/state_estimation'), ('/terrain_cloud', '/terrain_map_ext'),
                         ('/scan_cloud', '/terrain_map'), ('/terrain_local_cloud', '/registered_scan')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('graph_decoder'), 'launch', 'decoder.launch']))),
        # The upstream local_planner launch is authoritative for both nodes.
        # These are the explicit Stage4D simulation/autonomy overrides only.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([local, 'launch', 'local_planner.launch'])),
            launch_arguments={
                'sensorOffsetX': '0.0', 'sensorOffsetY': '0.0', 'cameraOffsetZ': '0.0',
                'autonomyMode': 'true', 'autonomySpeed': '0.35', 'maxSpeed': '0.35',
                'is_real_robot': 'false', 'sendSportCommand': 'false',
                'odomTimeoutSec': '0.5', 'pathTimeoutSec': '0.5', 'allowStaticPath': 'false',
                'goalCloseDis': '0.3',
                # Stage4D already owns this identity transform; retain the
                # original sensor->camera publisher because no equivalent exists.
                'publishSensorToVehicleTf': 'false', 'publishSensorToCameraTf': 'true',
            }.items()),
        Node(package='unitree_legged_real', executable='stage4d_readiness.py', output='screen'),
        Node(package='unitree_legged_real', executable='stage4d_terrain_diagnostics.py', output='screen'),
        # Visualization is isolated under sim_visual_* and never supplies navigation TF.
        Node(package='unitree_legged_real', executable='stage4d_robot_state_bridge.py', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='stage4d_robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(Command(['cat ', robot_urdf]), value_type=str)}],
             remappings=[('joint_states', '/stage4d/joint_states')]),
        # Full RobotModel and the BLUE GT path are visualization/evaluation only.
        Node(package='unitree_legged_real', executable='stage4d_rviz_robot.py', output='screen'),
        Node(package='unitree_legged_real', executable='stage4d_planner_visualization.py', output='screen'),
        Node(package='unitree_legged_real', executable='stage4d_navigation_explainer.py', output='screen'),
        Node(package='unitree_legged_real', executable='stage4d_scene_markers.py', output='screen', arguments=['--scene', PathJoinSubstitution([FindPackageShare('unitree_mujoco'), 'scene', 'scene_stage4d.xml'])]),
        Node(package='unitree_legged_real', executable='stage4d_full_navigation_evaluator.py', output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('unitree_legged_real'), 'config', 'stage4d_full_navigation.rviz'])]),
    ])
