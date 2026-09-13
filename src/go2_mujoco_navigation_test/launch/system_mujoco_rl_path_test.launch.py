from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    test_path = LaunchConfiguration('test_path')
    config_path = LaunchConfiguration('config_path')
    model_path = LaunchConfiguration('model_path')
    csv_path = LaunchConfiguration('csv_path')
    return LaunchDescription([
        DeclareLaunchArgument('test_path', default_value='straight'),
        DeclareLaunchArgument('config_path', default_value=''),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('csv_path', default_value='/tmp/go2_mujoco_smoke_test.csv'),
        Node(
            package='go2_mujoco_navigation_test', executable='mujoco_ground_truth_odom',
            name='mujoco_ground_truth_odom', output='screen',
            parameters=[{'sport_state_topic': 'sportmodestate', 'low_state_topic': 'lowstate'}]),
        Node(
            package='local_planner', executable='pathFollower', name='pathFollower', output='screen',
            parameters=[{
                'autonomyMode': True, 'autonomySpeed': 0.30,
                'maxSpeed': 0.30, 'maxYawRate': 28.0, 'maxAccel': 1.0,
                'twoWayDrive': False, 'pubSkipNum': 1,
                'odomTimeoutSec': 0.50, 'pathTimeoutSec': 0.50,
                'is_real_robot': False, 'sendSportCommand': False,
            }]),
        Node(
            package='unitree_legged_real', executable='ros2_rl_go2', name='rl_locomotion', output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel', 'cmd_vel_timeout_sec': 0.25,
                'low_state_timeout_sec': 0.50,
                'max_linear_x': 0.30, 'max_linear_y': 0.30, 'max_yaw_rate': 0.50,
                'autostart': True, 'low_level_mode_verified': True,
                'robot_name': 'go2', 'config_path': config_path, 'model_path': model_path,
            }]),
        Node(
            package='go2_mujoco_navigation_test', executable='test_path_publisher',
            name='test_path_publisher', output='screen',
            parameters=[{'test_path': test_path, 'csv_path': csv_path}]),
    ])
