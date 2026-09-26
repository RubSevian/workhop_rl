from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("policy_path", description="Absolute TorchScript policy path"),
        DeclareLaunchArgument("rl_config_path", description="Absolute unified RL YAML path"),
        DeclareLaunchArgument("mujoco_config", default_value="config_go2_rars01.yaml"),
        DeclareLaunchArgument("goal_x", default_value="1.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_yaw", default_value="0.0"),
        DeclareLaunchArgument("route_duration", default_value="20.0"),
        DeclareLaunchArgument("start_delay", default_value="12.0"),
        DeclareLaunchArgument("report_path", default_value="/tmp/stage4b_navigation_metrics.json"),
        DeclareLaunchArgument("max_yaw_rate", default_value="18.0"),
        Node(
            package="unitree_mujoco",
            executable="unitree_mujoco",
            arguments=["--config", LaunchConfiguration("mujoco_config")],
            output="screen",
        ),
        Node(
            package="unitree_legged_real",
            executable="mujoco_sim",
            output="screen",
            parameters=[{
                "policy_path": LaunchConfiguration("policy_path"),
                "rl_config_path": LaunchConfiguration("rl_config_path"),
                "initial_navigation_active": False,
                "auto_start_rl": True,
            }],
        ),
        Node(
            package="local_planner",
            executable="pathFollower",
            name="pathFollower",
            output="screen",
            parameters=[{
                "is_real_robot": False,
                "sendSportCommand": False,
                "stopDisThre": 0.2,
                "goalCloseDis": 1.0,
                "maxSpeed": 0.35,
                "maxYawRate": ParameterValue(LaunchConfiguration("max_yaw_rate"), value_type=float),
                "autonomyMode": True,
                "autonomySpeed": 0.35,
                "odomTimeoutSec": 0.5,
                "pathTimeoutSec": 0.5,
                "allowStaticPath": False,
            }],
        ),
        Node(
            package="unitree_legged_real",
            executable="stage4b_navigation_test.py",
            output="screen",
            arguments=[
                "--goal-x", LaunchConfiguration("goal_x"),
                "--goal-y", LaunchConfiguration("goal_y"),
                "--goal-yaw", LaunchConfiguration("goal_yaw"),
                "--duration", LaunchConfiguration("route_duration"),
                "--start-delay", LaunchConfiguration("start_delay"),
                "--report", LaunchConfiguration("report_path"),
            ],
        ),
    ])
