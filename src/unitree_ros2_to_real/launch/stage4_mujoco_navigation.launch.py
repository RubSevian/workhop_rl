from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("policy_path", description="Absolute TorchScript policy path"),
        DeclareLaunchArgument("rl_config_path", description="Absolute unified RL YAML path"),
        DeclareLaunchArgument("mujoco_config", default_value="config_go2_rars01.yaml"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("navigation_active", default_value="false"),
        DeclareLaunchArgument("auto_start_rl", default_value="true"),
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
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "initial_navigation_active": LaunchConfiguration("navigation_active"),
                "auto_start_rl": LaunchConfiguration("auto_start_rl"),
            }],
        ),
    ])
