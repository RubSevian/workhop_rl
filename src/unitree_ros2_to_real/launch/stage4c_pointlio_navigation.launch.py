from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pointlio_default = PathJoinSubstitution([FindPackageShare("point_lio_unilidar"), "config", "unilidar_sim.yaml"])
    rviz_default = PathJoinSubstitution([FindPackageShare("point_lio_unilidar"), "rviz_cfg", "loam_unilidar_default.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("policy_path", description="Absolute TorchScript policy path"),
        DeclareLaunchArgument("rl_config_path", description="Absolute unified RL YAML path"),
        DeclareLaunchArgument("mujoco_config", default_value="config_go2_rars01_pointlio.yaml"),
        DeclareLaunchArgument("pointlio_config", default_value=pointlio_default),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("lidar_rate", default_value="10.0", description="Configured in MuJoCo YAML"),
        DeclareLaunchArgument("imu_rate", default_value="250.0", description="Fixed 500 Hz physics / 2 divider"),
        DeclareLaunchArgument("goal_x", default_value="1.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("route_duration", default_value="20.0"),
        DeclareLaunchArgument("report_path", default_value="/tmp/stage4c_pointlio_report.json"),
        Node(package="unitree_mujoco", executable="unitree_mujoco",
             arguments=["--config", LaunchConfiguration("mujoco_config")], output="screen"),
        Node(package="unitree_legged_real", executable="mujoco_sim", output="screen",
             parameters=[{"policy_path": LaunchConfiguration("policy_path"),
                          "rl_config_path": LaunchConfiguration("rl_config_path"),
                          "initial_navigation_active": False, "auto_start_rl": True}]),
        Node(package="point_lio_unilidar", executable="pointlio_mapping", output="screen",
             parameters=[LaunchConfiguration("pointlio_config")]),
        Node(package="unitree_legged_real", executable="pointlio_state_adapter.py", output="screen"),
        Node(package="local_planner", executable="pathFollower", name="pathFollower", output="screen",
             parameters=[{"is_real_robot": False, "sendSportCommand": False,
                          "stopDisThre": 0.2, "goalCloseDis": 1.0, "maxSpeed": 0.35,
                          "maxYawRate": 18.0, "autonomyMode": True, "autonomySpeed": 0.35,
                          "odomTimeoutSec": 0.5, "pathTimeoutSec": 0.5, "allowStaticPath": False}]),
        Node(package="unitree_legged_real", executable="stage4c_pointlio_evaluator.py", output="screen",
             arguments=["--mode", "service", "--goal-x", LaunchConfiguration("goal_x"),
                        "--goal-y", LaunchConfiguration("goal_y"),
                        "--duration", LaunchConfiguration("route_duration"),
                        "--report", LaunchConfiguration("report_path")]),
        Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", rviz_default]),
    ])
