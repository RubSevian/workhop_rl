from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path',default_value=PathJoinSubstitution([FindPackageShare('unitree_legged_real'),'config','go2_rars01_real.yaml'])),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('network_interface',default_value=EnvironmentVariable('GO2_NETWORK_INTERFACE',default_value='')),
        DeclareLaunchArgument('remote_auto_sequence',default_value='true'),
        DeclareLaunchArgument('read_only',default_value='true'),
        Node(package='unitree_legged_real',executable='go2_r3_commissioning',output='screen',parameters=[{
            'config_path':LaunchConfiguration('config_path'),'model_path':LaunchConfiguration('model_path'),
            'network_interface':LaunchConfiguration('network_interface'),
            'remote_auto_sequence':ParameterValue(LaunchConfiguration('remote_auto_sequence'),value_type=bool),
            'read_only':ParameterValue(LaunchConfiguration('read_only'),value_type=bool),'enable_actuator_output':False}]),
        # Startup stays disarmed. Fresh remote takeover can start the gated sequence
        # only with read_only=false; SDK calls run in separate child processes.
    ])
