from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PROFILES = ('read_only', 'arm_test', 'leg_safety_test', 'rl_zero_test',
            'remote_test', 'nav_test', 'full_mission')

def _boolean(value):
    if value.lower() not in ('true', 'false', '1', '0'):
        raise ValueError('Expected explicit boolean, got ' + value)
    return value.lower() in ('true', '1')

def _controller(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    profile = get('operation_profile')
    if profile and profile not in PROFILES:
        raise ValueError('Unknown operation_profile: ' + profile)
    parameters = {name: get(name) for name in
                  ('config_path', 'model_path', 'network_interface')}
    parameters.update(operation_profile=profile, enable_actuator_output=False,
                      remote_auto_sequence=_boolean(get('remote_auto_sequence')))
    # Empty compatibility args are omitted: the immutable profile supplies defaults.
    for name in ('read_only', 'remote_test_mode', 'motion_commands_enabled',
                 'controlled_stop_lie_down_trial'):
        value = get(name)
        if value:
            parameters[name] = _boolean(value)
    if get('control_mode'):
        parameters['control_mode'] = get('control_mode')
    return [Node(package='unitree_legged_real', executable='go2_r3_commissioning',
                 output='screen', parameters=[parameters])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([
            FindPackageShare('unitree_legged_real'), 'config', 'go2_rars01_real.yaml'])),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('network_interface', default_value=EnvironmentVariable(
            'GO2_NETWORK_INTERFACE', default_value='')),
        DeclareLaunchArgument('operation_profile', default_value=''),
        DeclareLaunchArgument('remote_auto_sequence', default_value='true'),
        DeclareLaunchArgument('remote_test_mode', default_value=''),
        DeclareLaunchArgument('control_mode', default_value=''),
        DeclareLaunchArgument('motion_commands_enabled', default_value=''),
        DeclareLaunchArgument('read_only', default_value=''),
        DeclareLaunchArgument('controlled_stop_lie_down_trial', default_value=''),
        OpaqueFunction(function=_controller),
    ])
