import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

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
    with open(get('config_path')) as stream:
        config = yaml.safe_load(stream)['real_deployment']['r3_commissioning']
    runtime, trace = config['runtime'], config['motion_diagnostics']
    def configured(name, section, key):
        value = get(name)
        return value if value else section[key]
    scheduler = configured('policy_scheduler', runtime, 'policy_scheduler')
    if scheduler not in ('fifo20', 'other'):
        raise ValueError('policy_scheduler must be fifo20 or other')
    if scheduler == 'other' and profile != 'read_only':
        raise ValueError('OTHER is available only for explicit read_only inspection')
    parameters = {name: get(name) for name in
                  ('config_path', 'model_path', 'network_interface')}
    parameters.update(operation_profile=profile, enable_actuator_output=False,
                      cooperative_policy_registration=scheduler == 'fifo20',
                      policy_worker_self_test_frames=int(get('policy_worker_self_test_frames')),
                      io_diagnostics_path=get('io_diagnostics_path'),
                      remote_auto_sequence=_boolean(str(configured('remote_auto_sequence', runtime, 'remote_auto_sequence'))),
                      motion_diagnostics_enabled=_boolean(str(configured('motion_diagnostics_enabled', trace, 'enabled'))),
                      motion_diagnostics_duration_s=float(configured('motion_diagnostics_duration_s', trace, 'duration_s')),
                      motion_diagnostics_path=str(configured('motion_diagnostics_path', trace, 'path')))
    # Empty compatibility args are omitted: the immutable profile supplies defaults.
    for name in ('read_only', 'remote_test_mode', 'motion_commands_enabled',
                 'controlled_stop_lie_down_trial'):
        value = get(name)
        if value:
            parameters[name] = _boolean(value)
    if get('control_mode'):
        parameters['control_mode'] = get('control_mode')
    prefix = [PathJoinSubstitution([FindPackagePrefix('unitree_legged_real'),
               'lib', 'unitree_legged_real', 'r3_rt_guardian.py'])] if scheduler == 'fifo20' else None
    return [Node(package='unitree_legged_real', executable='go2_r3_commissioning', prefix=prefix,
                 output='screen', parameters=[parameters],
                 # Guardian holds an active child until native output OFF. Do
                 # not let launch's default five-second escalation kill it.
                 sigterm_timeout=str(runtime['launch_sigterm_timeout_s']), sigkill_timeout=str(runtime['launch_sigkill_timeout_s']),
                 additional_env={'OMP_NUM_THREADS': '1', 'R3_RT_CONFIG_PATH': get('config_path')})]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([
            FindPackageShare('unitree_legged_real'), 'config', 'go2_rars01_real.yaml'])),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('network_interface', default_value=EnvironmentVariable(
            'GO2_NETWORK_INTERFACE', default_value='')),
        DeclareLaunchArgument('operation_profile', default_value=''),
        DeclareLaunchArgument('policy_scheduler', default_value=''),
        DeclareLaunchArgument('policy_worker_self_test_frames', default_value='0'),
        DeclareLaunchArgument('io_diagnostics_path', default_value=''),
        DeclareLaunchArgument('remote_auto_sequence', default_value=''),
        DeclareLaunchArgument('motion_diagnostics_enabled', default_value=''),
        DeclareLaunchArgument('motion_diagnostics_duration_s', default_value=''),
        DeclareLaunchArgument('motion_diagnostics_path', default_value=''),
        DeclareLaunchArgument('remote_test_mode', default_value=''),
        DeclareLaunchArgument('control_mode', default_value=''),
        DeclareLaunchArgument('motion_commands_enabled', default_value=''),
        DeclareLaunchArgument('read_only', default_value=''),
        DeclareLaunchArgument('controlled_stop_lie_down_trial', default_value=''),
        OpaqueFunction(function=_controller),
    ])
