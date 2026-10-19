"""One launch; default is inspection only (no owners, sensors or motor commands)."""
from pathlib import Path
import importlib.util
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def _launch(context):
    get=lambda key:LaunchConfiguration(key).perform(context)
    share=Path(get_package_share_directory('unitree_legged_real'))
    spec=importlib.util.spec_from_file_location('mission_config',Path(get_package_prefix('unitree_legged_real'))/'lib/unitree_legged_real/mission/config.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    cfg=module.load(get('mission_config_path'),get('workspace'))
    if get('physical') not in ('true','false') or get('start_arm_owner') not in ('true','false'):raise ValueError('explicit true/false required')
    physical=get('physical')=='true'
    failures=module.blockers(cfg)
    if physical and (not cfg['physical_enabled'] or failures):
        raise RuntimeError('PHYSICAL_NOT_READY: '+', '.join(failures+([] if cfg['physical_enabled'] else ['mission_physical_disabled'])))
    params=dict(mission_config_path=get('mission_config_path'),workspace=get('workspace'),physical=physical)
    coordinator=Node(package='unitree_legged_real',executable='grasp_mission.py',parameters=[params],output='screen')
    perception=Node(package='unitree_legged_real',executable='grasp_perception.py',prefix=[cfg['paths']['python']],parameters=[params],output='screen',additional_env=dict(PYTHONUNBUFFERED='1',**{key:str(cfg['compute']['cpu_threads']) for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')}))
    if not physical:return [coordinator,perception]
    actions=[coordinator]
    # No second arm SDK client. Reusing an already-running owner is allowed only
    # if it reports the mission-capable contract (checked before R3 takeover).
    if get('start_arm_owner')=='true':
        owner=cfg['owner']
        actions.append(Node(package='unitree_legged_real',executable='rars_r3_owner',parameters=[dict(
            mission_config_path=get('mission_config_path'),workspace=get('workspace'),
            sdk_config_path=cfg['paths']['sdk_config'],config_path=cfg['paths']['deployment'],
            read_only=False,connect_serial=True,device_path=owner['device_path'],
            lock_directory=str(Path(get('workspace'))/owner['lock_directory']),
            journal_path=str(Path(get('workspace'))/owner['journal_path']))],output='screen'))
    nav_share=Path(get_package_share_directory('go2_navigation_bridge'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch/real_navigation.launch.py')),
        launch_arguments=dict(stage='far',rviz='false',observe_only='false',profile_path=cfg['paths']['navigation']).items()))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/go2_rars01_r3_commissioning.launch.py')),
        launch_arguments=dict(operation_profile='full_mission',config_path=cfg['paths']['deployment'],
            model_path=str(Path(get('workspace'))/'weights/policy_3.pt'),network_interface=get('network_interface')).items()))
    actions.append(Node(package='robot_state_publisher',executable='robot_state_publisher',name='rars01_state_publisher',
        parameters=[dict(robot_description=Path(cfg['paths']['arm_description']).read_text())],
        remappings=[('/joint_states','/rars01/joint_states')],output='screen'))
    mount=cfg['frames']['vehicle_to_mount']
    actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='rars01_vehicle_mount',arguments=[
        '--frame-id',cfg['frames']['navigation'],'--child-frame-id','arm_mount_link',
        *[value for key,number in zip(('--x','--y','--z','--roll','--pitch','--yaw'),mount['xyz_m']+mount['rpy_rad']) for value in (key,str(number))]]))
    import numpy as np
    from scipy.spatial.transform import Rotation
    matrix=np.load(cfg['paths']['hand_eye'],allow_pickle=False)['T_result']
    quaternion=Rotation.from_matrix(matrix[:3,:3]).as_quat()
    actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='rars01_eye_in_hand',arguments=[
        '--frame-id',cfg['frames']['tcp'],'--child-frame-id',cfg['frames']['optical'],
        *[value for key,number in zip(('--x','--y','--z','--qx','--qy','--qz','--qw'),list(matrix[:3,3])+list(quaternion)) for value in (key,str(number))]]))
    # Heavy preload precedes NAV/owner startup. Completion only starts their
    # processes; their existing live readiness and operator takeover still gate motion.
    startup_spec=importlib.util.spec_from_file_location('mission_startup',Path(get_package_prefix('unitree_legged_real'))/'lib/unitree_legged_real/mission/startup.py')
    startup=importlib.util.module_from_spec(startup_spec);startup_spec.loader.exec_module(startup)
    release=startup.StartupRelease()
    def on_preload_output(event):
        if release.feed(event.text):
            return [LogInfo(msg='Preload checks passed; starting NAV and controllers. Camera standby.'),*actions]
        return []
    return [RegisterEventHandler(OnProcessIO(target_action=perception,on_stdout=on_preload_output)),perception]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace',default_value=str(Path.home()/'go2_diploma/sim2real')),
        DeclareLaunchArgument('mission_config_path',default_value=PathJoinSubstitution([LaunchConfiguration('workspace'),'repos/rars01_graspnet/config/jetson_orin_nano.yaml'])),
        DeclareLaunchArgument('physical',default_value='false'),
        DeclareLaunchArgument('start_arm_owner',default_value='true'),
        DeclareLaunchArgument('network_interface',default_value='enP8p1s0'),
        OpaqueFunction(function=_launch)])
