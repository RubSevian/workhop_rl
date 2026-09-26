from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p=FindPackageShare('point_lio_unilidar'); far=FindPackageShare('far_planner')
    urdf_path=PathJoinSubstitution([FindPackageShare('unitree_mujoco'),'urdf','go2_arm_dynamic_train_mujoco.urdf'])
    robot_description=open(get_package_share_directory('unitree_mujoco')+'/urdf/go2_arm_dynamic_train_mujoco.urdf').read().replace('../assets/rars01/','package://unitree_mujoco/assets/rars01/')
    return LaunchDescription([
      DeclareLaunchArgument('policy_path'), DeclareLaunchArgument('rl_config_path'),
      DeclareLaunchArgument('mujoco_config',default_value='config_go2_rars01_stage4d.yaml'),
      DeclareLaunchArgument('pointlio_config',default_value=PathJoinSubstitution([p,'config','unilidar_sim.yaml'])),
      DeclareLaunchArgument('far_config',default_value=PathJoinSubstitution([far,'config','sim_pointlio.yaml'])),
      DeclareLaunchArgument('rviz',default_value='false'), DeclareLaunchArgument('report_path',default_value='/tmp/stage4d_full_navigation_report.json'),
      Node(package='unitree_mujoco', executable='unitree_mujoco', arguments=['--config',LaunchConfiguration('mujoco_config')], output='screen'),
      Node(package='unitree_legged_real', executable='mujoco_sim', output='screen', parameters=[{'policy_path':LaunchConfiguration('policy_path'),'rl_config_path':LaunchConfiguration('rl_config_path'),'initial_navigation_active':False,'auto_start_rl':True}]),
      Node(package='point_lio_unilidar', executable='pointlio_mapping', output='screen', parameters=[LaunchConfiguration('pointlio_config')], remappings=[('/path','/point_lio/path')]),
      Node(package='unitree_legged_real', executable='pointlio_state_adapter.py', arguments=['--frame','map'], output='screen'),
      Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_camera_init', arguments=['0','0','0','0','0','0','map','camera_init'], output='screen'),
      Node(package='terrain_analysis', executable='terrainAnalysis', output='screen', remappings=[('/registered_scan','/cloud_registered')], parameters=[{'worldFrame':'map','vehicleHeight':1.5,'clearingDis':8.0,'noDataObstacle':True}]),
      Node(package='terrain_analysis_ext', executable='terrainAnalysisExt', output='screen', remappings=[('/registered_scan','/cloud_registered')], parameters=[{'worldFrame':'map','vehicleHeight':1.5,'clearingDis':30.0}]),
      Node(package='far_planner', executable='far_planner', output='screen', parameters=[LaunchConfiguration('far_config')], remappings=[('/odom_world','/state_estimation'),('/terrain_cloud','/terrain_map_ext'),('/scan_cloud','/terrain_map'),('/terrain_local_cloud','/cloud_registered')]),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('graph_decoder'),'launch','decoder.launch']))),
      Node(package='unitree_legged_real', executable='stage4d_graph_bootstrap.py', output='screen', arguments=['--graph-path',PathJoinSubstitution([FindPackageShare('far_planner'),'data','boundary_graph.vgh'])]),
      Node(package='local_planner', executable='localPlanner', name='localPlanner', output='screen', parameters=[{'pathFolder':PathJoinSubstitution([FindPackageShare('local_planner'),'paths']),'is_real_robot':False,'autonomyMode':True,'autonomySpeed':0.35,'maxSpeed':0.35,'allowStaticPath':False,'pathTimeoutSec':0.5,'odomTimeoutSec':0.5,'goalCloseDis':0.3}], remappings=[('/registered_scan','/cloud_registered')]),
      Node(package='local_planner', executable='pathFollower', name='pathFollower', output='screen', parameters=[{'is_real_robot':False,'sendSportCommand':False,'autonomyMode':True,'autonomySpeed':0.35,'maxSpeed':0.35,'maxYawRate':18.0,'stopDisThre':0.2,'goalCloseDis':0.3,'odomTimeoutSec':0.5,'pathTimeoutSec':0.5,'allowStaticPath':False}]),
      Node(package='unitree_legged_real', executable='stage4d_readiness.py', output='screen'),
      Node(package='unitree_legged_real', executable='stage4d_robot_state_bridge.py', output='screen'),
      Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description':robot_description,'use_sim_time':True}]),
      Node(package='unitree_legged_real', executable='stage4d_full_navigation_evaluator.py', output='screen', arguments=['--report',LaunchConfiguration('report_path')]),
      Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')), arguments=['-d',PathJoinSubstitution([FindPackageShare('unitree_legged_real'),'config','stage4d_full_navigation.rviz'])], output='screen'),
    ])
