import pathlib
import unittest
ROOT = pathlib.Path(__file__).parents[1]
WORKHOP = ROOT.parents[1]
AUTONOMY = WORKHOP.parent / 'autonomy_nav_go2'

class Stage4DOriginalStackContract(unittest.TestCase):
    def test_original_sensor_chain_and_far_remaps(self):
        launch = (ROOT/'launch/stage4d_full_navigation.launch.py').read_text()
        for token in ('transform_everything', 'utlidar_sim.yaml', '/registered_scan', '/state_estimation',
                      "('/terrain_cloud', '/terrain_map_ext')", "('/scan_cloud', '/terrain_map')",
                      "('/terrain_local_cloud', '/registered_scan')", 'local_planner.launch', "'goalCloseDis': '0.3'"):
            self.assertIn(token, launch)
        self.assertNotIn('pointlio_state_adapter.py', launch)
        self.assertNotIn('stage4d_graph_bootstrap.py', launch)
    def test_mujoco_raw_sensor_contract(self):
        source = (WORKHOP/'src/unitree_mujoco/simulate/src/main.cc').read_text()
        for token in ('/utlidar/cloud', '/utlidar/imu', 'mj_multiRay', 'environment_mask[0] = 1',
                      'modifier.resize(scan_hits_.size())', 'static_cast<float>(distance)*dir[0]',
                      'time; uint16_t ring', 'scan_start_time_', 'sensor_start_delay',
                      'next_imu_time_ = data->time + 0.01',
                      'mujoco_clock->Publish(d)'):
            self.assertIn(token, source)
        self.assertNotIn('exit(0);', source)
        self.assertNotIn('pthread_exit(NULL)', source)
    def test_original_point_lio_config(self):
        config = (AUTONOMY/'src/slam/point_lio_unilidar/config/utlidar_sim.yaml').read_text()
        for token in ('/utlidar/transformed_cloud', '/utlidar/transformed_raw_imu',
                      'lidar_type: 5', 'scan_line: 18', 'timestamp_unit: 0', 'path_en: false'):
            self.assertIn(token, config)
    def test_stage4d_includes_original_local_planner_launch(self):
        stage4d = (ROOT/'launch/stage4d_full_navigation.launch.py').read_text()
        original = (AUTONOMY/'src/base_autonomy/local_planner/launch/local_planner.launch').read_text()
        self.assertIn('AnyLaunchDescriptionSource', stage4d)
        self.assertIn("'local_planner.launch'", stage4d)
        self.assertNotIn("Node(package='local_planner', executable='localPlanner'", stage4d)
        self.assertNotIn("Node(package='local_planner', executable='pathFollower'", stage4d)
        for token in ("'autonomyMode': 'true'", "'is_real_robot': 'false'",
                      "'sendSportCommand': 'false'", "'goalCloseDis': '0.3'",
                      "'publishSensorToVehicleTf': 'false'",
                      "'publishSensorToCameraTf': 'true'"):
            self.assertIn(token, stage4d)
        self.assertIn('<param name="useTerrainAnalysis" value="true" />', original)
        self.assertIn('<arg name="publishSensorToVehicleTf" default="true"/>', original)
        self.assertIn('<arg name="publishSensorToCameraTf" default="true"/>', original)

    def test_far_parameters_match_the_launch_node_name(self):
        launch = (ROOT/'launch/stage4d_full_navigation.launch.py').read_text()
        config = (AUTONOMY/'src/route_planner/far_planner/config/sim_pointlio.yaml').read_text()
        self.assertIn("Node(package='far_planner', executable='far_planner', name='far_planner'", launch)
        self.assertTrue(config.startswith('far_planner:\n  ros__parameters:'))
        self.assertIn('stage4d_terrain_diagnostics.py', launch)

    def test_scene_and_rviz(self):
        scene = (WORKHOP/'src/unitree_mujoco/unitree_robots/go2_rars01/scene_stage4d.xml').read_text()
        self.assertIn('name="stage4d_obstacle_single_01"', scene)
        self.assertIn('group="0"', scene)
        rviz = (ROOT/'config/stage4d_full_navigation.rviz').read_text()
        for token in ('MuJoCo world grid', '/utlidar/cloud', '/utlidar/transformed_cloud', '/registered_scan',
                      '/terrain_map', '/terrain_map_ext', '/stage4d/scene_markers', 'Fixed Frame: map',
                      '/stage4d/ground_truth_path', '/stage4d/planner_footprint', '/free_paths'):
            self.assertIn(token, rviz)
        launch = (ROOT/'launch/stage4d_full_navigation.launch.py').read_text()
        self.assertIn('go2_rars01_stage4d_rviz.urdf', launch)
        self.assertIn('stage4d_planner_visualization.py', launch)
if __name__ == '__main__': unittest.main()
