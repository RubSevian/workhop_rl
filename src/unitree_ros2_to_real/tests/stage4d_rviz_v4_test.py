import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).parents[1]
WORKHOP = ROOT.parents[1]
AUTONOMY = WORKHOP.parent / 'autonomy_nav_go2'
EXPECTED = {
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
    'gripper_left_joint', 'gripper_right_joint',
}

class Stage4DRvizV4Test(unittest.TestCase):
    def setUp(self):
        self.urdf = ET.parse(ROOT / 'config/go2_rars01_stage4d_rviz.urdf').getroot()

    def test_full_derivative_has_isolated_20_joint_tree(self):
        links = {link.get('name') for link in self.urdf.findall('link')}
        self.assertTrue(links)
        self.assertTrue(all(name.startswith('sim_visual_') for name in links))
        movable = set()
        for joint in self.urdf.findall('joint'):
            self.assertTrue(joint.find('parent').get('link').startswith('sim_visual_'))
            self.assertTrue(joint.find('child').get('link').startswith('sim_visual_'))
            if joint.get('type') in ('revolute', 'continuous', 'prismatic'):
                movable.add(joint.get('name'))
        self.assertEqual(movable, EXPECTED)

    def test_all_mesh_uris_resolve_in_unitree_mujoco_assets(self):
        assets = WORKHOP / 'src/unitree_mujoco/unitree_robots'
        for mesh in self.urdf.findall('.//mesh'):
            uri = mesh.get('filename')
            self.assertTrue(uri.startswith('package://unitree_mujoco/assets/'), uri)
            relative = uri.removeprefix('package://unitree_mujoco/assets/')
            if relative.startswith('go2/'):
                source = assets / 'go2/assets' / relative.removeprefix('go2/')
            else:
                source = assets / 'go2_rars01/assets' / relative
            self.assertTrue(source.is_file(), source)

    def test_20_named_joint_transport_and_leg_slot_order(self):
        source = (ROOT / 'src/mujoco_sim.cpp').read_text()
        bridge = (ROOT / 'scripts/stage4d_robot_state_bridge.py').read_text()
        expected_order = ['FR_hip_joint', 'FL_hip_joint', 'RR_hip_joint', 'RL_hip_joint',
                          'joint1', 'joint6', 'gripper_left_joint', 'gripper_right_joint']
        indices = [source.index('"' + name + '"') for name in expected_order]
        self.assertEqual(indices, sorted(indices))
        self.assertIn('std::array<const char*, 20> motor_names', source)
        self.assertIn('set(positions) != set(JOINT_NAMES)', bridge)
        self.assertNotIn('LEG_JOINTS', bridge)

    def test_launch_uses_full_robot_and_single_derived_paths(self):
        launch = (ROOT / 'launch/stage4d_full_navigation.launch.py').read_text()
        self.assertIn('go2_rars01_stage4d_rviz.urdf', launch)
        self.assertNotIn("'config', 'go2_rars01_rviz.urdf'", launch)
        self.assertIn('stage4d_rviz_robot.py', launch)
        self.assertIn('stage4d_planner_visualization.py', launch)
        self.assertIn('stage4d_navigation_explainer.py', launch)
        self.assertNotIn('stage4d_odometry_path.py', launch)
        planner_viz = (ROOT / 'scripts/stage4d_planner_visualization.py').read_text()
        self.assertIn("transform.header.frame_id = 'camera_init'", planner_viz)
        self.assertIn("transform.child_frame_id = 'aft_mapped'", planner_viz)
        self.assertIn("'/state_estimation'", planner_viz)
        self.assertNotIn('/sim/ground_truth_odom', planner_viz)
        fake = (ROOT / 'scripts/stage4d_rviz_robot.py').read_text()
        self.assertNotIn('/stage4d/robot_model', fake)
        self.assertNotIn('MarkerArray', fake)

    def test_truthful_color_and_geometry_contract(self):
        rviz = (ROOT / 'config/stage4d_full_navigation.rviz').read_text()
        for token in ('FAR route — GREEN', 'MuJoCo actual trajectory — BLUE',
                      'Local planner path — ORANGE', 'Point-LIO trajectory — CYAN',
                      '/stage4d/planner_footprint', '/free_paths',
                      '/stage4d/localplanner_obstacles_viz', '/stage4d/far_free_viz',
                      '/stage4d/far_obstacles_viz', '/registered_scan',
                      'rviz_default_plugins/Orbit',
                      'Description Source: Topic', 'Description Topic:', 'rviz_default_plugins/MoveCamera',
                      'rviz_default_plugins/FocusCamera', '/robot_description'):
            self.assertIn(token, rviz)
        self.assertNotIn('/stage4d/navigation_status_marker', rviz)
        explainer = (ROOT / 'scripts/stage4d_navigation_explainer.py').read_text()
        self.assertIn('/stage4d/navigation_explanation', explainer)
        far = (AUTONOMY / 'src/route_planner/far_planner/src/planner_visualizer.cpp').read_text()
        self.assertIn('is_free_nav ? VizColor::GREEN : VizColor::EMERALD', far)
        local = (AUTONOMY / 'src/base_autonomy/local_planner/src/localPlanner.cpp').read_text()
        for token in ('"vehicle_length"', '"correspondence_search_radius"',
                      'relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw'):
            self.assertIn(token, local)

if __name__ == '__main__':
    unittest.main()
