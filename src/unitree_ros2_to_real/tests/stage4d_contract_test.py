import pathlib, unittest
ROOT=pathlib.Path(__file__).parents[1]
class Stage4DContract(unittest.TestCase):
 def test_launch_chain_and_ownership(self):
  s=(ROOT/'launch/stage4d_full_navigation.launch.py').read_text()
  for x in ('terrain_analysis','terrain_analysis_ext','far_planner','localPlanner','pathFollower','stage4d_readiness.py','stage4d_full_navigation_evaluator.py','/cloud_registered','/point_lio/path','camera_init'): self.assertIn(x,s)
  self.assertNotIn('stage4c_pointlio_evaluator.py',s)
 def test_scene_and_frames(self):
  self.assertIn('stage4d_obstacle_single_01',(ROOT.parent/'unitree_mujoco/unitree_robots/go2_rars01/scene_stage4d.xml').read_text())
  self.assertIn('world_frame: camera_init',(ROOT.parents[3]/'repos/autonomy_nav_go2/src/route_planner/far_planner/config/sim_pointlio.yaml').read_text())
 def test_goal_helper(self): self.assertIn("'camera_init'",(ROOT/'scripts/stage4d_send_goal.py').read_text())
if __name__=='__main__': unittest.main()
