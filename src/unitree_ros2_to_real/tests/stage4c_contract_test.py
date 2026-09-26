import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class Stage4CContractTest(unittest.TestCase):
    def test_sensor_bridge_contract(self):
        source = (ROOT.parent / "unitree_mujoco" / "simulate" / "src" / "main.cc").read_text()
        for field in ("x", "y", "z", "intensity", "time"):
            self.assertIn(f'"{field}"', source)
        self.assertIn('"/unilidar/cloud"', source)
        self.assertIn('"/unilidar/imu"', source)
        self.assertIn('physics_ticks_ % 2', source)

    def test_pointlio_config_and_gt_isolation(self):
        cfg = (ROOT.parent / "unitree_mujoco" / "simulate" / "config_go2_rars01_pointlio.yaml").read_text()
        self.assertIn('odom_topic: "/sim/ground_truth_odom"', cfg)
        sim_cfg = (ROOT.parent.parent.parent / "autonomy_nav_go2" / "src" / "slam" /
                   "point_lio_unilidar" / "config" / "unilidar_sim.yaml").read_text()
        self.assertIn('lid_topic: "/unilidar/cloud"', sim_cfg)
        self.assertIn('imu_topic: "/unilidar/imu"', sim_cfg)
        self.assertIn('extrinsic_T:', sim_cfg)

    def test_adapter_does_not_subscribe_to_gt(self):
        source = (ROOT / "scripts" / "pointlio_state_adapter.py").read_text()
        self.assertNotIn("sim/ground_truth_odom", source)
        self.assertIn('"/state_estimation"', source)


if __name__ == "__main__":
    unittest.main()
