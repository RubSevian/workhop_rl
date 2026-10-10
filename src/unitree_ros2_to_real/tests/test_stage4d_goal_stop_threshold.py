"""Exercise the actual full-launch override, not a duplicate of its formula."""
import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription


SCRIPT = Path(__file__).parents[1] / 'launch/stage4d_full_navigation.launch.py'
spec = importlib.util.spec_from_file_location('full_navigation_under_test', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GoalStopThresholdTest(unittest.TestCase):
    def test_follower_stops_inside_far_completion_radius(self):
        launch = module.generate_launch_description()
        overrides = [dict(action.launch_arguments) for action in launch.entities
                     if isinstance(action, IncludeLaunchDescription)]
        stop_overrides = [args['stopDisThre'] for args in overrides if 'stopDisThre' in args]
        self.assertEqual(len(stop_overrides), 1)
        for stop, far, expected in [('0.30', '0.25', 0.225),
                                    ('0.15', '0.15', 0.135),
                                    ('0.10', '0.25', 0.10)]:
            with self.subTest(stop=stop, far=far):
                context = LaunchContext()
                context.launch_configurations.update(stop_dis_thre=stop,
                                                     far_converge_distance=far)
                value = float(stop_overrides[0].perform(context))
                self.assertAlmostEqual(value, expected)
                self.assertLess(value, float(far))


if __name__ == '__main__':
    unittest.main()
