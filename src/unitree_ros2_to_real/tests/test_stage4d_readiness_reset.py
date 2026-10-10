"""A new simulation epoch must not inherit retained readiness/path facts."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SCRIPT = Path(__file__).parents[1] / 'scripts/stage4d_readiness.py'
spec = importlib.util.spec_from_file_location('readiness_under_test', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReadinessResetTest(unittest.TestCase):
    def instance(self):
        return SimpleNamespace(clock_value=100.0, clock_advanced=True,
                               seen={'/state_estimation': 100.0}, nonempty={'/terrain_map': True},
                               pointlio_ok=True, nav_active=True, waypoint=True,
                               path_len=10, nonzero_command=True, pre=Mock(), post=Mock())

    def test_rewind_invalidates_previous_epoch(self):
        node = self.instance()
        msg = SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=100000000))
        module.Readiness.clock(node, msg)
        self.assertFalse(node.clock_advanced)
        self.assertFalse(node.nav_active)
        self.assertFalse(node.waypoint)
        self.assertEqual(node.path_len, 0)
        self.assertFalse(node.nonzero_command)
        self.assertNotIn('/state_estimation', node.seen)
        self.assertFalse(node.nonempty)
        self.assertFalse(node.pre.publish.call_args.args[0].data)
        self.assertFalse(node.post.publish.call_args.args[0].data)

    def test_normal_clock_progress_keeps_current_epoch(self):
        node = self.instance()
        msg = SimpleNamespace(clock=SimpleNamespace(sec=101, nanosec=0))
        module.Readiness.clock(node, msg)
        self.assertTrue(node.clock_advanced)
        self.assertTrue(node.nav_active)
        self.assertEqual(node.path_len, 10)
        node.pre.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
