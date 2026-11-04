"""Diagnostic clocks distinguish work inside a tick from gaps between ticks."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from mission.tick_timing import TickTiming


class TimingTest(unittest.TestCase):
    def test_slow_publish_and_external_gap_preserve_previous_tick(self):
        journal = Mock()
        node = SimpleNamespace(short_journal=journal, input_diagnostics=Mock())
        with patch('mission.tick_timing.time.monotonic_ns', side_effect=[0, 10_000_000, 130_000_000]), patch('mission.tick_timing.time.thread_time_ns', side_effect=[0, 5_000_000, 6_000_000]):
            first = TickTiming()
            first.mark('status_encode')
            first.finish(node)
        row = journal.put.call_args.kwargs['tick']
        self.assertAlmostEqual(row['stages']['status_publish']['wall_s'], .12)
        self.assertAlmostEqual(row['stages']['status_publish']['thread_cpu_s'], .001)
        with patch('mission.tick_timing.time.monotonic_ns', side_effect=[400_000_000, 405_000_000]), patch('mission.tick_timing.time.thread_time_ns', side_effect=[16_000_000, 17_000_000]):
            TickTiming().finish(node)
        data = journal.put.call_args.kwargs
        self.assertEqual(data['previous_tick'], row)
        self.assertAlmostEqual(data['tick']['between_ticks_wall_s'], .27)
        self.assertAlmostEqual(data['tick']['between_ticks_thread_cpu_s'], .01)

    def test_fast_tick_does_not_enqueue_or_require_journal(self):
        node = SimpleNamespace(short_journal=Mock())
        with patch('mission.tick_timing.time.monotonic_ns', side_effect=[0, 1_000_000]), patch('mission.tick_timing.time.thread_time_ns', side_effect=[0, 1_000_000]):
            TickTiming().finish(node)
        node.short_journal.put.assert_not_called()
        node.short_journal = None
        TickTiming().finish(node)


if __name__ == '__main__':
    unittest.main()
