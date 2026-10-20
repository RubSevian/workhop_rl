"""Keep the actual full probe status inside its bounded snapshot capacity."""
from pathlib import Path
import re
import unittest

class CapacityTest(unittest.TestCase):
    def test_all_capture_fields_fit_and_are_unique(self):
        root=Path(__file__).resolve().parents[1]
        header=(root/'include/control_diagnostics.hpp').read_text()
        capacity=int(re.search(r'capacity\s*=\s*(\d+)',header).group(1))
        source=(root/'src/go2_r3_commissioning.cpp').read_text()
        body=source.split('ControlDiagnostics CaptureStatus(',1)[1].split('void StatusTick(',1)[0]
        keys=re.findall(r'snapshot\.Add\("([^"]+)"',body)
        self.assertEqual(len(keys),len(set(keys)))
        self.assertLessEqual(len(keys),capacity)
        self.assertGreater(len(keys),128) # Includes the originally overflowing probe.

if __name__=='__main__':unittest.main()
