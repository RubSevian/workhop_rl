"""Status decoding parity; no DDS, serial access or robot commands."""
from pathlib import Path
import math
import sys
import unittest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.status_codec import decode_status


class StatusCodecTests(unittest.TestCase):
    def same(self,a,b):
        self.assertIs(type(a),type(b))
        if isinstance(a,dict):
            self.assertEqual(a.keys(),b.keys())
            for k in a:self.same(a[k],b[k])
        elif isinstance(a,list):
            self.assertEqual(len(a),len(b))
            for x,y in zip(a,b):self.same(x,y)
        elif isinstance(a,float) and math.isnan(a):self.assertTrue(math.isnan(b))
        else:self.assertEqual(a,b)

    def test_cpp_flow_and_json_scalar_types(self):
        for payload in (
            '{observed_ns: 24547031215115, feedback_ready: false, q: [.nan, .inf, -.inf, 0.0], object_grasped: ~, error: ""}',
            '{owner_state: HOLD_HOME, q: [1.2e-05, -2.5, 0], owner_session_id: 18446744073709551615}',
            '{"ready": true, "observed_ns": 123456789012345, "age": 1.0e-05, "reason": null}',
        ):self.same(yaml.safe_load(payload),decode_status(payload))

    def test_rejects_unsafe_tags_malformed_and_non_mapping(self):
        for payload in ('!!python/object/apply:os.system [echo BAD]', '{a: [}', 'null', '[1,2]', 'x'*2_000_001):
            with self.assertRaises((ValueError,yaml.YAMLError)):decode_status(payload)


if __name__=='__main__':unittest.main()
