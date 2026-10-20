"""Status decoding parity; no DDS, serial access or robot commands."""
from pathlib import Path
import math
import sys
import unittest
from unittest.mock import patch
import random
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

    def test_native_flow_avoids_yaml_construction_and_preserves_scientific_strings(self):
        payload='{observed_ns: 123456789, owner_state: HOLD_HOME, q: [0.0, -2.5, 1.2e-05, 1e-06], feedback_ready: true, error: "", object_grasped: ~}'
        expected=yaml.safe_load(payload)
        with patch('mission.status_codec.yaml.load',side_effect=AssertionError('slow path')):
            self.same(expected,decode_status(payload))

    def test_ambiguous_yaml_and_noncanonical_syntax_use_original_semantics(self):
        for payload in ('{a:b}', '{a:1,b:2}', '{a: 01, b: 09, c: +2}',
                        '{a: 1.2e06, b: .5, c: -.5, d: 1.}',
                        '{a: 2026-10-20, b: 0x10}', '{yes: yes, no: Off}',
                        "{a: 'one: two', b: [on, null, ~]}", '{a: &id [1, 2], b: *id}',
                        '{a: \"\\\\x41\", b: \"\\\\N\"}', '{a: 1,}', '{}',
                        '{a: unquoted words}', '{a: 1, a: 2}'):
            with self.subTest(payload=payload):self.same(yaml.safe_load(payload),decode_status(payload))

    def test_seeded_nested_mapping_equivalence(self):
        rng=random.Random(8)
        scalars=[None,True,False,0,-4,2**64-1,.5,-.00001,'on','false','1e-06',
                 'HOLD_HOME','a:b','two words','quote\"slash\\\\','line\\nnext']
        for _ in range(150):
            value={'q':[rng.choice(scalars) for _ in range(7)],
                   'nested':{'error':rng.choice(scalars),'value':rng.choice(scalars)}}
            payload=yaml.safe_dump(value,default_flow_style=True,width=10000)
            self.same(yaml.safe_load(payload),decode_status(payload))

if __name__=='__main__':unittest.main()
