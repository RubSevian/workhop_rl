"""CDR projection parity with installed ROS serializer, without ROS init."""
from pathlib import Path
import math
import random
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.leg_state_reader import LegStateReader,LowState,serialize_message,deserialize_message


class LegReaderTests(unittest.TestCase):
    def test_random_full_messages_match_ros_deserialization(self):
        rng=random.Random(17);reader=LegStateReader()
        self.assertEqual(reader.mode,'fixed_cdr_projection')
        for i in range(100):
            m=LowState();m.tick=rng.randrange(2**32);m.head=[rng.randrange(256),rng.randrange(256)]
            for motor in m.motor_state:
                motor.q=rng.uniform(-3,3);motor.dq=rng.uniform(-10,10);motor.tau_est=rng.uniform(-10,10)
            wire=serialize_message(m);reference=deserialize_message(wire,LowState)
            self.assertEqual(reader.read(wire),(reference.tick,tuple(v.q for v in reference.motor_state[:12])))
            self.assertEqual(reader.read(reference),reader.read(wire))

    def test_nonfinite_values_and_uint32_wrap_are_preserved(self):
        reader=LegStateReader()
        for tick in (0,2**32-1):
            m=LowState();m.tick=tick;m.motor_state[0].q=float('nan');m.motor_state[1].q=float('inf')
            value,q=reader.read(serialize_message(m));self.assertEqual(value,tick)
            self.assertTrue(math.isnan(q[0]));self.assertTrue(math.isinf(q[1]))

    def test_unknown_schema_falls_back_to_ros_decoder(self):
        with patch('mission.leg_state_reader.fixed_type',return_value=False):reader=LegStateReader()
        m=LowState();m.tick=17;m.motor_state[2].q=-1.25
        self.assertEqual(reader.mode,'ros_deserialize');self.assertEqual(reader.read(serialize_message(m)),reader.read(m))

    def test_unexpected_length_or_encoding_cannot_use_offsets(self):
        reader=LegStateReader();m=LowState();m.tick=21;wire=serialize_message(m)
        for value in (wire[:-1],b'\x00\x00'+wire[2:]):
            with patch('mission.leg_state_reader.deserialize_message',return_value=m) as fallback:
                self.assertEqual(reader.read(value)[0],21);fallback.assert_called_once()


if __name__=='__main__':unittest.main()
