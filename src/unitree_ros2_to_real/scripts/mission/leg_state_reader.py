"""Read the twelve joint angles/tick needed by the non-actuator scene nodes.

CDR offsets are discovered from the installed fixed-size ROS type at startup,
not copied from an SDK layout. Other encodings/layouts use the ROS decoder.
The locomotion controller still receives its original complete LowState.
"""
import struct
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_parser.definition import Array, BasicType, NamespacedType
from rosidl_runtime_py.utilities import get_message
from unitree_go.msg import LowState


def fixed_type(slot):
    if isinstance(slot,BasicType):return True
    if isinstance(slot,Array):return fixed_type(slot.value_type)
    if isinstance(slot,NamespacedType):
        return all(fixed_type(s) for s in get_message('/'.join(slot.namespaced_name())).SLOT_TYPES)
    return False


class LegStateReader:
    def __init__(self):
        self.mode='ros_deserialize';self.layout=None;self.projected=0;self.fallback=0
        if not all(fixed_type(s) for s in LowState.SLOT_TYPES):return
        message=LowState();baseline=serialize_message(message)
        if baseline[:4]!=b'\x00\x01\x00\x00':return
        fields=LowState.get_fields_and_field_types()
        if fields.get('tick')!='uint32' or message.motor_state[0].get_fields_and_field_types().get('q')!='float':
            return
        message.tick=0x13572468
        for i in range(12):message.motor_state[i].q=123.25+i*1.5
        wire=serialize_message(message)
        markers=[struct.pack('<I',message.tick)]+[struct.pack('<f',m.q) for m in message.motor_state[:12]]
        if len(wire)!=len(baseline) or any(wire.count(m)!=1 for m in markers):return
        self.layout=(len(wire),[wire.index(m) for m in markers]);self.mode='fixed_cdr_projection'

    def read(self,data):
        if self.layout is not None and isinstance(data,bytes) and len(data)==self.layout[0] and data[:4]==b'\x00\x01\x00\x00':
            self.projected+=1
            offsets=self.layout[1]
            return struct.unpack_from('<I',data,offsets[0])[0],tuple(struct.unpack_from('<f',data,o)[0] for o in offsets[1:])
        self.fallback+=1
        message=data if isinstance(data,LowState) else deserialize_message(data,LowState)
        return int(message.tick),tuple(m.q for m in message.motor_state[:12])

    def status(self):
        return dict(mode=self.mode,projected=self.projected,fallback=self.fallback)
