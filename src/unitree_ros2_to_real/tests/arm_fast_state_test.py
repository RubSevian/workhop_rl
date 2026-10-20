"""Typed/YAML snapshot parity and ownership negatives; no ROS graph or hardware."""
import json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.arm_fast_state import to_mapping,from_mapping
from rclpy.serialization import serialize_message,deserialize_message
from rars01_state_interfaces.msg import ArmFastState

class Tests(unittest.TestCase):
    def payload(self):
        value=yaml.safe_load((Path(__file__).with_name('fixtures')/'arm_owner_state.yaml').read_text())
        value['owner_session_id']='owner-test';return value
    def same(self,a,b):
        if isinstance(a,dict):
            self.assertEqual(a.keys(),b.keys())
            for k in a:self.same(a[k],b[k])
        elif isinstance(a,list):
            self.assertEqual(len(a),len(b))
            for x,y in zip(a,b):self.same(x,y)
        elif isinstance(a,float) and math.isnan(a):self.assertTrue(math.isnan(b))
        else:self.assertEqual(a,b)
    def test_wire_parity_and_conditional_accepted_target(self):
        for accepted in (False,True):
            a=self.payload()
            if accepted:a.update(accepted_ns=2**63+11,accepted_target=[.125]*6)
            else:a.pop('accepted_ns',None);a.pop('accepted_target',None)
            a['observed_ns']=2**63+1;a['q'][0]=float('nan');a['dq'][1]=float('inf')
            a['torque'][2]=-float('inf')
            msg=deserialize_message(serialize_message(from_mapping(a)),ArmFastState)
            self.same(a,to_mapping(msg))
    def test_version_rejected(self):
        msg=from_mapping(self.payload());msg.schema_version=2
        with self.assertRaisesRegex(ValueError,'SCHEMA'):to_mapping(msg)
    def test_oversized_text_rejected(self):
        msg=from_mapping(self.payload());msg.mission_error='x'*2_000_001
        with self.assertRaisesRegex(ValueError,'oversized'):to_mapping(msg)
    def test_duplicate_and_restarted_owner_not_admitted(self):
        from grasp_mission import Coordinator
        payload=self.payload();payload['observed_ns']=300
        for publishers,old_gid,minimum in [([],None,0),([NS(endpoint_gid=b'a'),NS(endpoint_gid=b'b')],None,0),([NS(endpoint_gid=b'a')],b'a',400)]:
            received={'owner':1};calls=[]
            node=NS(cfg={'frames':{'arm_history_depth':32}},arm_state_transport='typed',topics={'owner':'test'},gids={} if old_gid is None else {'owner':old_gid},gid_since={'owner':minimum},get_publishers_info_by_topic=lambda _:publishers,engine=NS(received=received,state='IDLE',observe=lambda *x:calls.append(x)))
            Coordinator.observe(node,'owner',from_mapping(payload),None)
            self.assertFalse(calls)
            # Existing valid state survives a delayed sample without renewed
            # receipt time, exactly as on the legacy path; its TTL still ages.
            if old_gid is None:self.assertNotIn('owner',received)
            else:self.assertEqual(received,{'owner':1})

if __name__=='__main__':unittest.main()
