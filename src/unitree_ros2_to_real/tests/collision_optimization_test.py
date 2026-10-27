"""Cache invalidation and conditional transforms on the real model, synthetic scene."""
import copy,time,unittest
import numpy as np
from collision_alignment_test import Collision,CFG,GEOMETRY,HOME,READY,LEGS

class OptimizationTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.base=Collision(CFG,GEOMETRY)
 def test_mapping_and_no_output_alias(self):
  c=self.base
  self.assertEqual(len(c._joint_mapping),20)
  for name,index,lo,hi in c._joint_mapping:
   self.assertEqual(index,c.model.idx_qs[c.model.getJointId(name)])
   self.assertEqual(lo,c.model.lowerPositionLimit[index]);self.assertEqual(hi,c.model.upperPositionLimit[index])
  old=c.placements(HOME,LEGS);snapshot=old.copy();c.placements(READY,LEGS)
  np.testing.assert_array_equal(old,snapshot)
 def test_pair_replacement_refreshes_read_only_cache(self):
  c=copy.copy(self.base);original=c.pairs
  c.pairs=[original[-1]]
  np.testing.assert_array_equal(c._pairs_array,[original[-1]])
  with self.assertRaises(ValueError):c._pairs_array[0,0]=99
  self.assertIsInstance(c.pairs,tuple)
  c.pairs=original
  np.testing.assert_array_equal(c._pairs_array,original)
 def test_world_only_when_consumed_scene_always_validated(self):
  for env,payload_enabled,payload_request in ((False,False,True),(False,False,False),(True,False,False),(False,True,True),(True,True,True)):
   c=copy.copy(self.base);c.cfg=copy.deepcopy(CFG)
   c.environment_enabled=env;c.payload_enabled=payload_enabled
   c.floor=-2.;c.boxes=[];c.payload_size=[.02]*3;c.payload_xyz=[0,0,.2]
   c.cfg['collision'].update(environment_frame='map',scene_revision='synthetic')
   count=[0];validated=[0]
   class Map:
    def __init__(self,array):self.array=array
    def __matmul__(self,rhs):count[0]+=1;return self.array@rhs
   def validate(scene,payload,**kwargs):
    validated[0]+=1
    return Map(Collision.validated_scene(c,scene,payload,**kwargs))
   c.validated_scene=validate
   stamp=time.monotonic_ns()
   scene=dict(legs=LEGS,legs_ns=stamp,feedback_ns=stamp,base_ns=stamp,map_body=np.eye(4).tolist(),scene_revision='synthetic',command_rate_hz=100,velocity_limits=[1.]*7)
   c.check([HOME,HOME],scene,payload_request,reference_ns=stamp)
   self.assertEqual(validated[0],1)
   self.assertEqual(count[0]>0,env or (payload_enabled and payload_request))
   scene['legs_ns']=0
   self.assertFalse(c.check([HOME,HOME],scene,payload_request,reference_ns=stamp))
   self.assertEqual(c.last_reason,'COLLISION_STATE_STALE:legs_ns')

if __name__=='__main__':unittest.main()
