"""Scene payload schema and ownership parity; no ROS or hardware."""
import sys,unittest
from pathlib import Path
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.scene import BodyPose,SceneSnapshot,validate_scene_data

class ScenePayloadTests(unittest.TestCase):
 def make(self,live=None):
  matrix=[[float(i==j) for j in range(4)] for i in range(4)]
  body=BodyPose(10,20,'nav','tf',[r[:] for r in matrix],[r[:] for r in matrix])
  return SceneSnapshot(body,tuple(float(i) for i in range(7)),tuple(float(i) for i in range(12)),21,22,'arm',live,live)
 def test_separate_leg_age_preserves_arm_limit(self):
  cfg={'limits':{'sensor_age_s':.5,'arm_age_s':.25,'legs_age_s':.5},'frames':{'max_state_skew_s':.1}}
  s=self.make();scene=s.data();now=2_000_000_000
  scene['body']['mono_ns']=now;scene['feedback_ns']=now;scene['legs_ns']=now
  scene['live_feedback_ns']=now
  for age in (265_409_144,500_000_000):
   scene['live_legs_ns']=now-age
   validate_scene_data(scene,cfg,now,'nav','arm')
  scene['live_legs_ns']=now-500_000_001
  with self.assertRaisesRegex(ValueError,'live_legs_ns'):validate_scene_data(scene,cfg,now,'nav','arm')
  scene['live_legs_ns']=now;scene['live_feedback_ns']=now-250_000_001
  with self.assertRaisesRegex(ValueError,'live_feedback_ns'):validate_scene_data(scene,cfg,now,'nav','arm')

 def test_half_second_arm_scene_boundary_and_future_stamp(self):
  cfg={'limits':{'sensor_age_s':1.,'arm_age_s':.5,'legs_age_s':.5},'frames':{'max_state_skew_s':.1}}
  scene=self.make().data();now=2_000_000_000
  scene['body']['mono_ns']=now;scene['feedback_ns']=now;scene['legs_ns']=now;scene['live_legs_ns']=now
  for age in (300_000_000,500_000_000):
   scene['live_feedback_ns']=now-age
   validate_scene_data(scene,cfg,now,'nav','arm')
  for age in (500_000_001,-1):
   scene['live_feedback_ns']=now-age
   with self.assertRaisesRegex(ValueError,'live_feedback_ns'):validate_scene_data(scene,cfg,now,'nav','arm')

 def test_payload_matches_recursive_baseline_including_types(self):
  for live in [None,31]:
   s=self.make(live);expected=asdict(s);expected.update(live_feedback_ns=21 if live is None else live,live_legs_ns=22 if live is None else live,frame='base',world_frame='map',anchor_ns=20)
   self.assertEqual(s.data(),expected);self.assertEqual(s.body.data(),asdict(s.body));self.assertIsInstance(s.data()['q7'],tuple)
 def test_output_mutation_does_not_change_live_body_or_other_payload(self):
  s=self.make();a=s.data();b=s.data();a['body']['map_body'][0][3]=999;a['body']['map_vehicle'][1][3]=123;a['q7']=(999,)*7
  self.assertEqual(s.body.map_body[0][3],0.);self.assertEqual(s.body.map_vehicle[1][3],0.);self.assertEqual(b['body']['map_body'][0][3],0.);self.assertEqual(s.q7[0],0.)
 def test_live_mutation_does_not_rewrite_published_payload(self):
  s=self.make();a=s.data();s.body.map_body[0][3]=123
  self.assertEqual(a['body']['map_body'][0][3],0.)

if __name__=='__main__':unittest.main()
