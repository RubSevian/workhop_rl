"""Compare optimized selection with frozen behavior, including rejection details."""
import sys,random,copy,unittest
from pathlib import Path
sys.path[:0]=[str(Path(__file__).parents[1]/'scripts'),str(Path(__file__).parent/'fixtures')]
from mission.scene import SceneBuffer,BodyPose
from scene_snapshot_reference import ReferenceSceneBuffer
from collections import deque
CFG={'limits':{'sensor_age_s':.5,'arm_age_s':.5,'legs_age_s':.5},'frames':{'max_state_skew_s':.05},'execution':{'camera_max_age_s':2.}}
M=[[float(i==j) for j in range(4)] for i in range(4)]
def outcome(s,now,capture):
 try:return ('ok',s.snapshot(now,capture).data(),s.selection)
 except ValueError as e:return ('error',str(e),s.selection)
class Tests(unittest.TestCase):
 def test_histories_and_epochs(self):
  rng=random.Random(371)
  for trial in range(500):
   s=SceneBuffer(copy.deepcopy(CFG));now=2_000_000_000
   for t in sorted(rng.sample(range(900,2100),rng.randrange(1,65))):
    s.bodies.append(BodyPose(t*1000000,t*1000000,'nav'+str(t>1800 and trial%7==0),'tf'+str(t>1900 and trial%11==0),M,M))
   for t in sorted(rng.sample(range(900,2100),rng.randrange(1,100))):s.arm([0.]*7,t*1000000,'owner'+str(t>1800 and trial%5==0))
   for t in sorted(rng.sample(range(900,2100),rng.randrange(1,150))):s.leg([0.]*12,t*1000000)
   if trial%13==0:s.bodies.reverse()
   old=ReferenceSceneBuffer(copy.deepcopy(CFG));old.bodies=copy.deepcopy(s.bodies);old.arms=copy.deepcopy(s.arms);old.legs=copy.deepcopy(s.legs)
   capture=rng.choice([None,now,rng.randrange(900,2100)*1000000])
   self.assertEqual(outcome(s,now,capture),outcome(old,now,capture),trial)
 def test_joint_history_is_traversed_once(self):
  class Counted(deque):
   reads=0
   def __reversed__(self):
    for x in super().__reversed__():self.reads+=1;yield x
  cfg=copy.deepcopy(CFG);cfg['frames']['max_state_skew_s']=0.
  s=SceneBuffer(cfg);now=2_000_000_000
  for i in range(400):
   t=now-400_000_000+i*1_000_000;s.bodies.append(BodyPose(t,t,'nav','tf',M,M))
  for i in range(1000):s.leg([0.]*12,now-499_500_000+i*500_000)
  for i in range(400):s.arm([0.]*7,now-399_750_000+i*1_000_000,'arm')
  s.arms=Counted(s.arms);s.legs=Counted(s.legs)
  outcome(s,now,None)
  self.assertLessEqual(s.arms.reads,len(s.arms));self.assertLessEqual(s.legs.reads,len(s.legs))
if __name__=='__main__':unittest.main()
