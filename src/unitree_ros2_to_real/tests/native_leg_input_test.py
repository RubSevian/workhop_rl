from pathlib import Path
from types import SimpleNamespace as NS
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.native_leg_input import NativeLegInput
from mission.scene import SceneBuffer, BodyPose

class Tests(unittest.TestCase):
    def setUp(self):
        self.cfg={'frames':{'odometry_history_s':1.,'leg_collector_capacity':512}}
        self.scene=SceneBuffer(self.cfg);self.reader=NativeLegInput(self.scene,self.cfg)
    def batch(self,stamps,session='native1',published=1_000_000_000):
        return NS(observed_ns=published,source_session_id=session,samples_received=len(stamps),samples_dropped=0,
                  samples=[NS(observed_ns=t,robot_tick=i,q=[float(i)]*12) for i,t in enumerate(stamps)])
    def test_delayed_batch_preserves_every_original_stamp_and_angle(self):
        stamps=[800_000_000+i*2_000_000 for i in range(100)]
        last=self.reader.apply(self.batch(stamps),1_200_000_000)
        self.assertEqual([v[0] for v in self.scene.legs],stamps)
        self.assertEqual(last,(stamps[-1],tuple([99.]*12)))
        self.assertEqual(self.scene.legs[0][0],800_000_000)
    def test_delayed_delivery_still_matches_historical_body_without_retiming(self):
        self.cfg.update(limits={'arm_age_s':.25,'sensor_age_s':.5},execution={'camera_max_age_s':.6})
        self.cfg['frames']['max_state_skew_s']=.05
        stamps=[800_000_000+i*2_000_000 for i in range(100)]
        self.reader.apply(self.batch(stamps),1_200_000_000)
        identity=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]
        self.scene.bodies.append(BodyPose(900_000_000,900_000_000,'nav','tf',identity,identity))
        self.scene.arm([0.]*7,900_000_000,'arm');self.scene.arm([0.]*7,1_200_000_000,'arm')
        selected=self.scene.snapshot(1_200_000_000)
        self.assertEqual(selected.legs_ns,900_000_000)
        self.assertEqual(selected.legs,tuple([50.]*12))

    def test_invalid_batch_is_atomic(self):
        for bad in [self.batch([10,9]),self.batch([10,1_000_000_001]),self.batch([0]),self.batch([1]*513)]:
            with self.assertRaises(ValueError):self.reader.apply(bad,1_100_000_000)
            self.assertFalse(self.scene.legs)
        bad=self.batch([10,20]);bad.samples[-1].q[0]=float('nan')
        with self.assertRaises(ValueError):self.reader.apply(bad,1_100_000_000)
        self.assertFalse(self.scene.legs)
    def test_epoch_change_clears_old_history_and_late_old_batch_cannot_return(self):
        old=self.batch([900_000_000]);self.reader.apply(old,1_100_000_000)
        new=self.batch([1_100_000_000],session='native2',published=1_200_000_000)
        self.reader.apply(new,1_300_000_000);self.assertEqual(len(self.scene.legs),1)
        self.assertIsNone(self.reader.apply(old,1_300_000_000));self.assertEqual(self.reader.session,'native2')
    def test_expired_history_is_not_made_fresh(self):
        self.assertIsNone(self.reader.apply(self.batch([900_000_000]),3_000_000_000))
        self.assertFalse(self.scene.legs)
    def test_future_envelope_rejected(self):
        with self.assertRaises(ValueError):self.reader.apply(self.batch([900_000_000]),999_000_000)
if __name__=='__main__':unittest.main()
