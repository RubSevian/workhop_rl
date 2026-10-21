"""Offline mathematical envelope and real-model conflict regressions, no IO."""
import copy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from gripper_sweep_analysis import sweep_points,enclosing_box,SweepAnalysis,ROOT


class BoundsTests(unittest.TestCase):
    def test_encloses_continuous_translation_not_just_endpoint_poses(self):
        vertices=np.array([[-.02,.01,.03],[.04,-.02,-.01],[.01,.05,.02],[-.01,-.01,-.02]])
        lo=np.eye(4);lo[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];lo[:3,3]=[.1,-.2,.3]
        hi=lo.copy();hi[:3,3]+=[.03,.05,-.02]
        points=sweep_points(vertices,lo,hi);center,half=enclosing_box(points)
        for t in np.linspace(0,1,101):
            at=vertices@lo[:3,:3].T+(1-t)*lo[:3,3]+t*hi[:3,3]
            self.assertTrue(np.all(np.abs(at-center)<=half+16*np.finfo(float).eps))
        self.assertTrue(np.all(np.abs(points-center)<=half))

    def test_invalid_or_rotating_models_are_not_certified(self):
        hi=np.eye(4);hi[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        with self.assertRaisesRegex(ValueError,'NOT_TRANSLATIONAL'):sweep_points([[0,0,0]],np.eye(4),hi)
        for points in ([],[[float('nan'),0,0]],[[1,2]]):
            with self.assertRaises(ValueError):enclosing_box(points)


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from mission.config import load
        cls.cfg=load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)
        cls.analysis=SweepAnalysis(cls.cfg)

    def test_two_full_urdf_strokes_with_no_jaw_exclusion(self):
        a=self.analysis
        self.assertEqual({m['jaw'] for m in a.metadata},{'gripper_left','gripper_right'})
        self.assertTrue(all(m['range_m']==[0.,.05] for m in a.metadata))
        for meta in a.metadata:
            self.assertGreater(meta['endpoint_vertex_count'],0)
            self.assertTrue(np.all(np.array(meta['upper_End_link_m'])>meta['lower_End_link_m']))

    def test_cached_envelopes_follow_end_link_and_conflicts_stay_blocked(self):
        a=self.analysis;ids=[int(shape.id()) for _,_,shape,_ in a.volumes]
        metadata=copy.deepcopy(a.metadata)
        home=a.attachment_conflicts([0.]*6)
        ready=a.attachment_conflicts([-.0011,.972,1.3737,-1.1017,-.0009,-.0007])
        self.assertEqual(len(home),4);self.assertEqual(len(ready),4)
        for row in home+ready:
            self.assertTrue(row['blocked']);self.assertLess(row['distance_m'],0)
        self.assertEqual(ids,[int(shape.id()) for _,_,shape,_ in a.volumes])
        self.assertEqual(metadata,a.metadata)
        for before,after in zip(home,ready):
            self.assertAlmostEqual(before['distance_m'],after['distance_m'],places=12)


if __name__=='__main__':unittest.main()
