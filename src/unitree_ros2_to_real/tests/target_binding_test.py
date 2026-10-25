"""Recorded RGB-D replay and input contracts; no inference, camera or robot."""
import sys,json,unittest
from pathlib import Path
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts';ROOT=Path(__file__).resolve().parents[5]
sys.path[:0]=[str(SCRIPTS),str(ROOT/'repos/rars01_graspnet')]
from mission.target_binding import filter_target_binding,validate_config
from utils.yolo_utils import detection_mask
from types import SimpleNamespace as NS

class TargetBindingTests(unittest.TestCase):
    def test_config_loads_as_standalone_launch_module(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('standalone_mission_config',SCRIPTS/'mission/config.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        cfg=module.load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)
        self.assertTrue(cfg['target_binding']['enabled'])
        self.assertEqual(cfg['target_binding']['min_target_fraction'],.5)

    def test_saved_frames_reproduce_offline_decisions_and_flip(self):
        for name,expected in [('banana',[38,60,165]),('cup',[47,177])]:
            path=ROOT/'docs/figures/target_binding_20261025'
            report=json.loads((path/(name+'_report.json')).read_text())
            z=np.load(report['mask']['source']);mask=np.load(path/(name+'_mask.npz'))['mask']
            def run(gs):return filter_target_binding(gs,z['rgbd_depth_mm'],z['K'],mask,{'enabled':True},mask_source='segmentation',min_depth=.05,max_depth=1.)
            kept,stats=run(z['depth']);indices=[int(np.argmin(np.max(abs(z['decoded']-g),axis=1))) for g in kept]
            self.assertEqual(indices,expected)
            if name=='banana':self.assertEqual(stats['candidates'][-1]['target_voxels'],0)
            flipped=z['depth'].copy()
            flipped[:,4:13]=(flipped[:,4:13].reshape(-1,3,3)@np.diag([1,-1,-1])).reshape(-1,9)
            _,other=run(flipped);self.assertEqual(stats['candidates'],other['candidates'])
    def test_config_rejects_invalid_values(self):
        for key,value in [('enabled','true'),('finger_length_m',0),('voxel_size_m',float('nan')),('min_target_voxels',True),('min_target_fraction',1.1)]:
            with self.assertRaises(ValueError):validate_config({key:value})
    def test_no_mask_or_bbox_fallback_cannot_grant_membership(self):
        g=np.zeros((1,17));g[:,4:13]=np.eye(3).ravel();g[:,15]=.5
        for source,mask in [('bbox',np.ones((2,2))),('segmentation',None),('segmentation',np.zeros((2,2))),('segmentation',np.ones((3,3)))]:
            with self.assertRaises(ValueError):filter_target_binding(g,np.full((2,2),500),np.eye(3),mask,{'enabled':True},mask_source=source,min_depth=.05,max_depth=1.)
        _,disabled=filter_target_binding(g,None,None,None,{'enabled':False},mask_source='unknown',min_depth=.05,max_depth=1.)
        self.assertFalse(disabled['enabled'])
    def test_detector_marks_rectangle_fallback(self):
        mask,source=detection_mask(NS(masks=None),0,(4,4),(0,0,3,3),with_source=True)
        self.assertEqual(source,'bbox');self.assertTrue(mask.all())
        mask,source=detection_mask(NS(masks=NS(data=np.eye(4)[None])),0,(4,4),(0,0,3,3),with_source=True)
        self.assertEqual(source,'segmentation');self.assertEqual(mask.sum(),4)
    def test_invalid_depth_yields_no_support_and_cancellation_propagates(self):
        g=np.zeros((1,17));g[:,4:13]=np.eye(3).ravel();g[:,15]=.5
        args=(g,np.zeros((2,2)),np.eye(3),np.ones((2,2)),{'enabled':True})
        kept,stats=filter_target_binding(*args,mask_source='segmentation',min_depth=.05,max_depth=1.)
        self.assertEqual(len(kept),0);self.assertEqual(stats['candidates'][0]['reason'],'INSUFFICIENT_TARGET')
        def cancel():raise RuntimeError('cancelled')
        with self.assertRaisesRegex(RuntimeError,'cancelled'):filter_target_binding(*args,mask_source='segmentation',min_depth=.05,max_depth=1.,cancel_check=cancel)

if __name__=='__main__':unittest.main()
