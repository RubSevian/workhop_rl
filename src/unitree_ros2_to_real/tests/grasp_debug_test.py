"""Synthetic saved-stage viewer; no model, camera, ROS or robot IO."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from mission.grasp_debug import GraspDebug,STAGES
import grasp_viewer

class DebugTests(unittest.TestCase):
    def sample(self):
        rows=np.zeros((3,17));rows[:,0]=[.9,.7,.5];rows[:,1]=.08;rows[:,3]=.02
        rows[:,4:13]=np.eye(3).ravel();rows[:,13:16]=[[0,0,.5],[.1,0,.5],[.2,0,.5]]
        result={key:rows[:count].copy() for key,count in zip(STAGES.values(),[3,2,0,0,0])}
        result.update(filter_counts={'decoded':3,'after_cloud_collision':2,'after_bbox':0},
            filter_parameters=dict(bbox_xyxy=[20,20,40,40],target_expand_ratio=1.35,target_margin_px=12))
        return result

    def test_snapshot_preserves_stages_and_viewer_renders_empty_filter(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        with tempfile.TemporaryDirectory() as directory:
            for request in (1,2):
                writer=GraspDebug(directory)
                result=self.sample()
                writer.submit(np.zeros((60,80,3),np.uint8),np.full((60,80),500,np.uint16),
                    np.array([[100,0,40],[0,100,30],[0,0,1.]]),result,
                    dict(request_id=request,capture_mono_ns=123,min_depth=.05,max_depth=1.))
                writer.close();self.assertFalse(writer.thread.is_alive());self.assertEqual(writer.error,'')
                data=grasp_viewer.load_snapshot(writer.path)
                self.assertEqual(data['metadata']['request_id'],request)
                np.testing.assert_array_equal(data['rgbd_depth_mm'],np.full((60,80),500,np.uint16))
                self.assertEqual(data['depth'].shape,(0,17))
                for stage,key in STAGES.items():np.testing.assert_array_equal(data[stage],result[key])
            self.assertEqual([p.name for p in Path(directory).iterdir()],['latest.npz'])
            fig=plt.figure();a=fig.add_subplot(121);b=fig.add_subplot(122,projection='3d')
            for stage in STAGES:
                grasp_viewer.draw_snapshot(a,b,data,stage,max_grasps=2)
                fig.canvas.draw()
                self.assertIn(f'{len(data[stage])} candidates',a.get_title())
            plt.close(fig)

    def test_missing_stages_and_bad_directory_are_diagnostic_only(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=GraspDebug(directory)
            writer.submit(np.zeros((2,2,3)),np.zeros((2,2)),np.eye(3),{}, {})
            writer.close();self.assertTrue(writer.error);self.assertFalse(writer.path.exists())
            blocked=Path(directory)/'file';blocked.write_text('file')
            writer=GraspDebug(blocked);writer.close();self.assertTrue(writer.error)

if __name__=='__main__':unittest.main()
