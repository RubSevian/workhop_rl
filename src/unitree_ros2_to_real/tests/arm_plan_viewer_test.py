"""Planning display tests: no ROS, camera or motor access."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from mission.arm_debug import ArmDebug,planning_snapshot
from arm_plan_viewer import arm_fk,draw_arm_snapshot,load_arm_snapshot

class ArmViewerTests(unittest.TestCase):
    def fixture(self,directory):
        urdf=Path(directory)/'arm.urdf'
        urdf.write_text('''<robot name="test"><link name="base_link"/><link name="a"/><link name="End_link"/>
          <joint name="joint1" type="revolute"><parent link="base_link"/><child link="a"/>
          <origin xyz="0 0 0"/><axis xyz="0 0 1"/></joint>
          <joint name="tcp" type="fixed"><parent link="a"/><child link="End_link"/><origin xyz="1 0 0"/></joint></robot>''')
        candidate=dict(candidate='rank=1 flip=0',pregrasp_End_link_pose6d=[.92,0,0,0,0,0],
                       grasp_End_link_pose6d=[1,0,0,0,0,0],translation_camera_m=[.97,0,0],
                       rotation_camera=np.eye(3).tolist(),width_m=.08,depth_m=.03,status='rejected')
        calls=[dict(candidate=candidate['candidate'],stage='pregrasp',success=False,
                    joints=[0.]*6,position_error_m=.08,orientation_error_rad=0,attempts=['not copied'])]
        return planning_snapshot({'id':12,'session_id':'session'},urdf,np.zeros(6),np.eye(4),
                                 {'orientations':[candidate]},calls,'NO_EXECUTABLE_CANDIDATE')

    def test_fk_follows_urdf_axis_and_fixed_tcp(self):
        with tempfile.TemporaryDirectory() as directory:
            data=self.fixture(directory)
            points,t=arm_fk(data['urdf'],[np.pi/2,0,0,0,0,0])
            np.testing.assert_allclose(t[:3,3],[0,1,0],atol=1e-12)
            np.testing.assert_allclose(points[-1],t[:3,3])

    def test_atomic_latest_snapshot_and_detached_input(self):
        with tempfile.TemporaryDirectory() as directory:
            data=self.fixture(directory);writer=ArmDebug(directory)
            writer.submit(data);data['current_q'][0]=99;writer.close()
            self.assertEqual(writer.error,'');self.assertFalse(writer.thread.is_alive())
            loaded=load_arm_snapshot(writer.path)
            self.assertEqual(loaded['request_id'],12)
            self.assertEqual(loaded['current_q'][0],0)
            self.assertNotIn('attempts',loaded['ik'][0])
            self.assertFalse(writer.path.with_suffix('.tmp').exists())

    def test_failed_ik_is_not_labelled_as_approved(self):
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        with tempfile.TemporaryDirectory() as directory:
            data=self.fixture(directory);fig=Figure();FigureCanvasAgg(fig);ax=fig.add_subplot(projection='3d')
            draw_arm_snapshot(ax,data,'rank=1 flip=0');fig.canvas.draw()
            self.assertIn('not approved',ax.get_title())
            self.assertTrue(any('FAILED IK' in x.get_label() for x in ax.lines))
            self.assertTrue(any('8.0 cm' in x.get_label() for x in ax.lines))
            data['candidates']=[];data['ik']=[]
            draw_arm_snapshot(ax,data,'');fig.canvas.draw()
            self.assertIn('Waiting for candidates',ax.get_title())

if __name__=='__main__':unittest.main()
