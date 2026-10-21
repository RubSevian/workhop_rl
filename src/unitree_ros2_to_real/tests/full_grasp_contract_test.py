"""Full mission software contracts; no hardware, ROS graph, camera or SDK IO."""
import copy,os,tempfile
_launch_logs=tempfile.TemporaryDirectory(prefix="go2-full-launch-test-")
os.environ["ROS_LOG_DIR"]=_launch_logs.name
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture,CFG
import architecture_mission_contract_test as architecture
from mission.approved_motion import release_for_retry
from mission.config import blockers

class FullGraspContracts(unittest.TestCase):
    def test_full_preflight_retains_both_physical_gates(self):
        self.assertEqual(CFG['test_mode'],'full')
        self.assertEqual(CFG['grasp_assessment'],dict(contact_torque_nm=1.5,min_grasp_width_m=.05,confirmation_duration_s=.5))
        self.assertEqual(CFG['grasp_verification']['max_total_torque_nm'],2.2)
        self.assertIn('full_grasp_motor_torque_cap_unqualified',blockers(CFG))
        self.assertIn('physical_retreat_clearance_unqualified',blockers(CFG))
        diagnostic=copy.deepcopy(CFG);diagnostic['test_mode']='ready_observe'
        self.assertFalse(blockers(diagnostic))

    def test_full_physical_launch_refuses_before_any_node_construction(self):
        import importlib.util,tempfile
        from unittest.mock import patch
        from launch import LaunchContext
        root=Path(__file__).resolve().parents[5]
        source=Path(__file__).resolve().parents[1]
        spec=importlib.util.spec_from_file_location('full_preflight_launch',source/'launch/go2_rars01_first_grasp.launch.py')
        launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
        context=LaunchContext();context.launch_configurations.update(workspace=str(root),physical='true',test_mode='full',start_arm_owner='true',network_interface='',mission_config_path=str(root/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'))
        with tempfile.TemporaryDirectory() as tmp:
            prefix=Path(tmp);lib=prefix/'lib/unitree_legged_real';lib.mkdir(parents=True)
            (lib/'mission').symlink_to(source/'scripts/mission',target_is_directory=True)
            with patch.object(launch,'get_package_prefix',return_value=str(prefix)),patch.object(launch,'get_package_share_directory',return_value=str(source)),patch.object(launch,'Node') as node:
                with self.assertRaisesRegex(RuntimeError,'full_grasp_motor_torque_cap_unqualified.*physical_retreat_clearance_unqualified'):
                    launch._launch(context)
                node.assert_not_called()

    def test_release_uses_cached_approval_and_only_changes_gripper(self):
        segment,current=architecture.ArchitectureMissionTests().reverse_fixture()
        segment['collision_gripper_reference']={'model':'open_gripper_approximation'}
        for point in segment['points']:point[6]=1.
        current['q7'][6]=.4
        out=release_for_retry(segment,current,CFG,open_angle=1.,rate=100.,velocity=2.,assessment='NOT_GRASPED')
        self.assertEqual(out['kind'],'release');self.assertTrue(out['reused_approval'])
        self.assertFalse(out['physical_geometry_qualified'])
        np.testing.assert_allclose(np.array(out['points'])[:,:6],np.repeat(np.array(current['q7'][:6])[None,:],len(out['points']),axis=0))
        self.assertAlmostEqual(out['points'][-1][6],1.)
        self.assertLessEqual(float(np.max(np.abs(np.diff(np.array(out['points'])[:,6]))))*100.,2.+1e-6)
        for state in ('UNKNOWN','GRASPED',None):
            with self.assertRaisesRegex(ValueError,'ASSESSMENT_UNKNOWN'):
                release_for_retry(segment,current,CFG,open_angle=1.,rate=100.,velocity=2.,assessment=state)

    def test_two_failed_closes_open_before_saved_reverse_then_home(self):
        class FailedCloses(Fixture):
            def effect(self,kind,data):
                super().effect(kind,data)
                if kind=='trajectory' and data['kind']=='close':self.owner['grasp_assessment']='NOT_GRASPED'
                if kind=='trajectory' and data['kind']=='release':self.owner.update(gripper_command_active=False)
            def compute(self,data):
                if data['operation']=='retry_open':
                    result={k:data[k] for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')}
                    end=data['owner']['q'].copy();end[6]=.8
                    result.update(ok=True,observed_ns=int(self.now*1e9),trajectory=dict(kind='release',points=[data['owner']['q'],end],collision_checked=True,payload_checked=False))
                    return result
                return super().compute(data)
        f=FailedCloses();self.assertEqual(f.run(),'TARGET_NOT_GRASPED')
        operations=[d['operation'] for kind,d in f.effects if kind=='compute']
        self.assertEqual(operations.count('close'),2);self.assertEqual(operations.count('grasp_plan'),2)
        self.assertEqual(operations.count('retry_open'),1)
        self.assertLess(operations.index('retry_open'),operations.index('reverse_search'))
        self.assertEqual(operations[-1],'home')

    def test_unknown_never_releases_or_retries(self):
        class Unknown(Fixture):
            def effect(self,kind,data):
                super().effect(kind,data)
                if kind=='trajectory' and data['kind']=='close':self.owner['grasp_assessment']='UNKNOWN'
        f=Unknown();self.assertEqual(f.run(),'BLOCKED')
        self.assertFalse(any(k=='compute' and d['operation'] in ('retreat','reverse_search','retry_open') for k,d in f.effects))

if __name__=='__main__':unittest.main()
