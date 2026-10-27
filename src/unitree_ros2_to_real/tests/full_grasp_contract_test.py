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
from mission.config import blockers,load

class FullGraspContracts(unittest.TestCase):
    def test_full_preflight_retains_clearance_without_claiming_hardware_cap(self):
        self.assertEqual(CFG['test_mode'],'full')
        self.assertEqual(CFG['grasp_assessment'],dict(contact_torque_nm=1.5,min_grasp_width_m=.05,confirmation_duration_s=2.))
        self.assertEqual(CFG['gripper_torque_control']['contact_torque_nm'],3.2)
        self.assertNotIn('full_grasp_motor_torque_cap_unqualified',blockers(CFG))
        self.assertFalse(CFG['grasp_verification']['motor_torque_cap_validated'])
        self.assertTrue(CFG['execution']['automatic_retreat'])
        self.assertNotIn('physical_retreat_clearance_unqualified',blockers(CFG))
        manual=copy.deepcopy(CFG);manual['execution']['automatic_retreat']=False;manual['operator_trial']=False
        self.assertIn('physical_retreat_clearance_unqualified',blockers(manual))
        diagnostic=copy.deepcopy(CFG);diagnostic['test_mode']='ready_observe'
        self.assertFalse(blockers(diagnostic))

    def test_contact_yaml_and_invalid_settings(self):
        import yaml
        root=Path(__file__).resolve().parents[5]
        control=CFG['gripper_torque_control']
        self.assertEqual((control['kp'],control['kd'],control['close_speed_rad_s']),(20.,1.,.25))
        self.assertGreaterEqual(CFG['grasp_verification']['close_duration_s'],1./control['close_speed_rad_s']+CFG['grasp_assessment']['confirmation_duration_s'])
        original=yaml.safe_load((root/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())
        for key in ('kp','kd','close_speed_rad_s','contact_torque_nm','hold_gain_rad_s_per_nm','hold_deadband_nm','max_target_lead_rad'):
            for value in (0.,-1.,float('nan'),float('inf'),True):
                cfg=copy.deepcopy(original);cfg['go2_mission']['gripper_torque_control'][key]=value
                with tempfile.TemporaryDirectory() as tmp:
                    path=Path(tmp)/'mission.yaml';path.write_text(yaml.safe_dump(cfg))
                    with self.assertRaisesRegex(ValueError,'gripper_torque_control'):
                        load(path,root)
        cfg=copy.deepcopy(original);cfg['go2_mission']['gripper_torque_control']['hold_deadband_nm']=control['contact_torque_nm']
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'mission.yaml';path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError,'deadband'):load(path,root)

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
            import yaml
            raw=yaml.safe_load(Path(context.launch_configurations['mission_config_path']).read_text())
            raw['go2_mission']['execution']['automatic_retreat']=False
            path=prefix/'manual.yaml';path.write_text(yaml.safe_dump(raw))
            context.launch_configurations['mission_config_path']=str(path)
            (lib/'mission').symlink_to(source/'scripts/mission',target_is_directory=True)
            with patch.object(launch,'get_package_prefix',return_value=str(prefix)),patch.object(launch,'get_package_share_directory',return_value=str(source)),patch.object(launch,'Node') as node:
                with self.assertRaisesRegex(RuntimeError,'physical_retreat_clearance_unqualified'):
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

    def test_failed_close_continues_retreat_home_without_retry(self):
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
        self.assertEqual(operations.count('close'),1);self.assertEqual(operations.count('grasp_plan'),1)
        self.assertNotIn('retry_open',operations);self.assertNotIn('reverse_search',operations)
        self.assertEqual(operations[-2: ],['retreat','home'])
        self.assertEqual(operations[-1],'home')

    def test_unknown_never_releases_or_retries(self):
        class Unknown(Fixture):
            def effect(self,kind,data):
                super().effect(kind,data)
                if kind=='trajectory' and data['kind']=='close':self.owner['grasp_assessment']='UNKNOWN'
        f=Unknown();self.assertEqual(f.run(),'BLOCKED')
        self.assertFalse(any(k=='compute' and d['operation'] in ('retreat','reverse_search','retry_open') for k,d in f.effects))

if __name__=='__main__':unittest.main()
