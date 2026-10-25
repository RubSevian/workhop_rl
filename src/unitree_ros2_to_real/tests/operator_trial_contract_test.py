"""Staged physical-trial software contracts only; no drives, ROS graph or SDK."""
import copy,importlib.util,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture,CFG,ROOT,SCRIPTS
from mission.config import apply_operator_trial,blockers
from mission.core import TERMINAL
spec=importlib.util.spec_from_file_location('physical_trial',SCRIPTS/'physical_trial.py')
helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)

class TrialFixture(Fixture):
    def __init__(self):
        super().__init__();self.m.cfg["execution"]["automatic_retreat"]=False;apply_operator_trial(self.m.cfg,True);self.retreat=None
    def effect(self,kind,data):
        if kind=='trajectory' and data.get('operation')=='retreat':self.retreat=copy.deepcopy(data);return
        super().effect(kind,data)
    def waiting(self):
        self.start()
        for _ in range(2000):
            self.tick()
            if self.retreat is not None:return
            assert self.m.state not in TERMINAL,self.m.reason
        raise AssertionError('did not stage retreat')

class TrialTests(unittest.TestCase):
    def test_default_admission_unchanged_and_trial_not_qualification(self):
        cfg=copy.deepcopy(CFG);cfg['execution']['automatic_retreat']=False;self.assertIn('physical_retreat_clearance_unqualified',blockers(cfg))
        before=copy.deepcopy(cfg['collision']);apply_operator_trial(cfg,True)
        self.assertNotIn('physical_retreat_clearance_unqualified',blockers(cfg));self.assertEqual(cfg['collision'],before)
        self.assertFalse(cfg['collision']['retreat_physical_clearance_validated'])
        cfg['frames']['vehicle_to_mount']=None;self.assertIn('vehicle_to_mount_missing',blockers(cfg))
        cfg['test_mode']='ready_observe'
        with self.assertRaisesRegex(ValueError,'requires test_mode=full'):apply_operator_trial(cfg,True)
        with self.assertRaisesRegex(ValueError,'boolean'):apply_operator_trial(cfg,'true')

    def test_automatic_cycle_finishes_without_approval_for_both_outcomes(self):
        class AutomaticFixture(Fixture):
            def effect(self,kind,data):
                super().effect(kind,data)
                if kind=='trajectory' and data['kind']=='close':
                    self.owner['grasp_assessment']=self.assessment
        for trial in (False,True):
            for assessment,terminal in (('GRASPED','GRASP_PRESUMED_SUCCESS'),('NOT_GRASPED','TARGET_NOT_GRASPED')):
                with self.subTest(trial=trial,assessment=assessment):
                    f=AutomaticFixture();f.assessment=assessment
                    f.m.cfg['execution']['automatic_retreat']=True
                    apply_operator_trial(f.m.cfg,trial)
                    self.assertNotIn('physical_retreat_clearance_unqualified',blockers(f.m.cfg))
                    self.assertFalse(f.m.cfg['collision']['retreat_physical_clearance_validated'])
                    f.start()
                    for _ in range(2000):
                        f.tick()
                        self.assertNotEqual(f.m.state,'WAIT_RETREAT_APPROVAL')
                        if f.m.state in TERMINAL:break
                    self.assertEqual(f.m.state,terminal,f.m.reason)
                    operations=[d['operation'] for k,d in f.effects if k=='trajectory']
                    self.assertEqual(operations[-3:],['close','retreat','home'])
                    self.assertTrue(f.owner['gripper_command_active'])

    def test_full_flow_waits_without_motion_then_approved_owner_can_finish(self):
        f=TrialFixture();f.waiting();self.assertEqual(f.m.state,'WAIT_RETREAT_APPROVAL')
        start=copy.deepcopy(f.owner['q']);commands=f.commands
        for _ in range(250):f.tick()
        self.assertEqual(f.m.state,'WAIT_RETREAT_APPROVAL');self.assertEqual(f.owner['q'],start);self.assertEqual(f.commands,commands)
        self.assertTrue(f.m.status(f.now)['retreat_operator_approval_required'])
        # Fake owner acknowledgment only; the native one-shot gate has separate tests.
        f.owner.update(mission_id=f.retreat['id'],mission_state='EXECUTING');f.tick()
        self.assertEqual(f.m.state,'RETREAT')
        f.owner.update(q=f.retreat['points'][-1],mission_state='SUCCEEDED')
        for _ in range(30):
            f.tick()
            if f.m.state in TERMINAL:break
        self.assertEqual(f.m.state,'GRASP_PRESUMED_SUCCESS')

    def test_pause_preserves_all_fault_session_feedback_and_timeout_checks(self):
        for failure in ('X','B','owner_epoch','nav_epoch','feedback','scene','timeout','assessment'):
            with self.subTest(failure=failure):
                f=TrialFixture();f.waiting()
                if failure=='X':f.r3.update(state='CONTROLLED_STOP',phase='CANCEL_TASKS')
                elif failure=='B':f.r3.update(state='EMERGENCY_FAULT',fault_latched=True)
                elif failure=='owner_epoch':f.owner['owner_session_id']='new-owner'
                elif failure=='nav_epoch':f.nav['session_id']='new-nav'
                elif failure=='feedback':f.owner['feedback_ready']=False
                elif failure=='scene':f.tf_revision='new-tf'
                elif failure=='timeout':
                    f.m.cfg['execution']['state_timeout_s']=5.
                    f.m.entered=f.now-6.
                elif failure=='assessment':
                    # The owner independently requires valid assessment to approve; a mere
                    # pending plan never grants motion on UNKNOWN feedback.
                    owner=dict(operator_trial=True,owner_session_id='a',feedback_ready=True,target_valid=True,grasp_assessment='UNKNOWN',retreat_trial=dict(state='WAITING_OPERATOR',approval_service='/rars01/mission/approve_retreat/ta',plan_id=1,mission_session_id='m',goal_revision=1))
                    with self.assertRaises(ValueError):helper.approval_identity(owner,dict(operator_trial=True,state='WAIT_RETREAT_APPROVAL',session_id='m',goal_revision=1),dict(state='ACTIVE',phase='RL_ZERO'))
                    continue
                f.tick();self.assertIn(f.m.state,TERMINAL);self.assertNotEqual(f.m.state,'GRASP_PRESUMED_SUCCESS')

    def test_cli_requires_exact_waiting_identity_not_early_stale_or_foreign_ticket(self):
        owner=dict(operator_trial=True,owner_session_id='owner',feedback_ready=True,target_valid=True,grasp_assessment='GRASPED',gripper_command_active=True,retreat_trial=dict(state='WAITING_OPERATOR',approval_service='/rars01/mission/approve_retreat/t123',plan_id=1,mission_session_id='mission',goal_revision=2))
        mission=dict(operator_trial=True,state='WAIT_RETREAT_APPROVAL',session_id='mission',goal_revision=2)
        r3=dict(state='ACTIVE',phase='RL_ZERO',fault_latched=False)
        identity=helper.approval_identity(owner,mission,r3);self.assertEqual(identity[3],1)
        self.assertEqual(helper.approval_identity(dict(owner,grasp_assessment='NOT_GRASPED'),mission,r3),identity)
        for target,key,value in [('owner','gripper_command_active',False),('owner','feedback_ready',False),('owner','grasp_assessment','UNKNOWN'),('mission','session_id','foreign'),('mission','goal_revision',3),('mission','state','RETREAT'),('r3','phase','RL_ACTIVE'),('r3','fault_latched',True)]:
            args=dict(owner=copy.deepcopy(owner),mission=copy.deepcopy(mission),r3=copy.deepcopy(r3));args[target][key]=value
            with self.assertRaises(ValueError):helper.approval_identity(**args)

    def test_descent_evidence_requires_successful_verify_passive_and_output_off(self):
        r3=dict(state='SYSTEM_HOLD',phase='LIE_DOWN_HOLD',output_enabled=False,output_stop_confirmed=True,passive_sequence_complete=True,fault_latched=False)
        self.assertTrue(helper.descent_record_allowed(r3))
        for key,value in [('state','CONTROLLED_STOP'),('phase','LIE_DOWN_BLOCKED'),('output_enabled',True),('output_stop_confirmed',False),('passive_sequence_complete',False),('fault_latched',True)]:
            self.assertFalse(helper.descent_record_allowed(dict(r3,**{key:value})))

    def test_launch_propagates_trial_and_existing_descent_override(self):
        from launch import LaunchContext
        from launch_ros.actions import Node
        import launch.logging
        spec=importlib.util.spec_from_file_location('trial_launch',SCRIPTS.parent/'launch/go2_rars01_first_grasp.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp:
            launch.logging.launch_config.log_dir=tmp;p=Path(tmp);lib=p/'lib/unitree_legged_real';lib.mkdir(parents=True);(lib/'mission').symlink_to(SCRIPTS/'mission',target_is_directory=True)
            c=LaunchContext();c.launch_configurations.update(workspace=str(ROOT),mission_config_path=str(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'),physical='true',test_mode='full',operator_trial='false',start_arm_owner='true',network_interface='',controlled_stop_lie_down_trial='true',architecture_monitor='true')
            with patch.object(module,'get_package_prefix',return_value=str(p)),patch.object(module,'get_package_share_directory',return_value=str(SCRIPTS.parent)):
                self.assertEqual(len(module._launch(c)),3) # Explicit automatic cycle in working YAML
                c.launch_configurations['operator_trial']='true';actions=module._launch(c)
                self.assertEqual(len(actions),3) # monitor + preload release handler + perception
                # Only action construction; no action execution/hardware.
                self.assertTrue(any(isinstance(a,Node) and 'grasp_perception.py' in str(a.node_executable) for a in actions))
        owner=(SCRIPTS.parent/'src/rars_r3_owner.cpp').read_text()
        self.assertIn('Authority(now,is_home)',owner);self.assertIn('trial_approval_.Consume',owner)
        self.assertIn('mission_->Start',owner);self.assertIn('RevokeRetreatTrial("EMERGENCY")',owner)
        self.assertIn('SlowRetreat(result.points,max_plan_samples_)',owner)

if __name__=='__main__':unittest.main()
