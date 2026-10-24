#!/usr/bin/env python3
"""Synthetic diagnostic transitions/journaling/launch: no motor or SDK IO."""
import copy
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from mission.operator_events import OperatorEvents,render_event
spec=importlib.util.spec_from_file_location('architecture_monitor',SCRIPTS/'architecture_monitor.py')
monitor=importlib.util.module_from_spec(spec);spec.loader.exec_module(monitor)

class MonitorTests(unittest.TestCase):
    def test_thirty_seconds_steady_rl_zero_is_silent(self):
        m=OperatorEvents();s=dict(state='ACTIVE',phase='RL_ZERO',operation_profile='full_mission',
            phase_warnings=[],blockers=[],arm_ready=True,navigation_ready=True,perception_ready=True)
        self.assertIsNotNone(m.observe('r3',s,0,0))
        for tick in range(1,3001):
            s.update(observed_ns=tick,policy_ms=tick/100,policy_result_age_s=tick/1e6,deadline_misses=tick,
                motion_command=[tick/100,0,0],measured_motor_q=[tick/100]*12)
            self.assertIsNone(m.observe('r3',s,tick,tick))
        s['phase']='RL_ACTIVE';event=m.observe('r3',s,3001,3001)
        self.assertEqual(event['changes'],{'phase':{'from':'RL_ZERO','to':'RL_ACTIVE'}})
        self.assertIn('"RL_ZERO" -> "RL_ACTIVE"',render_event(event))
        self.assertIsNone(m.observe('r3',s,3002,3002))

    def test_modes_and_readiness_in_all_five_components(self):
        cases={'r3':('operation_profile','full_mission','remote_test','arm_ready'),
               'nav':('input_mode','stack','external','navigation_ready'),
               'mission':('test_mode','full','ready_observe','ready'),
               'perception':('camera_mode','standby','active','ready'),
               'arm':('grasp_verification','stub','torque_width','feedback_ready')}
        for name,(key,old,new,ready) in cases.items():
            with self.subTest(name=name):
                m=OperatorEvents();s={key:old,ready:False};m.observe(name,s,1,1)
                s[key]=new;e=m.observe(name,s,2,2);self.assertEqual(e['changes'][key],{'from':old,'to':new})
                s[ready]=True;e=m.observe(name,s,3,3);self.assertIn(ready,e['changes'])
                self.assertIn(ready,render_event(e));self.assertIsNone(m.observe(name,s,4,4))

    def test_nested_diagnostics_and_free_text_age_do_not_repeat(self):
        m=OperatorEvents();s=dict(navigation_ready=False,block_reason='localization_unavailable',
            tf_pair=dict(reason='ODOMETRY_MISSING_OR_STALE',age_s=.7,stamp_ns=100,frames=1),
            goal=dict(revision=1,state='ACTIVE',elapsed_s=1,error_xy_m=.3))
        m.observe('nav',s,1,1)
        for tick in range(100):
            s['tf_pair'].update(age_s=tick,stamp_ns=tick,frames=tick)
            s['goal'].update(elapsed_s=tick,error_xy_m=tick/10)
            self.assertIsNone(m.observe('nav',s,tick+2,tick+2))
        s['tf_pair']['reason']='TF_SESSION_MISMATCH';self.assertIsNotNone(m.observe('nav',s,200,200))
        a=dict(state='BLOCKED',reason='SCENE_STALE:arm age_ms=601.5 frames=123 motor7')
        m.observe('mission',a,1,1);a['reason']='SCENE_STALE:arm age_ms=650.0 frames=124 motor7'
        self.assertIsNone(m.observe('mission',a,2,2))
        a['reason']='SCENE_STALE:legs age_ms=650.0 frames=124 motor7';self.assertIsNotNone(m.observe('mission',a,3,3))

    def test_blocker_order_and_numeric_details_are_not_events(self):
        m=OperatorEvents();s=dict(state='ACTIVE',phase='RL_ZERO',phase_warnings=['NAV_UNAVAILABLE','ARM_UNAVAILABLE'],
            blockers=[dict(code='LOWSTATE_STALE',source='/lowstate',age_ms=601,frame_count=20)])
        e=m.observe('r3',s,1,1);self.assertEqual(e['level'],'BLOCKED')
        s['phase_warnings'].reverse();s['blockers'][0].update(age_ms=700,frame_count=21)
        self.assertIsNone(m.observe('r3',s,2,2))
        s['blockers'][0]['source']='/rars01/commissioning/state';self.assertIsNotNone(m.observe('r3',s,3,3))

    def test_fault_warning_block_and_recovery_exactly_once(self):
        for name,healthy,bad in [
            ('r3',dict(state='ACTIVE',phase='RL_ZERO',fault_latched=False,fault='',phase_warnings=[]),dict(state='EMERGENCY_FAULT',fault_latched=True,fault='policy_result_stale')),
            ('nav',dict(navigation_ready=True,block_reason=''),dict(navigation_ready=True,block_reason='path_unavailable')),
            ('perception',dict(ready=True,reason='READY',scene_error=''),dict(ready=False,reason='SCENE_SNAPSHOT_STALE:live_arm',scene_error='SCENE_SNAPSHOT_STALE:live_arm')),
            ('arm',dict(owner_state='HOLD_HOME',feedback_ready=True,last_error=''),dict(owner_state='FAULT_LATCHED',feedback_ready=False,last_error='runtime_watchdog')),
            ('mission',dict(state='WAIT_GOAL',reason='',availability=dict(navigation=True,manipulation=True)),dict(state='WAIT_GOAL',reason='ARM_UNAVAILABLE',availability=dict(navigation=True,manipulation=False)))]:
            with self.subTest(name=name):
                m=OperatorEvents();m.observe(name,healthy,1,1);e=m.observe(name,dict(healthy,**bad),2,2)
                self.assertNotEqual(e['level'],'OK');self.assertTrue(e['cause']);self.assertIn(name.upper(),render_event(e))
                self.assertIsNone(m.observe(name,dict(healthy,**bad),3,3))
                e=m.observe(name,healthy,4,4);self.assertEqual(e['event_type'],'RECOVERED')
                self.assertIsNone(m.observe(name,healthy,5,5))

    def test_operation_lifecycle_ids_and_goal_results(self):
        m=OperatorEvents();s=dict(state='GRASP',pending=None);m.observe('mission',s,1,1)
        s['pending']=dict(id=7,kind='motion',operation='close',at=1.2,scene={'age_s':.1})
        e=m.observe('mission',s,2,2);self.assertEqual(e['operations'],[dict(event='STARTED',id=7,kind='motion',operation='close')])
        s['pending']['at']=2.3;self.assertIsNone(m.observe('mission',s,3,3))
        s['pending']=None;e=m.observe('mission',s,4,4);self.assertEqual(e['operations'][0]['event'],'ENDED')
        s['pending']=dict(id=8,kind='motion',operation='close');self.assertIsNotNone(m.observe('mission',s,5,5))
        for name,begin,end in [('arm',dict(mission_id=1,mission_state='EXECUTING'),dict(mission_id=1,mission_state='SUCCEEDED')),
                               ('perception',dict(busy=True),dict(busy=False,last_compute=dict(operation='grasp_plan',elapsed_s=5)) )]:
            self.assertEqual(m.observe(name,begin,6,6)['operations'][0]['event'],'STARTED')
            self.assertEqual(m.observe(name,end,7,7)['operations'][0]['event'],'ENDED')
            self.assertIsNone(m.observe(name,end,8,8))
        a=dict(navigation_ready=True,goal=dict(revision=1,state='ACTIVE',elapsed_s=1))
        m.observe('nav',a,1,1);a['goal']['state']='REACHED';self.assertIsNotNone(m.observe('nav',a,2,2))
        a['goal']['revision']=2;self.assertIsNotNone(m.observe('nav',a,3,3))

    def test_every_suppressed_snapshot_is_journaled_with_full_details(self):
        journal=io.StringIO();terminal=io.StringIO();sink=monitor.MonitorOutput(journal,terminal)
        for n in range(301):sink.receive('r3',json.dumps(dict(state='ACTIVE',phase='RL_ZERO',observed_ns=n,policy_ms=n/100)),n,n)
        self.assertEqual(len(terminal.getvalue().splitlines()),1)
        records=[json.loads(line) for line in journal.getvalue().splitlines()]
        self.assertEqual(len(records),301);self.assertEqual(sum(r['event'] is not None for r in records),1)
        self.assertEqual(json.loads(records[-1]['raw'])['policy_ms'],3.)

    def test_decoder_failure_deduplication_and_recovery(self):
        for name,bad in [('nav','not json'),('r3','{bad: [')]:
            with self.subTest(name=name):
                journal=io.StringIO();terminal=io.StringIO();sink=monitor.MonitorOutput(journal,terminal)
                e=sink.receive(name,bad,1,1);self.assertEqual(e['level'],'BLOCKED')
                self.assertIsNone(sink.receive(name,bad+' ',2,2))
                e=sink.receive(name,'{"ready":true}',3,3);self.assertEqual(e['event_type'],'RECOVERED')
                self.assertEqual(len(journal.getvalue().splitlines()),3);self.assertEqual(len(terminal.getvalue().splitlines()),2)

    def test_nonfinite_wire_measurements_stay_in_raw_valid_jsonl(self):
        journal=io.StringIO();sink=monitor.MonitorOutput(journal,io.StringIO(),format='json')
        raw='{owner_state: HOLD_HOME, feedback_ready: true, q: [.nan], streams: {arm: {age_s: .inf}}}'
        e=sink.receive('arm',raw,1,1)
        parsed=json.loads(journal.getvalue(),parse_constant=lambda x:self.fail('nonstandard JSON '+x))
        self.assertEqual(parsed['raw'],raw);self.assertEqual(e['evidence']['streams']['arm']['age_s'],'inf')

    def test_general_launch_monitor_default_off_explicit_opt_in_and_preflight(self):
        from launch import LaunchContext
        from launch.actions import RegisterEventHandler
        from launch_ros.actions import Node
        import launch.logging
        root=SCRIPTS.parents[4]
        spec=importlib.util.spec_from_file_location('monitor_launch',SCRIPTS.parent/'launch/go2_rars01_first_grasp.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            launch.logging.launch_config.log_dir=folder
            prefix=Path(folder);lib=prefix/'lib/unitree_legged_real';lib.mkdir(parents=True)
            (lib/'mission').symlink_to(SCRIPTS/'mission',target_is_directory=True)
            c=LaunchContext();c.launch_configurations.update(workspace=str(root),physical='false',test_mode='full',start_arm_owner='true',network_interface='',mission_config_path=str(root/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'))
            with patch.object(module,'get_package_prefix',return_value=str(prefix)),patch.object(module,'get_package_share_directory',return_value=str(SCRIPTS.parent)):
                actions=module._launch(c)
                is_monitor=lambda action:isinstance(action,Node) and 'architecture_monitor.py' in str(action.node_executable)
                self.assertFalse(any(map(is_monitor,actions)))
                c.launch_configurations['architecture_monitor']='true'
                self.assertEqual(sum(map(is_monitor,module._launch(c))),1)
                c.launch_configurations['architecture_monitor']='false';self.assertFalse(any(map(is_monitor,module._launch(c))))
                c.launch_configurations.update(architecture_monitor='true',physical='true',test_mode='ready_observe')
                actions=module._launch(c);self.assertTrue(is_monitor(actions[0]));self.assertEqual(sum(map(is_monitor,actions)),1)
                self.assertIsInstance(actions[1],RegisterEventHandler)
                c.launch_configurations['test_mode']='full'
                with patch.object(module,'Node') as node:
                    with self.assertRaisesRegex(RuntimeError,'physical_retreat_clearance_unqualified'):module._launch(c)
                    node.assert_not_called()

if __name__=='__main__':unittest.main()
