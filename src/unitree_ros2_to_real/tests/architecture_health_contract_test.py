"""v4 status TTL boundaries; no ROS init, camera, serial or actuator output."""
import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
sys.path.insert(0, str(PACKAGE/'scripts'))
from mission.config import load
from mission.core import Mission
from mission.health_contract import source_ttl
from mission.stop_home import geometry_home_ready

CONFIG = ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'


class HeartbeatContracts(unittest.TestCase):
    def setUp(self):
        self.cfg = load(CONFIG, ROOT)

    def test_perception_timer_publishes_health_with_disabled_and_enabled_timeout(self):
        # Execute the real timer body without starting ROS, camera or motors.
        import ast,json,queue,threading
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        tree=ast.parse((PACKAGE/'scripts/grasp_perception.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Perception')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
        scope=dict(time=NS(monotonic=lambda:10000.,monotonic_ns=lambda:10000000000000),
            dispatch_pending=lambda *args:None,report_scene=lambda *args:None,
            geometry_home_ready=geometry_home_ready,CONTROLLED_HOME_PHASES=(),String=NS,json=json,queue=queue,verified=lambda checks:True)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'grasp_perception.tick','exec'),scope)
        for timeout,busy,camera_ok,scene_ok,expected in (
            (None,True,True,True,True),(None,False,True,True,True),
            (None,True,False,True,False),(None,True,True,False,False),
            (300.,True,True,True,False),(20000.,True,True,True,True)):
            with self.subTest(timeout=timeout,busy=busy,camera_ok=camera_ok,scene_ok=scene_ok):
                snapshot=Mock(side_effect=None if scene_ok else ValueError('SCENE_STALE'))
                node=NS(cfg={'execution':{'state_timeout_s':timeout}},busy=busy,started=0.,reason='READY',controlled_stop=False,r3_phase='',vision_cancel_complete=False,vision_reloading=False,last_request=0,request_session='',
                    owner_subscription=None,owner=None,native_leg_input=None,odometry_subscription=None,odom=None,
                    mission_status_subscription=None,mission_status=None,navigation_session='nav',lock=threading.Lock(),
                    body_source=NS(listener=NS(tf_sub=None,callback=None),select=lambda _:None,diagnostics={}),
                    scene_buffer=NS(snapshot=snapshot,diagnostics=lambda _:{}),
                    backend=NS(readiness=lambda:dict(ready=camera_ok,reason='READY' if camera_ok else 'CAMERA_FRAME_STALE'),
                        camera_mode='active',startup_checks={},compute_metrics={},collision=NS()),
                    health=Mock(),last_compute={},startup_checks={},leg_reader=NS(status=lambda:{}),completed=queue.Queue())
                scope['tick'](node)
                row=json.loads(node.health.publish.call_args.args[0].data)
                self.assertEqual(row['ready'],expected)
                self.assertEqual(row['busy'],busy)
                self.assertIn('last_tick_duration_s',vars(node))

    def test_queued_arm_feedback_after_slow_tf_is_consumed_before_snapshot(self):
        import ast,json,queue,threading
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        source=(PACKAGE/'scripts/grasp_perception.py').read_text()
        # Compare the actual timer with its previous receive-before-TF order.
        old=source.replace('            dispatch_pending(self.owner_subscription,self.owner)\n','')
        for code,queued,expected in ((old,True,False),(source,True,True),(source,False,False)):
            now=[100.];last=[100.];pending=[];ages=[]
            def select(_):
                now[0]=100.7
                if queued:pending.append(100.69)
            def dispatch(subscription,callback):
                if subscription=='arm' and pending:callback(pending.pop())
            def snapshot(stamp):
                age=stamp/1e9-last[0];ages.append(age)
                if age>.5:raise ValueError('SCENE_SNAPSHOT_STALE:live_arm')
            tree=ast.parse(code)
            cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Perception')
            method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
            scope=dict(time=NS(monotonic=lambda:now[0],monotonic_ns=lambda:round(now[0]*1e9)),
                dispatch_pending=dispatch,report_scene=lambda *args:None,
                geometry_home_ready=geometry_home_ready,CONTROLLED_HOME_PHASES=(),String=NS,json=json,queue=queue,verified=lambda _:True)
            exec(compile(ast.Module(body=[method],type_ignores=[]),'timer-regression','exec'),scope)
            node=NS(cfg={'execution':{'state_timeout_s':None}},busy=False,started=0.,reason='READY',controlled_stop=False,r3_phase='',vision_cancel_complete=False,vision_reloading=False,last_request=0,request_session='',
                owner_subscription='arm',owner=lambda stamp:last.__setitem__(0,stamp),native_leg_input=None,
                odometry_subscription=None,odom=None,mission_status_subscription=None,mission_status=None,
                navigation_session='nav',lock=threading.Lock(),
                body_source=NS(listener=NS(tf_sub=None,callback=None),select=select,diagnostics={}),
                scene_buffer=NS(snapshot=snapshot,diagnostics=lambda _:{}),
                backend=NS(readiness=lambda:dict(ready=True,reason='READY'),camera_mode='active',
                    startup_checks={},compute_metrics={},collision=NS()),
                health=Mock(),last_compute={},startup_checks={},leg_reader=NS(status=lambda:{}),completed=queue.Queue())
            scope['tick'](node)
            result=json.loads(node.health.publish.call_args.args[0].data)
            self.assertEqual(result['ready'],expected)
            self.assertAlmostEqual(ages[-1],.01 if expected else .7)

    def test_arm_scene_budget_is_separate_from_owner_watchdog(self):
        self.assertEqual(self.cfg['limits']['arm_age_s'],.5)
        deployment=yaml.safe_load(Path(self.cfg['paths']['deployment']).read_text())['real_deployment']
        self.assertEqual(deployment['rars01']['feedback_timeout_s'],.25)
        raw=yaml.safe_load(CONFIG.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'mission.yaml'
            for value in (True,0.,-.1,.501,float('nan')):
                data=copy.deepcopy(raw);data['go2_mission']['frames']['arm_feedback_max_age_s']=value
                path.write_text(yaml.safe_dump(data))
                with self.assertRaisesRegex(ValueError,'arm_feedback_max_age_s'):load(path,ROOT)

    def test_timing_profile_keeps_compact_gripper_recording_without_changing_control(self):
        self.assertFalse(self.cfg['execution']['diagnostic_recording_enabled'])
        self.assertTrue(self.cfg['gripper_observation']['recording_enabled'])
        self.assertTrue(self.cfg['gripper_observation']['enabled'])
        deployment=yaml.safe_load(Path(self.cfg['paths']['deployment']).read_text())['real_deployment']['r3_commissioning']
        self.assertFalse(deployment['control_latency']['enabled'])
        self.assertFalse(deployment['motion_diagnostics']['enabled'])
        self.assertFalse(deployment['guardian']['status_recording_enabled'])
        self.assertEqual(deployment['guardian']['priority'],20)
        self.assertEqual(deployment['guardian']['status_source_timeout_s'],.25)

    def test_source_specific_admission_and_later_freshness_share_one_clock(self):
        for name, ttl in [('nav', .60), ('perception', 1.50), ('r3', .50),
                          ('owner', .25), ('base', 1.00), ('scene', 1.00)]:
            with self.subTest(name=name):
                for age, accepted in [(ttl-.001, True), (ttl, True), (ttl+.001, False)]:
                    m=Mission(copy.deepcopy(self.cfg), 'test')
                    stamp=100_000_000_000
                    self.assertEqual(m.observe(name, {'observed_ns': stamp}, 100+age), accepted)
                    self.assertEqual(m.fresh(name, 100+age), accepted)
                    if accepted:
                        self.assertFalse(m.fresh(name, 100+ttl+.001))

    def test_nav_jitter_does_not_renew_physical_inputs(self):
        m=Mission(self.cfg, 'test')
        for delay in (.04, .08, .25, .50):
            self.assertTrue(m.observe('nav', {'observed_ns': int((100+delay)*1e9)}, 100+delay+.05))
        self.assertFalse(m.observe('nav', {'observed_ns': 101_000_000_000}, 101.65))
        self.assertTrue(m.observe('nav', {'observed_ns': 102_000_000_000}, 102.49))
        self.assertFalse(m.observe('base', {'observed_ns': 101_990_000_000}, 103.001))
        self.assertFalse(m.observe('owner', {'observed_ns': 102_000_000_000}, 102.251))

    def test_status_alive_does_not_make_scene_valid(self):
        m=Mission(self.cfg, 'test')
        self.assertTrue(m.observe('perception', {'observed_ns': 100_000_000_000, 'ready': False}, 100.49))
        self.assertFalse(m.perception_service_ready(100.49))
        self.assertFalse(m.fresh('perception', 101.501))
        self.assertFalse(m.observe('scene', {'observed_ns': 100_000_000_000}, 101.001))

    def test_future_duplicate_and_out_of_order_never_renew_lease(self):
        m=Mission(self.cfg, 'test')
        self.assertTrue(m.observe('nav', {'observed_ns': 100_000_000_000}, 100.1))
        for stamp in (100_000_000_000, 99_999_999_999, 101_000_000_000):
            self.assertFalse(m.observe('nav', {'observed_ns': stamp}, 100.2))
        self.assertEqual(m.received['nav'], 100.)
        self.assertFalse(m.fresh('nav', 100.601))

    def test_rejection_names_source_and_preserves_actual_age(self):
        m=Mission(self.cfg, 'test')
        self.assertFalse(m.observe('nav', {'observed_ns': 100_000_000_000,'session_id':'nav1'}, 100.65))
        d=m.input_rejections['nav']
        self.assertEqual(d['source_topic'], '/go2/navigation/status')
        self.assertAlmostEqual(d['status_age_ms'],650.)
        self.assertEqual(d['ttl_ms'],600.)
        self.assertEqual(d['session'],'nav1')
        self.assertEqual(d['action_taken'],'REJECT_INPUT')

    def test_legacy_config_falls_back_to_250ms_without_other_changes(self):
        cfg=copy.deepcopy(self.cfg)
        for key in ('r3_status_ttl_s','nav_status_ttl_s','perception_status_ttl_s'):cfg['execution'].pop(key)
        for name in ('r3','owner','nav','perception','compute'):
            self.assertEqual(source_ttl(cfg,name),.25)
        self.assertEqual(source_ttl(cfg,'scene'),1.00)
        self.assertEqual(cfg['execution']['camera_max_age_s'],2.00)

    def test_bad_ttl_is_rejected_and_launch_direct_import_still_works(self):
        spec=importlib.util.spec_from_file_location('standalone_config',PACKAGE/'scripts/mission/config.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        raw=yaml.safe_load(CONFIG.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'mission.yaml'
            for key, values in [('nav_status_ttl_s', (True,0.,float('nan'),.601)),
                                ('perception_status_ttl_s', (True,-1.,float('inf'),1.501))]:
                for value in values:
                    data=copy.deepcopy(raw);data['go2_mission']['execution'][key]=value
                    path.write_text(yaml.safe_dump(data))
                    with self.assertRaisesRegex(ValueError,key):module.load(path,ROOT)


if __name__ == '__main__':
    unittest.main()
