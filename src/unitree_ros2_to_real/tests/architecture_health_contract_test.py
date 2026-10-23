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

CONFIG = ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'


class HeartbeatContracts(unittest.TestCase):
    def setUp(self):
        self.cfg = load(CONFIG, ROOT)

    def test_source_specific_admission_and_later_freshness_share_one_clock(self):
        for name, ttl in [('nav', .60), ('perception', 1.00), ('r3', .50),
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
        self.assertFalse(m.fresh('perception', 101.001))
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
        self.assertEqual(cfg['execution']['camera_max_age_s'],1.00)

    def test_bad_ttl_is_rejected_and_launch_direct_import_still_works(self):
        spec=importlib.util.spec_from_file_location('standalone_config',PACKAGE/'scripts/mission/config.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        raw=yaml.safe_load(CONFIG.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'mission.yaml'
            for key, values in [('nav_status_ttl_s', (True,0.,float('nan'),.601)),
                                ('perception_status_ttl_s', (True,-1.,float('inf'),1.001))]:
                for value in values:
                    data=copy.deepcopy(raw);data['go2_mission']['execution'][key]=value
                    path.write_text(yaml.safe_dump(data))
                    with self.assertRaisesRegex(ValueError,key):module.load(path,ROOT)


if __name__ == '__main__':
    unittest.main()
