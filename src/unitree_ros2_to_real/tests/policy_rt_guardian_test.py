"""Offline process identity/lease and stop rules; never starts R3 or ROS."""
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('guardian', Path(__file__).resolve().parents[1]/'scripts/r3_rt_guardian.py')
g = importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
CONFIG=g.load_guardian_config(Path(__file__).resolve().parents[1]/'config/profiles/go2_rars01_commissioning.yaml')
CHILD = '''import ctypes,json,os,threading,time
from pathlib import Path
def worker():
 ctypes.CDLL(None).prctl(15,b'r3_rt_policy',0,0,0)
 print(json.dumps(dict(pid=os.getpid(),tid=threading.get_native_id())),flush=True)
 time.sleep(30)
threading.Thread(target=worker).start()
time.sleep(30)
'''


class GuardianTests(unittest.TestCase):
    def setUp(self):
        self.child = subprocess.Popen([sys.executable, '-c', CHILD], stdout=subprocess.PIPE, text=True)
        d = json.loads(self.child.stdout.readline())
        d.update(pid_start=g.identity(d['pid'])['start'],tid_start=g.identity(d['pid'],d['tid'])['start'],uid=os.getuid())
        self.ready = d
        self.lease = os.memfd_create('guardian-test')
        os.ftruncate(self.lease,4096)
        os.pwrite(self.lease,struct.pack('<8s11Q',b'R3OFF001',*([0]*11)),0)
        self.sha = g.sha(f'/proc/{self.child.pid}/exe')

    def tearDown(self):
        self.child.terminate();self.child.wait(timeout=3);self.child.stdout.close();os.close(self.lease)

    def test_identity_negatives_never_call_scheduler(self):
        for key,value in [('tid',self.child.pid),('pid_start',self.ready['pid_start']+1),
                          ('tid_start',self.ready['tid_start']+1),('uid',os.getuid()+1),('pid',self.child.pid+1)]:
            d=dict(self.ready);d[key]=value
            with self.subTest(key=key), patch.object(os,'sched_setscheduler',side_effect=AssertionError('unexpected syscall')):
                with self.assertRaises(RuntimeError):g.change_scheduler(self.child,d,self.lease,self.sha,20)

    def test_wrong_executable_never_call_scheduler(self):
        with patch.object(os,'sched_setscheduler',side_effect=AssertionError('unexpected syscall')):
            with self.assertRaises(RuntimeError):g.change_scheduler(self.child,self.ready,self.lease,'0'*64,20)

    def test_terminal_lease_refuses_scheduler(self):
        os.pwrite(self.lease,struct.pack('<Q',2),8)
        with patch.object(os,'sched_setscheduler',side_effect=AssertionError('unexpected syscall')):
            with self.assertRaises(RuntimeError):g.change_scheduler(self.child,self.ready,self.lease,self.sha,0)

    def test_own_tid_reset_readback_and_closes_admission(self):
        d=g.change_scheduler(self.child,self.ready,self.lease,self.sha,0)
        self.assertEqual((d['policy'],d['priority']),(0,0))
        self.assertEqual(struct.unpack('<Q',os.pread(self.lease,8,88))[0],1)

    def test_shutdown_requires_fresh_stable_no_output(self):
        s=dict(output_enabled=False,lowcmd_publisher_present=False,lowcmd_lease_present=False,custom_leg_output='OFF')
        self.assertTrue(g.safe_exit(s,.1,2,CONFIG))
        for age,stable in [(-.1,2),(.5,2),(.1,1)]:self.assertFalse(g.safe_exit(s,age,stable,CONFIG))
        for key in ['output_enabled','lowcmd_publisher_present','lowcmd_lease_present']:
            d=dict(s);d[key]=True;self.assertFalse(g.safe_exit(d,.1,100,CONFIG))
        self.assertFalse(g.safe_exit({},.1,100,CONFIG))


class LiveProbeAdmissionTests(unittest.TestCase):
    def test_wrong_live_domain_stops_before_child_creation(self):
        with patch.dict(os.environ, {}, clear=True), patch.object(sys, 'argv', ['guardian', '--live-compute-probe', '/tmp/r3_live_shadow']), patch.object(subprocess, 'Popen', side_effect=AssertionError('unexpected child')):
            with self.assertRaisesRegex(RuntimeError, 'real domain0'):
                g.main()

    def test_live_mode_refuses_production_executable(self):
        env=dict(ROS_DOMAIN_ID='0', CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP8p1s0" /></Interfaces></General></Domain></CycloneDDS>')
        with patch.dict(os.environ, env, clear=True), patch.object(sys, 'argv', ['guardian', '--live-compute-probe', '/tmp/go2_r3_commissioning']), patch.object(subprocess, 'Popen', side_effect=AssertionError('unexpected child')):
            with self.assertRaisesRegex(RuntimeError, 'Expected installed'):
                g.main()


class ComputeProbeAdmissionTests(unittest.TestCase):
    def test_wrong_domain_stops_before_child_creation(self):
        with patch.dict(os.environ, {}, clear=True), patch.object(sys, 'argv', ['guardian', '--compute-probe', '/tmp/r3_runtime_timing']), patch.object(subprocess, 'Popen', side_effect=AssertionError('unexpected child')):
            with self.assertRaisesRegex(RuntimeError, 'isolated domain228'):
                g.main()

    def test_compute_mode_rejects_production_binary(self):
        env=dict(ROS_DOMAIN_ID='228', ROS_LOCALHOST_ONLY='1', CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>')
        with patch.dict(os.environ, env, clear=True), patch.object(sys, 'argv', ['guardian', '--compute-probe', '/tmp/go2_r3_commissioning']), patch.object(subprocess, 'Popen', side_effect=AssertionError('unexpected child')):
            with self.assertRaisesRegex(RuntimeError, 'Expected installed R3 executable'):
                g.main()

    def test_missing_rt_rights_stops_before_ros_and_child(self):
        env=dict(ROS_DOMAIN_ID='228', ROS_LOCALHOST_ONLY='1', CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>')
        with patch.dict(os.environ, env, clear=True), patch.object(sys, 'argv', ['guardian', '--compute-probe', '/tmp/r3_runtime_timing']), patch.object(g.resource, 'getrlimit', return_value=(0,0)), patch.object(subprocess, 'Popen', side_effect=AssertionError('unexpected child')):
            with self.assertRaisesRegex(RuntimeError, 'FIFO20 unavailable'):
                g.main()


if __name__ == '__main__':unittest.main()
