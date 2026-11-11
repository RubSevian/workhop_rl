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


class ProbeInputRtTests(unittest.TestCase):
    def setUp(self):
        self.states={tid:dict(pid=123,tid=tid,start=tid+100,comm=name,
                     policy=os.SCHED_OTHER,priority=0,cpu_ns=0)
                     for tid,name in enumerate(sorted(g.ProbeInputRt.names),200)}
        self.proc=unittest.mock.Mock(pid=123)
        self.proc.poll.return_value=None
        def identify(pid,tid=None):
            if tid is None:return dict(start=10,parent=os.getpid())
            return dict(self.states[tid])
        def schedule(tid,policy,param):
            self.states[tid].update(policy=policy,priority=param.sched_priority)
        self.patches=[patch.object(g,'identity',side_effect=identify),patch.object(g,'sha',return_value='hash'),
                      patch.object(os,'sched_getaffinity',return_value={1}),
                      patch.object(os,'sched_setscheduler',side_effect=schedule)]
        self.mocks=[p.start() for p in self.patches]
        self.addCleanup(lambda:[p.stop() for p in reversed(self.patches)])
        self.group=g.ProbeInputRt(self.proc,10,'hash')
        self.group._tasks=lambda:list(self.states)

    def test_physical_live_and_wrong_priority_are_refused(self):
        for compute,live,value in ((False,False,'15'),(True,True,'15'),(True,False,'20')):
            with self.assertRaises(RuntimeError):g.input_rt_requested(value,compute,live)
        self.assertTrue(g.input_rt_requested('15',True,False))
        self.assertFalse(g.input_rt_requested(None,False,False))

    def test_promotion_audit_and_demotion(self):
        self.assertTrue(self.group.discover());self.group.promote()
        self.group.audit(1,.5,.8)
        self.assertTrue(all(v['priority']==15 for v in self.states.values()))
        self.group.demote()
        self.assertTrue(all(v['priority']==0 for v in self.states.values()))
        self.assertTrue(self.group.demoted)

    def test_wrong_identity_does_not_change_schedulers(self):
        self.group.pid_start=11
        with self.assertRaises(RuntimeError):self.group.discover()
        self.mocks[-1].assert_not_called()

    def test_missing_or_unpinned_threads_do_not_promote(self):
        tid=next(iter(self.states));self.states.pop(tid)
        self.assertFalse(self.group.discover())
        self.mocks[-1].assert_not_called()

    def test_partial_failure_rolls_back(self):
        self.assertTrue(self.group.discover())
        original=self.mocks[-1].side_effect
        bad=sorted(self.states)[1]
        def fail(tid,policy,param):
            if tid==bad and policy==os.SCHED_FIFO:raise OSError('simulated admission failure')
            original(tid,policy,param)
        self.mocks[-1].side_effect=fail
        with self.assertRaises(OSError):self.group.promote()
        self.assertTrue(all(v['priority']==0 for v in self.states.values()))

    def test_reused_tid_is_not_demoted_but_others_are(self):
        self.assertTrue(self.group.discover());self.group.promote()
        bad=next(iter(self.states));self.states[bad]['start']+=1
        self.mocks[-1].reset_mock()
        with self.assertRaises(RuntimeError):self.group.demote()
        self.assertFalse(any(call.args[0]==bad for call in self.mocks[-1].call_args_list))
        self.assertTrue(all(v['priority']==0 for tid,v in self.states.items() if tid!=bad))

    def test_group_cpu_runaway_is_detected(self):
        self.assertTrue(self.group.discover());self.group.promote();self.group.audit(1,.5,.8)
        for v in self.states.values():v['cpu_ns']=300_000_000
        with self.assertRaisesRegex(RuntimeError,'CPU runaway'):self.group.audit(1_000_000_001,.5,.8)


if __name__ == '__main__':unittest.main()
