#!/usr/bin/env python3
"""No ROS/SDK; informational classifier and bounded recording regression."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.gripper_observation import GripperObservation,ObservationLog
CFG=dict(min_abs_torque_nm=1.,max_abs_velocity_rad_s=.05,stable_s=.2,record_hz=50.,queue_capacity=100,flush_s=.01)
GRIP=dict(angle_open=1.,closed_position_rad=0.,hard_stop_angle_rad=.01)

def sample(stamp,**extra):
    return dict(frame_received_ns=stamp,owner_session_id='one',feedback_ready=True,
                q=[0.]*6+[.3],dq=[0.]*7,torque=[0.]*6+[-2.3],valid7=[True]*7,
                motor_id=list(range(1,8)),motor_status=[1]*7,gripper_command_active=True,**extra)

class ObservationTests(unittest.TestCase):
    def test_presumed_not_exact_command_and_not_latched(self):
        o=GripperObservation(CFG,GRIP,.1)
        for i in range(5):row=o.observe(sample(10**9+i*50000000),10**9+i*50000000,'RETREAT')
        self.assertEqual(row['state'],'PRESUMED_HELD');self.assertEqual(row['torque_nm'],-2.3)
        self.assertTrue(o.status(1200000000)['informational_only'])
        self.assertIsNone(o.observe(sample(1200000000),1200000001,'RETREAT'))
        d=sample(1250000000);d['q'][6]=0
        self.assertEqual(o.observe(d,1250000000,'HOME')['reason'],'CLOSED_STOP')
        d=sample(1300000000);d['torque'][6]=.1
        self.assertEqual(o.observe(d,1300000000,'HOME')['state'],'UNCONFIRMED')
    def test_bad_feedback_does_not_report_hold(self):
        for key,value in [('torque',[0.]*6+[float('nan')]),('valid7',[True]*6+[False]),('motor_status',[1]*6+[8]),('torque',None)]:
            with self.subTest(key=key,value=value):
                o=GripperObservation(CFG,GRIP,.1);d=sample(10**9);d[key]=value
                self.assertEqual(o.observe(d,10**9,'GRASP')['state'],'UNKNOWN')
        o=GripperObservation(CFG,GRIP,.1)
        self.assertEqual(o.observe(sample(10**9),2*10**9,'HOME')['state'],'UNKNOWN')
    def test_gap_session_stale_and_no_command(self):
        o=GripperObservation(CFG,GRIP,.1)
        o.observe(sample(10**9),10**9,'RETREAT')
        self.assertEqual(o.observe(sample(2*10**9),2*10**9,'RETREAT')['state'],'CONTACT_CANDIDATE')
        self.assertEqual(o.status(3*10**9)['state'],'UNKNOWN')
        d=sample(2050000000);d['owner_session_id']='new'
        self.assertEqual(o.observe(d,2050000000,'RETREAT')['state'],'CONTACT_CANDIDATE')
        d=sample(2100000000);d['gripper_command_active']=False
        self.assertEqual(o.observe(d,2100000000,'GRASP')['reason'],'NO_HOLD_COMMAND')
    def test_mission_records_compact_measured_torque_and_configured_target(self):
        import ast,time
        from types import SimpleNamespace as NS
        source=Path(__file__).resolve().parents[1]/'scripts/grasp_mission.py'
        tree=ast.parse(source.read_text())
        method=next(node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and node.name=='observe_gripper')
        scope={'time':time};exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        rows=[];observer=GripperObservation(CFG,GRIP,.1)
        fake=NS(grip_observer=observer,grip_log=NS(put=rows.append),grip_error='',
                engine=NS(session='test-session',state='GRASP'),
                cfg={'gripper_torque_control':{'contact_torque_nm':3.2}})
        stamp=time.monotonic_ns();owner=sample(stamp,grasp_assessment='NOT_GRASPED',grasp_assessment_reason='WIDTH_BELOW_MINIMUM')
        scope['observe_gripper'](fake,owner)
        self.assertEqual(fake.grip_error,'');self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['torque_nm'],-2.3)
        self.assertEqual(rows[0]['configured_contact_target_nm'],3.2)
        self.assertEqual(rows[0]['grasp_assessment'],'NOT_GRASPED')
        self.assertNotIn('last_compute_metrics',rows[0])
        self.assertLess(len(json.dumps(rows[0])),1800)

    def test_recording_and_disk_error_are_diagnostic(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'data.jsonl';log=ObservationLog(path,CFG)
            o=GripperObservation(CFG,GRIP,.1)
            for i in range(5):
                stamp=10**9+i*50000000
                log.put(o.observe(sample(stamp),stamp,'RETREAT'))
            log.close();rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(rows),5);self.assertEqual(rows[-1]['state'],'PRESUMED_HELD')
            self.assertEqual(log.error,'');self.assertEqual(log.dropped,0)
            failed=ObservationLog(path,CFG);failed.close();self.assertTrue(failed.error)

if __name__=='__main__':unittest.main()
