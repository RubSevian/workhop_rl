"""v4 mission/recovery/reverse contracts; synthetic inputs, zero physical IO."""
import copy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture, CFG
from mission.approved_motion import reverse_segment
from mission.config import blockers
from mission.operator_events import OperatorEvents
from mission.scene import WorldObjectAnchor


class ArchitectureMissionTests(unittest.TestCase):
    def test_r3_status_has_separate_half_second_ttl(self):
        from mission.health_contract import source_ttl
        f=Fixture();f.feed()
        self.assertEqual(source_ttl(f.m.cfg,'r3'),.5)
        self.assertEqual(source_ttl(f.m.cfg,'owner'),.25)
        for age,expected in ((.281,True),(.499,True),(.501,False)):
            f.m.received['r3']=f.now-age
            self.assertEqual(f.m.fresh('r3',f.now),expected)
        f.m.received['owner']=f.now-.281
        self.assertFalse(f.m.fresh('owner',f.now))
        cfg=copy.deepcopy(f.m.cfg);cfg['execution'].pop('r3_status_ttl_s')
        self.assertEqual(source_ttl(cfg,'r3'),.25)

    def test_perception_status_missing_in_wait_goal_is_recoverable(self):
        f=Fixture();f.tick();f.feed();f.m.received['perception']=f.now-1.
        f.m.step(f.now,int(f.now*1e9))
        self.assertEqual(f.m.state,'WAIT_GOAL')
        self.assertEqual(f.m.reason,'PERCEPTION_STATUS_UNAVAILABLE')
        self.assertFalse(f.m.availability['manipulation'])
        self.assertFalse(any(k=='cancel' for k,_ in f.m.drain()))
        f.now+=100;f.tick(False);self.assertEqual(f.m.state,'WAIT_GOAL')

    def test_nav_status_lost_during_navigation_revokes_goal_not_leg_authority(self):
        f=Fixture();f.tick();f.nav['goal']=dict(revision=1,stamp_ns=int((f.now+.01)*1e9),state='ACTIVE')
        f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')
        f.feed();f.m.received['nav']=f.now-.601
        effects=f.m.step(f.now,int(f.now*1e9))
        self.assertEqual(f.m.state,'WAIT_GOAL');self.assertEqual(f.r3['phase'],'RL_ZERO')
        self.assertEqual([kind for kind,_ in effects],['invalidate_navigation_goal'])
        f.tick(False);self.assertEqual(f.m.reason,'RECOVERED_NEW_GOAL_REQUIRED')
        f.tick(False);self.assertEqual(f.m.state,'WAIT_GOAL')
        f.nav['goal']=dict(revision=2,stamp_ns=int((f.now+.01)*1e9),state='REQUESTED')
        f.tick(False);self.assertEqual(f.m.state,'NAVIGATING')

    def test_vision_unavailable_does_not_cancel_navigation(self):
        f=Fixture();f.tick();f.nav['goal']=dict(revision=1,stamp_ns=int((f.now+.01)*1e9),state='ACTIVE')
        f.tick(False);f.feed();f.m.received['perception']=f.now-2.
        f.m.step(f.now,int(f.now*1e9));self.assertEqual(f.m.state,'NAVIGATING')

    def test_fixed_wait_ignores_small_sway_but_tilt_still_blocks(self):
        f=Fixture();f.start();f.tick(False);since=f.m.stable_since
        f.base['pose'][0]=.03;f.now=since+3.-.02;f.tick(False)
        self.assertEqual(f.m.stable_since,since)
        f.now=since+5.001-.02;f.tick(False);self.assertEqual(f.m.state,'ARM_SCAN_POSE')
        f=Fixture();f.start();f.base['tilt_rad']=.3;f.tick(False)
        self.assertEqual(f.m.state,'BLOCKED');self.assertEqual(f.m.reason,'BASE_TILT_UNSAFE')

    def test_planning_rejection_waits_for_operator_without_retry_or_motion(self):
        for reason in ('IK_FAIL','NO_CANDIDATES','CANDIDATE_GEOMETRY_REJECTED','NO_COLLISION_FREE_CANDIDATE: arm/leg'):
            class Rejected(Fixture):
                def compute(self,data):
                    result=super().compute(data)
                    if data['operation']=='grasp_plan':result.update(ok=False,reason=reason)
                    return result
            f=Rejected();self.assertEqual(f.run(),'BLOCKED');self.assertEqual(f.m.reason,reason)
            plans=[d for k,d in f.effects if k=='compute' and d['operation']=='grasp_plan']
            self.assertEqual(len(plans),1)
            self.assertFalse(any(k=='compute' and d['operation'] in ('pregrasp','close','reverse_search','home','scan_step') for k,d in f.effects))
            for _ in range(10):f.tick()
            self.assertEqual(f.m.state,'BLOCKED');self.assertIsNone(f.m.pending)

    def test_unknown_grasp_does_not_start_reverse_or_retreat(self):
        class Unknown(Fixture):
            def effect(self,kind,data):
                super().effect(kind,data)
                if kind=='trajectory' and data['kind']=='close':self.owner.pop('grasp_assessment',None)
        f=Unknown();self.assertEqual(f.run(),'BLOCKED')
        self.assertEqual(f.m.reason,'GRASP_ASSESSMENT_UNKNOWN:NO_VALID_ASSESSMENT')
        self.assertFalse(any(k=='compute' and d['operation'] in ('retreat','reverse_search') for k,d in f.effects))

    def reverse_fixture(self):
        f=Fixture();f.feed();scene=copy.deepcopy(f.m.inputs['scene'])
        points=np.zeros((3,7));points[:,0]=[0.,.001,.002]
        segment=dict(kind='move',id=7,collision_checked=True,payload_checked=False,
                     points=points.tolist(),scene_snapshot=scene)
        current=dict(q7=points[-1].tolist(),scene_snapshot=copy.deepcopy(scene))
        return segment,current

    def test_reverse_is_exact_and_does_not_rewrite_original_approval(self):
        segment,current=self.reverse_fixture();original=copy.deepcopy(segment)
        result=reverse_segment(segment,current,CFG,known_empty=True)
        self.assertEqual(result['points'],segment['points'][::-1])
        self.assertEqual(result['reverse_of_request'],7);self.assertTrue(result['reused_approval'])
        self.assertEqual(segment,original)
        self.assertEqual(result['approval_scene_snapshot'],original['scene_snapshot'])

    def test_reverse_rejects_unknown_hold_epoch_and_start(self):
        segment,current=self.reverse_fixture()
        with self.assertRaisesRegex(ValueError,'HOLD_UNKNOWN'):reverse_segment(segment,current,CFG,known_empty=False)
        for change,reason in [('epoch','EPOCH_CHANGED'),('start','START_CHANGED')]:
            modified=copy.deepcopy(current)
            if change=='epoch':modified['scene_snapshot']['body']['tf_revision']='new'
            else:modified['q7'][0]+=.021
            with self.assertRaisesRegex(ValueError,reason):reverse_segment(segment,modified,CFG,known_empty=True)

    def test_reverse_keeps_approval_when_live_legs_change(self):
        segment,current=self.reverse_fixture()
        current['scene_snapshot']['legs']=list(current['scene_snapshot']['legs'])
        current['scene_snapshot']['legs'][0]+=.01
        result=reverse_segment(segment,current,CFG,known_empty=True)
        self.assertEqual(result['points'],segment['points'][::-1])
        self.assertEqual(result['approval_scene_snapshot'],segment['scene_snapshot'])
        self.assertNotEqual(result['scene_snapshot']['legs'],segment['scene_snapshot']['legs'])

    def test_preflight_full_mode_does_not_require_hardware_torque_cap(self):
        cfg=copy.deepcopy(CFG);cfg['test_mode']='ready_observe';self.assertNotIn('full_grasp_motor_torque_cap_unqualified',blockers(cfg))
        cfg['test_mode']='full'
        self.assertNotIn('full_grasp_motor_torque_cap_unqualified',blockers(cfg))
        self.assertFalse(cfg['grasp_verification']['motor_torque_cap_validated'])

    def test_monitor_deduplicates_age_changes_but_keeps_first_cause(self):
        m=OperatorEvents();status={'state':'ACTIVE','phase':'RL_ZERO','observed_ns':100}
        self.assertIsNotNone(m.observe('r3',status,100,1000))
        status['observed_ns']=200;self.assertIsNone(m.observe('r3',status,200,2000))
        status.update(state='EMERGENCY_FAULT',fault='policy_result_stale',fault_phase='RL_ZERO')
        event=m.observe('r3',status,300,3000)
        self.assertEqual(event['view']['fault'],'policy_result_stale')
        self.assertIsNone(m.observe('r3',status,400,4000))
        self.assertIsNotNone(m.observe('r3',{'decode_error':'bad YAML'},500,5000))
        self.assertIsNone(m.observe('r3',{'decode_error':'bad YAML'},600,6000))

    def test_operator_levels_recover_once_and_preserve_diagnostic_evidence(self):
        from mission.operator_events import render_event
        m=OperatorEvents()
        event=m.observe('mission',dict(state='BLOCKED',reason='PHYSICAL_NOT_READY',
            blockers=['physical_retreat_clearance_unqualified'],
            stale_input_diagnostic=dict(source_topic='/go2/navigation/status',status_age_ms=601,ttl_ms=600,action_taken='BLOCK_NEW_GOAL')),1,1)
        self.assertEqual(event['level'],'BLOCKED')
        line=render_event(event)
        self.assertIn('physical_retreat_clearance_unqualified',line);self.assertIn('ttl_ms=600',line)
        data=dict(state='WAIT_GOAL',availability=dict(navigation=True,manipulation=True))
        event=m.observe('mission',data,2,2)
        self.assertEqual(event['event_type'],'RECOVERED');self.assertEqual(event['level'],'OK')
        data['observed_ns']=3;self.assertIsNone(m.observe('mission',data,3,3))
        warning=m.observe('r3',dict(state='ACTIVE',phase='RL_ZERO',phase_warnings=['NAV_UNAVAILABLE']),4,4)
        self.assertEqual(warning['level'],'WARNING')
        fault=m.observe('r3',dict(state='EMERGENCY_FAULT',fault_latched=True,fault='arm_feedback_age',fault_phase='RL_ZERO'),5,5)
        self.assertEqual(fault['level'],'FAULT');self.assertIn('arm_feedback_age',render_event(fault))
        with self.assertRaisesRegex(ValueError,'STATUS_MAPPING_REQUIRED'):m.observe('nav',[],6,6)

    def test_object_anchor_is_separate_from_grasp_and_epoch_bound(self):
        f=Fixture();f.feed()
        from mission.scene import BodyPose
        data=f.m.inputs['scene']['body'];body=BodyPose(**data)
        anchor=WorldObjectAnchor([.1,.2,.3],body.mono_ns,body.ros_ns,'nav','synthetic-tf-r1',1,'arm',.9)
        anchor.validate(body,1,'arm');self.assertEqual(anchor.data()['point_map'],[.1,.2,.3])
        with self.assertRaisesRegex(ValueError,'EPOCH_CHANGED'):anchor.validate(body,2,'arm')


if __name__=='__main__':unittest.main()
