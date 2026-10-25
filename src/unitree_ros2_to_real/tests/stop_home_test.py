"""Stop handshake regression: no ROS graph, camera or motor commands."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.stop_home import HomeDispatch,home_dispatch_blocker,geometry_home_ready

class StopHomeTests(unittest.TestCase):
    def setUp(self):
        self.e=NS(inputs=dict(r3=dict(state='CONTROLLED_STOP',phase='ARM_RETURN_HOME',fault_latched=False),
            owner=dict(home_requested=True,mission_capable=True,feedback_ready=True,target_valid=True),
            perception=dict(home_compute_ready=True,busy=False,reason='READY')),fresh=lambda name,now:True)
        self.options=dict(physical=True,blockers=[],cancel_ack=True,services_ready=True,unique_publishers=True)
    def gate(self):return home_dispatch_blocker(self.e,1.,**self.options)
    def test_shutdown_window_is_not_geometry_ready(self):
        args=dict(controlled_stop=True,phase='ARM_RETURN_HOME',cancel_complete=False,backend_ready=True,scene_error='')
        # Backend has stopped worker already, but shutdown handshake has not finished.
        self.assertFalse(geometry_home_ready(**args))
        args['cancel_complete']=True
        self.assertTrue(geometry_home_ready(**args))
        for key,value in [('phase','ARM_DISABLE'),('scene_error','STALE'),('backend_ready',False),('reloading',True),('controlled_stop',False)]:
            self.assertFalse(geometry_home_ready(**dict(args,**{key:value})))
    def test_stop_from_planning_approach_and_hold_waits_for_idle(self):
        for previous in ('PLAN_GRASP','GRASP','WAIT_RETREAT_APPROVAL','RETREAT'):
            self.e.state=previous
            self.e.inputs['perception']['busy']=True
            self.assertEqual(self.gate(),'WAIT_PLANNER_IDLE')
            self.e.inputs['perception']['busy']=False
            self.assertEqual(self.gate(),'')
    def test_every_gate_reports_and_recovers(self):
        for source in ('r3','owner','perception','nav'):
            self.e.fresh=lambda name,now: name!=source
            self.assertEqual(self.gate(),'STALE_'+source.upper())
        self.e.fresh=lambda name,now:True
        self.options['cancel_ack']=False;self.assertEqual(self.gate(),'WAIT_CANCEL_ACK')
        self.options['cancel_ack']=True
        for key in ('mission_capable','feedback_ready','target_valid'):
            self.e.inputs['owner'][key]=False;self.assertEqual(self.gate(),'OWNER_FEEDBACK_NOT_READY')
            self.e.inputs['owner'][key]=True
        self.e.inputs['perception']['home_compute_ready']=False
        self.assertTrue(self.gate().startswith('WAIT_GEOMETRY_READY'))
        self.e.inputs['perception']['home_compute_ready']=True
        self.assertEqual(self.gate(),'')
        self.e.inputs['r3']['fault_latched']=True;self.assertEqual(self.gate(),'FAULT_LATCHED')
    def test_lost_publication_retries_identity_until_ack_and_never_restarts(self):
        h=HomeDispatch();h.request=dict(id=12,session_id='m')
        self.assertTrue(h.due(1.,{},True));h.sent_at=1.
        self.assertFalse(h.due(1.1,{},True));self.assertFalse(h.due(2.,{},False))
        self.assertTrue(h.due(2.,{},True))
        self.assertTrue(h.due(2.,dict(active_request_id=12,active_request_session='other'),True))
        self.assertFalse(h.due(2.,dict(active_request_id=12,active_request_session='m'),True))
        self.assertFalse(h.due(8.,{},True))
    def test_result_before_ack_prevents_retry(self):
        h=HomeDispatch();h.request=dict(id=12,session_id='m');h.sent_at=1.;h.finished=True
        self.assertFalse(h.due(8.,{},True))

if __name__=='__main__':unittest.main()
