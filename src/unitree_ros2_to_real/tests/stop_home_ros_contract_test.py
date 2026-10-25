"""Call real ROS callbacks with in-memory doubles; never create a node/ROS graph."""
import json,queue,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import grasp_mission as gm
import grasp_perception as gp
from mission.stop_home import HomeDispatch
from mission.compute_cancel import ComputeCancellation

class ContractTests(unittest.TestCase):
    def test_perception_refuses_shutdown_window_and_deduplicates_retry(self):
        p=NS(backend=object(),busy=False,count_publishers=lambda _:1,controlled_stop=True,
             r3_phase='ARM_RETURN_HOME',vision_cancel_complete=False,last_request=0,request_session='',
             compute_cancel=ComputeCancellation(),jobs=queue.Queue(),cfg={'execution':{'heartbeat_ttl_s':.25}})
        r=dict(id=7,session_id='m',operation='home',observed_ns=time.monotonic_ns())
        msg=NS(data=json.dumps(r));gp.Perception.request(p,msg)
        self.assertTrue(p.jobs.empty())
        p.vision_cancel_complete=True;gp.Perception.request(p,msg)
        self.assertEqual(p.jobs.get_nowait()['id'],7)
        p.busy=False;gp.Perception.request(p,msg)
        self.assertTrue(p.jobs.empty())
    def test_real_coordinator_tick_waits_then_dispatches_and_ack_stops_retry(self):
        def pub():return NS(values=[],publish=lambda msg:None)
        health=[];sent=[]
        e=NS(inputs=dict(r3=dict(state='CONTROLLED_STOP',phase='ARM_RETURN_HOME',fault_latched=False),
            nav=dict(session_id='nav'),owner=dict(owner_session_id='arm',mission_capable=True,feedback_ready=True,target_valid=True,home_requested=True),
            perception=dict(ready=False,home_compute_ready=False,busy=False,reason='SHUTDOWN')),
            fresh=lambda name,now:True,step=lambda *args:[],stale_input_diagnostic=None,
            # A stale navigation-goal latch must not prevent stop HOME supervision.
            wait_goal_stale_since=1.,perception_service_ready=lambda now:False,
            session='m',revision=1,state='CANCELLED',request_id=10,
            status=lambda now:dict(observation_recovery={}))
        listener=NS(tf_sub=None,callback=lambda msg:None)
        client=NS(service_is_ready=lambda:True,call_async=lambda req:None)
        s=NS(engine=e,input_diagnostics=NS(tick=lambda now:None),status_subscriptions={},native_leg_input=None,
            odometry_subscription=None,base=lambda msg:None,body_source=NS(listener=listener,diagnostics={}),
            refresh_scene=lambda:None,physical=True,blockers=[],short_journal=None,scene_error='',
            leg_reader=NS(status=lambda:{}),scene_buffer=NS(diagnostics=lambda now:{}),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=1)),topics={},count_publishers=lambda _:1,
            cancel=client,home=client,cancel_nav=client,invalidate_nav=client,
            health_pub=NS(publish=lambda msg:health.append(json.loads(msg.data))),
            compute_pub=NS(publish=lambda msg:sent.append(json.loads(msg.data))),status_pub=pub(),
            cancel_future=None,owner_cancel_ack=True,cancel_retry_at=0.,stop_home=None,stop_home_attempted=False,
            home_dispatch=HomeDispatch(),stop_home_status=None,stop_home_result='',
            get_logger=lambda:NS(info=lambda msg:None),grip_observer=None,grip_error='',grip_log=None,
            cfg={'collision':{}})
        with patch.object(gm,'dispatch_pending',lambda *args:None):
            gm.Coordinator.tick(s)
            self.assertFalse(health[-1]['ready']);self.assertFalse(sent)
            e.inputs['perception'].update(home_compute_ready=True,ready=True)
            gm.Coordinator.tick(s)
            self.assertTrue(health[-1]['ready']);self.assertEqual(len(sent),1)
            self.assertEqual(sent[0]['operation'],'home')
            # A dropped request is retried, retaining identity.
            s.home_dispatch.sent_at-=1;gm.Coordinator.tick(s)
            self.assertEqual(len(sent),2);self.assertEqual(sent[0]['id'],sent[1]['id'])
            e.inputs['perception'].update(active_request_id=sent[0]['id'],active_request_session='m',busy=True)
            s.home_dispatch.sent_at-=1;gm.Coordinator.tick(s)
            self.assertEqual(len(sent),2);self.assertTrue(s.home_dispatch.accepted)
            e.inputs['r3'].update(state='EMERGENCY_FAULT',fault_latched=True)
            gm.Coordinator.tick(s);self.assertEqual(len(sent),2)

if __name__=='__main__':unittest.main()
