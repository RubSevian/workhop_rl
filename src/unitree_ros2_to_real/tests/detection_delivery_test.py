"""Delayed detection recovery must never adopt an old target or move the arm."""
import json
import queue
import threading
from types import SimpleNamespace as NS
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parent))
import first_grasp_mission_test as integration
from grasp_perception import Perception
from mission.compute_cancel import ComputeCancellation


def search():
    f,r=integration.Integration().observation_fixture()
    f.effects.clear();f.m.effects.clear()
    return f,r


def test_342ms_negative_result_advances_search_and_common_ttl_is_unchanged():
    f,request=search();r=f.compute(request);r['found']=False
    f.now+=.322;f.feed()  # feed advances the fixture clock by another 20 ms
    assert f.m.cfg['execution']['heartbeat_ttl_s']==.25
    assert f.m.result(r,f.now)
    assert f.m.state!='BLOCKED'
    assert f.m.last_result_diagnostic['result_ttl_s']==.35


def test_stale_positive_is_discarded_and_only_one_fresh_request_is_allowed():
    f,request=search();r=f.compute(request)
    f.now+=.36;f.feed()
    assert not f.m.result(r,f.now)
    assert f.m.object_anchor is None and f.m.pending is None
    assert f.m.retry_operation=='detect' and not f.m.effects
    since=f.m.recovery_since
    assert not f.m.result(r,f.now)  # duplicate cannot consume retry or change target
    f.now+=.26;effects=f.tick(False)
    requests=[d for k,d in effects if k=='compute']
    assert len(requests)==1 and requests[0]['id']!=request['id']
    assert requests[0]['operation']=='detect'
    assert not any(k=='trajectory' for k,_ in effects)
    late=f.compute(requests[0]);f.now+=.36;f.feed()
    assert not f.m.result(late,f.now)
    assert f.m.state=='BLOCKED' and f.m.reason=='DETECTION_RESULT_RETRY_EXHAUSTED'
    assert f.m.object_anchor is None and since is not None


def test_cancel_prevents_retry_and_late_target_admission():
    f,request=search();r=f.compute(request);f.now+=.36;f.feed()
    f.m.result(r,f.now);f.m.cancel(f.now,'OPERATOR_CANCEL')
    f.now+=.3;effects=f.tick(False)
    assert not any(k=='compute' for k,_ in effects)
    assert not f.m.result(r,f.now) and f.m.object_anchor is None


def test_retry_requires_zero_command_and_has_existing_recovery_deadline():
    for fail in ('motion','deadline'):
        f,request=search();r=f.compute(request);f.now+=.36;f.feed();f.m.result(r,f.now)
        if fail=='motion':f.r3.update(phase='RL_ACTIVE',motion_command=[.1,0,0]);f.now+=.3
        else:f.now+=31
        effects=f.tick(False)
        assert f.m.state=='BLOCKED'
        assert not any(k=='compute' for k,_ in effects)


def test_delivery_preserves_completion_timestamp_and_honors_cancellation():
    for cancelled in (False,True):
        sent=[];cancel=ComputeCancellation()
        r=dict(id=1,session_id='s',observed_ns=1,ok=True,found=False)
        cancel.begin(dict(r,operation='detect'))
        if cancelled:cancel.observe(dict(session_id='s',observed_ns=2,state='BLOCKED'))
        n=NS(completed=queue.Queue(1),busy=True,stop_event=threading.Event(),
            compute_cancel=cancel,controlled_stop=False,results=NS(publish=lambda msg:sent.append(json.loads(msg.data))))
        n.completed.put((r,json.dumps(r)))
        Perception.publish_completed(n)
        assert not n.busy
        assert len(sent)==(0 if cancelled else 1)
        if sent:
            assert sent[0]['observed_ns']==1 and sent[0]['publish_started_ns']>1
