"""Large-plan receipt is not a heartbeat; no ROS or hardware needed."""
import copy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from first_grasp_mission_test import Fixture


def pending(operation='grasp_plan'):
    f=Fixture();f.feed()
    f.m.nav_session='nav';f.m.owner_session='arm';f.m.r3_session='r3'
    f.m.state='PLAN_GRASP';f.m.request(operation,f.now)
    request=f.m.effects[-1][1];result=f.compute(request)
    return f,result


def test_large_plan_after_transport_delay_advances_to_live_pregrasp_check():
    for delay in (.251,1.5,5.):
        f,r=pending();assert f.m.result(r,f.now+delay)
        assert f.m.state=='PREGRASP'
        assert f.m.pending['kind']=='compute' and f.m.pending['operation']=='pregrasp'
        assert not any(kind=='trajectory' for kind,_ in f.m.effects)
        assert f.m.last_result_diagnostic['accepted']


def test_other_request_session_and_duplicate_results_cannot_advance():
    for key in ('id','session_id','navigation_session_id','owner_session_id','goal_revision'):
        f,r=pending();before=copy.deepcopy(f.m.pending)
        r[key]='wrong'
        assert not f.m.result(r,f.now+1)
        assert f.m.pending==before and f.m.state=='PLAN_GRASP'
        assert f.m.last_result_diagnostic['reason']=='REQUEST_IDENTITY_MISMATCH'
    f,r=pending();assert f.m.result(r,f.now+1)
    before=copy.deepcopy(f.m.pending)
    assert not f.m.result(r,f.now+1.1) and f.m.pending==before


def test_cancelled_plan_cannot_be_resurrected():
    f,r=pending();f.m.cancel(f.now,'OPERATOR_CANCEL')
    assert not f.m.result(r,f.now+1)
    assert f.m.state=='CANCELLED'


def test_stale_observation_and_bad_timestamps_do_not_hang_pending():
    f,r=pending('detect')
    assert not f.m.result(r,f.now+.251)
    assert f.m.state=='BLOCKED' and f.m.reason=='COMPUTE_RESULT_STALE'
    assert f.m.pending is None
    for stamp in (None,-1,int(1000e9)):
        f,r=pending();r['observed_ns']=stamp
        assert not f.m.result(r,f.now)
        assert f.m.state=='BLOCKED' and f.m.reason=='INVALID_COMPUTE_TIMESTAMP'


def test_recovery_timeout_still_blocks_delayed_plan():
    f,r=pending();f.m.recovery_since=f.now-31;f.m.recovery_reason='RGBD_STALE_OR_REPEATED'
    assert not f.m.result(r,f.now+1)
    assert f.m.state=='BLOCKED'
    assert f.m.reason.startswith('OBSERVATION_RECOVERY_TIMEOUT')
