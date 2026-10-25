"""Controlled-stop HOME handshake. Does not grant actuator authority."""
from .health_contract import CONTROLLED_HOME_PHASES


def geometry_home_ready(*, controlled_stop, phase, cancel_complete, backend_ready,
                        scene_error, reloading=False):
    return bool(controlled_stop and phase in CONTROLLED_HOME_PHASES and cancel_complete
                and backend_ready and not scene_error and not reloading)


def home_dispatch_blocker(engine, now, *, physical, blockers, cancel_ack, services_ready,
                          unique_publishers):
    r3=engine.inputs.get('r3',{});owner=engine.inputs.get('owner',{})
    p=engine.inputs.get('perception',{})
    if r3.get('state')!='CONTROLLED_STOP' or r3.get('phase') not in CONTROLLED_HOME_PHASES:return 'OUTSIDE_HOME_PHASE'
    if r3.get('fault_latched'):return 'FAULT_LATCHED'
    if not physical or blockers:return 'PHYSICAL_NOT_READY'
    for source in ('r3','owner','perception','nav'):
        if not engine.fresh(source,now):return 'STALE_'+source.upper()
    if not services_ready or not unique_publishers:return 'SERVICE_OR_PUBLISHER_NOT_READY'
    if not cancel_ack:return 'WAIT_CANCEL_ACK'
    if not owner.get('home_requested'):return 'WAIT_OWNER_HOME_REQUEST'
    if not all(owner.get(k) is True for k in ('mission_capable','feedback_ready','target_valid')):return 'OWNER_FEEDBACK_NOT_READY'
    if not p.get('home_compute_ready'):return 'WAIT_GEOMETRY_READY:'+str(p.get('reason','UNKNOWN'))
    if p.get('busy',True):return 'WAIT_PLANNER_IDLE'
    return ''


class HomeDispatch:
    """Retry unacknowledged publication with SAME identity; worker deduplicates it."""
    def __init__(self):
        self.request=None;self.sent_at=None;self.accepted=False;self.finished=False
    def due(self, now, perception, allowed):
        if self.request is None or self.finished:return False
        if (perception.get('active_request_id')==self.request['id']
                and perception.get('active_request_session')==self.request['session_id']):
            self.accepted=True
        return bool(allowed and not self.accepted and (self.sent_at is None or now-self.sent_at>=.5))
