"""Read-only operator projection; no actuator effects and no FSM ownership."""
import json


class OperatorEvents:
    FIELDS={
        'r3':('state','phase','fault','fault_latched','fault_phase','fault_blockers','stop_blocker','phase_warnings'),
        'mission':('state','reason','pending','availability','blockers','gripper_collision_policy'),
        'nav':('navigation_ready','block_reason','path_recovery','new_goal_required'),
        'perception':('ready','model_ready','camera_mode','scene_ready','reason'),
        'arm':('owner_state','last_error','feedback_ready','arm_home_ready','mission_blocker'),
    }
    def __init__(self):self.last={};self.levels={}

    def observe(self, name, data, now_ns, ros_ns):
        if not isinstance(data,dict):raise ValueError('STATUS_MAPPING_REQUIRED')
        fields={key:data.get(key) for key in self.FIELDS[name]}
        fields['decode_error']=data.get('decode_error')
        # Pending at/id changes describe real work; producer timestamps and
        # continuously changing ages are evidence, not operator state changes.
        pending=fields.get('pending')
        if isinstance(pending,dict):fields['pending']={k:pending.get(k) for k in ('id','operation','kind')}
        fingerprint=json.dumps(fields,sort_keys=True,default=str)
        if self.last.get(name)==fingerprint:return None
        self.last[name]=fingerprint
        level,cause=classify(name,fields)
        previous=self.levels.get(name);self.levels[name]=level
        event_type='RECOVERED' if level=='OK' and previous in ('WARNING','BLOCKED','FAULT') else level
        return dict(subsystem=name,level=level,event_type=event_type,cause=cause,monotonic_ns=now_ns,ros_ns=ros_ns,view=fields,
                    evidence={key:data.get(key) for key in ('observed_ns','session_id','controller_session_id','owner_session_id',
                        'navigation_session_id','goal_revision','input_rejections','stale_input_diagnostic','streams','tf_pair') if key in data})


def classify(name,view):
    """Project owner facts for the operator; this does not request transitions."""
    if view.get('fault_latched') or view.get('state')=='EMERGENCY_FAULT' or view.get('owner_state')=='FAULT_LATCHED':
        return 'FAULT',view.get('fault') or view.get('last_error') or 'fault_latched'
    if view.get('decode_error'):return 'BLOCKED',view['decode_error']
    if view.get('state')=='BLOCKED' or view.get('blockers'):
        return 'BLOCKED',view.get('blockers') or view.get('reason') or 'operation_blocked'
    if view.get('stop_blocker'):return 'BLOCKED',view['stop_blocker']
    if view.get('mission_blocker'):return 'BLOCKED',view['mission_blocker']
    if name=='nav' and view.get('navigation_ready') is False:
        return 'BLOCKED',view.get('block_reason') or 'navigation_unavailable'
    if name=='perception' and view.get('ready') is False:
        return 'BLOCKED',view.get('reason') or 'perception_unavailable'
    if name=='arm' and view.get('feedback_ready') is False:
        return 'BLOCKED','arm_feedback_unavailable'
    availability=view.get('availability') or {}
    if availability.get('navigation') is False:return 'BLOCKED',view.get('reason') or 'navigation_unavailable'
    if availability.get('manipulation') is False:return 'WARNING',view.get('reason') or 'manipulation_unavailable'
    if view.get('phase_warnings'):return 'WARNING',view['phase_warnings']
    return 'OK',''


def render_event(event):
    view=event['view'];parts=[event['subsystem'].upper(),event['event_type']]
    for key in ('state','phase','owner_state'):
        if view.get(key):parts.append(key+'='+str(view[key]))
    if event.get('cause'):parts.append('cause='+json.dumps(event['cause'],ensure_ascii=False))
    pending=view.get('pending')
    if pending:parts.append('operation='+str(pending.get('operation') or pending.get('kind')))
    diagnostic=event.get('evidence',{}).get('stale_input_diagnostic') or {}
    for key in ('source_topic','source_stamp','status_age_ms','ttl_ms','action_taken'):
        if diagnostic.get(key) is not None:parts.append(key+'='+str(diagnostic[key]))
    return ' | '.join(parts)
