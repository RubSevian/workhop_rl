"""Semantic diagnostic events only; no actuator effects or FSM ownership."""
import json
import re
import math

# Only these discrete producer facts participate in event identity. In
# particular, never recursively fingerprint a complete diagnostic snapshot.
COMMON = ('state','phase','mode','status','ready','reason','error','errors',
          'warning','warnings','blocked','block_reason','blockers','decode_error')
FIELDS = {
    'r3': ('operation_profile','command_source','control_mode','fault','fault_latched',
           'fault_phase','fault_blockers','stop_blocker','phase_warnings','arm_error',
           'arm_ready','arm_home_ready','arm_control_ready','navigation_ready','perception_ready',
           'own_output_healthy','pd_hold_confirmed','failed_arm_stop','output_enabled',
           'output_stop_confirmed','remote_sequence_active','remote_sequence_message',
           'remote_sequence_blockers','sdk_release_pending','policy_zero_handoff_pending',
           'arm_home_port','arm_emergency_port','arm_planned_disable_port',
           'navigation_cancel_port','manipulation_cancel_port'),
    'mission': ('test_mode','operator_trial','retreat_operator_approval_required','availability','pending','grasp_assessment','grasp_attempt',
                'last_grasp_error','stability_reason','observation_recovery','gripper_collision_policy'),
    'nav': ('input_mode','navigation_ready','command_enabled','block_reason','path_recovery',
            'new_goal_required','cancel_latched','far_cancel_pending','tf_error','tf_pair','goal'),
    'perception': ('model_ready','camera_mode','camera_state','scene_ready','scene_error',
                   'preload_ready','startup_checks','controlled_stop_geometry_only','busy','last_compute'),
    'arm': ('owner_state','last_error','feedback_ready','arm_home_ready','motors_enabled',
            'mission_capable','mission_blocker','mission_id','mission_state','mission_error',
            'grasp_verification','grasp_assessment','grasp_assessment_reason','home_requested',
            'gripper_command_active','gripper_hold_confirmed','watchdog_tripped','stm32_watchdog_tripped',
            'planned_disable_attempted','planned_disable_sent','operator_trial','retreat_trial'),
}
# Allowlisted semantic fields inside arbitrary error/availability/port reports.
SEMANTIC_KEYS = frozenset(COMMON + ('code','source','subsystem','topic','operation','kind',
    'result','ok','accepted','enabled','active','navigation','manipulation','geometry','yolo',
    'graspnet','camera','model','feedback','owner','motor_id','joint_id','link','pair'))
NESTED = {
    'pending': ('id','operation','kind'),
    'goal': ('revision','state','reason','far_reason','result'),
    'last_compute': ('operation','ok','state','reason','error'),
    'observation_recovery': ('reason','retry_operation'),
    'tf_pair': ('reason','state','ready'),
    'retreat_trial': ('state','plan_id','approval_service'),
}
# Numeric details embedded in free-text diagnostics must not create periodic
# events. Keep identifiers such as motor7, joint1 and reason codes unchanged.
_DYNAMIC = re.compile(r'\b((?:[A-Za-z_]*(?:age|elapsed|timestamp|stamp|duration|count|frames|counter)[A-Za-z_]*|since|retries)\s*[=:]\s*)[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', re.IGNORECASE)


def semantic(value):
    if isinstance(value, dict):
        return {key: semantic(item) for key,item in sorted(value.items()) if key in SEMANTIC_KEYS}
    if isinstance(value, (list,tuple)):
        # Blockers/warnings are sets of facts; publisher ordering is irrelevant.
        items=[semantic(item) for item in value]
        unique={json.dumps(item,sort_keys=True,ensure_ascii=False):item for item in items}
        return [unique[key] for key in sorted(unique)]
    if isinstance(value, str):return _DYNAMIC.sub(r'\1<diagnostic>',value)
    if isinstance(value,float) and not math.isfinite(value):return str(value)
    return value


def projection(name, data):
    if not isinstance(data,dict):raise ValueError('STATUS_MAPPING_REQUIRED')
    for key in (*NESTED,'availability','startup_checks'):
        if key in data and data[key] is not None and not isinstance(data[key],dict):
            raise ValueError('STATUS_FIELD_MAPPING_REQUIRED:'+key)
    fields={key:semantic(data[key]) for key in dict.fromkeys(COMMON+FIELDS[name]) if key in data}
    for key,keys in NESTED.items():
        if key in fields and isinstance(data[key],dict):
            fields[key]={k:semantic(data[key][k]) for k in keys if k in data[key]}
    # These mappings describe named capabilities/checks, with boolean results.
    for key in ('availability','startup_checks'):
        if isinstance(data.get(key),dict):
            fields[key]={k:v for k,v in sorted(data[key].items()) if isinstance(v,bool)}
    return fields


class OperatorEvents:
    FIELDS=FIELDS
    def __init__(self):self.last={};self.levels={}

    def observe(self, name, data, now_ns, ros_ns):
        fields=projection(name,data)
        previous_view=self.last.get(name)
        if previous_view==fields:return None
        # Store a detached projection: publishers/tests may reuse and mutate data.
        self.last[name]=fields
        level,cause=classify(name,fields)
        previous=self.levels.get(name);self.levels[name]=level
        event_type='RECOVERED' if level=='OK' and previous in ('WARNING','BLOCKED','FAULT') else level
        changes={key:{'from':(previous_view or {}).get(key),'to':fields.get(key)}
                 for key in sorted(set(fields)|set(previous_view or {}))
                 if previous_view is None or (previous_view or {}).get(key)!=fields.get(key)}
        operations=operation_changes(name,previous_view,fields)
        return dict(subsystem=name,level=level,event_type=event_type,cause=cause,
                    initial=previous_view is None,changes=changes,operations=operations,monotonic_ns=now_ns,ros_ns=ros_ns,view=fields,
                    evidence={key:data[key] for key in ('observed_ns','session_id','controller_session_id','owner_session_id',
                        'navigation_session_id','goal_revision','input_rejections','stale_input_diagnostic','streams','tf_pair') if key in data})


def operation_changes(name,previous,current):
    """Report observed operation boundaries, without inferring physical success."""
    previous=previous or {};out=[]
    if name=='mission' and previous.get('pending')!=current.get('pending'):
        if previous.get('pending'):out.append(dict(event='ENDED',**previous['pending']))
        if current.get('pending'):out.append(dict(event='STARTED',**current['pending']))
    if name=='arm':
        old=(previous.get('mission_id'),previous.get('mission_state'))
        new=(current.get('mission_id'),current.get('mission_state'))
        if old!=new and new[1] in ('EXECUTING','SUCCEEDED','FAILED','CANCELLED'):
            out.append(dict(event='STARTED' if new[1]=='EXECUTING' else 'ENDED',id=new[0],result=new[1]))
    if name=='perception':
        if current.get('busy') is True and previous.get('busy') is not True:
            out.append(dict(event='STARTED',operation='compute'))
        if (previous.get('busy') is True and current.get('busy') is False) or (current.get('last_compute') and previous.get('last_compute')!=current.get('last_compute')):
            out.append(dict(event='ENDED',operation=(current.get('last_compute') or {}).get('operation','compute')))
    return out


def classify(name,view):
    """Project published owner facts; never request or manufacture a transition."""
    if view.get('fault_latched') or view.get('state')=='EMERGENCY_FAULT' or view.get('owner_state')=='FAULT_LATCHED':
        return 'FAULT',view.get('fault') or view.get('last_error') or 'fault_latched'
    if view.get('decode_error'):return 'BLOCKED',view['decode_error']
    for key in ('stop_blocker','mission_blocker','blockers','block_reason'):
        if view.get(key):return 'BLOCKED',view[key]
    if view.get('blocked') or view.get('state')=='BLOCKED':
        return 'BLOCKED',view.get('reason') or 'operation_blocked'
    if name=='arm' and view.get('mission_state')=='FAILED':
        return 'BLOCKED',view.get('mission_error') or 'arm_operation_failed'
    goal=view.get('goal') or {}
    if name=='nav' and goal.get('state') in ('FAILED','GOAL_UNREACHABLE','STOPPED_ADJUSTED'):
        return 'BLOCKED',goal.get('far_reason') or goal.get('reason') or goal['state']
    if name=='nav' and view.get('navigation_ready') is False:
        return 'BLOCKED',view.get('tf_error') or 'navigation_unavailable'
    if name=='perception' and view.get('ready') is False:
        return 'BLOCKED',view.get('reason') or view.get('scene_error') or 'perception_unavailable'
    if name=='arm' and view.get('feedback_ready') is False:
        return 'BLOCKED',view.get('last_error') or 'arm_feedback_unavailable'
    availability=view.get('availability') or {}
    if availability.get('navigation') is False:return 'BLOCKED',view.get('reason') or 'navigation_unavailable'
    if availability.get('manipulation') is False:return 'WARNING',view.get('reason') or 'manipulation_unavailable'
    for key in ('phase_warnings','warnings','warning','error','errors','scene_error','tf_error','last_error','arm_error'):
        if view.get(key):return 'WARNING',view[key]
    return 'OK',''


def render_event(event):
    parts=[event['subsystem'].upper(),event['event_type']]
    changes=event.get('changes',{})
    for key,change in changes.items():
        # Show every changed fact, including false values, clears and modes.
        fmt=lambda value:json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(',',':'))
        value=fmt(change['to']) if event.get('initial') else fmt(change['from'])+' -> '+fmt(change['to'])
        parts.append(key+'='+value)
    for operation in event.get('operations',[]):parts.append('operation_'+operation['event'].lower()+'='+json.dumps({key:value for key,value in operation.items() if key!='event'},ensure_ascii=False,sort_keys=True))
    if event.get('cause'):parts.append('cause='+json.dumps(event['cause'],ensure_ascii=False))
    diagnostic=event.get('evidence',{}).get('stale_input_diagnostic') or {}
    for key in ('source_topic','source_stamp','status_age_ms','ttl_ms','action_taken'):
        if diagnostic.get(key) is not None:parts.append(key+'='+str(diagnostic[key]))
    return ' | '.join(parts)
