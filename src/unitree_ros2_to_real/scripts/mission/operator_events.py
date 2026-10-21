"""Read-only operator projection; no actuator effects and no FSM ownership."""
import json


class OperatorEvents:
    FIELDS={
        'r3':('state','phase','fault','fault_phase','fault_blockers','stop_blocker','phase_warnings'),
        'mission':('state','reason','pending','availability'),
        'nav':('navigation_ready','block_reason','path_recovery','new_goal_required'),
        'perception':('model_ready','camera_state','scene_ready','reason'),
        'arm':('owner_state','last_error','feedback_ready','arm_home_ready'),
    }
    def __init__(self):self.last={}

    def observe(self, name, data, now_ns, ros_ns):
        fields={key:data.get(key) for key in self.FIELDS[name]}
        fields['decode_error']=data.get('decode_error')
        # Pending at/id changes describe real work; producer timestamps and
        # continuously changing ages are evidence, not operator state changes.
        pending=fields.get('pending')
        if isinstance(pending,dict):fields['pending']={k:pending.get(k) for k in ('id','operation','kind')}
        fingerprint=json.dumps(fields,sort_keys=True,default=str)
        if self.last.get(name)==fingerprint:return None
        self.last[name]=fingerprint
        return dict(subsystem=name,monotonic_ns=now_ns,ros_ns=ros_ns,view=fields,
                    evidence={key:data.get(key) for key in ('observed_ns','session_id','controller_session_id','owner_session_id',
                        'navigation_session_id','goal_revision','input_rejections','stale_input_diagnostic','streams','tf_pair') if key in data})
