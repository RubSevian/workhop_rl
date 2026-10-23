"""Pure mission FSM. Effects are serviced outside R3 and outside inference.

All times here are host MONOTONIC. ROS capture time stays in stamped TF data.
No command is sent merely because an old SUCCEEDED is replayed.
"""
import copy
import math
from .scene import matrix_pose, validate_scene_data
from .transforms import pose_distance
from .scene_wait import transient_scene_error
from .health_contract import source_ttl, input_diagnostic

OBSERVATIONS = {'detect', 'observe_grasp', 'grasp_plan'}

def retryable_observation(reason):
    return transient_scene_error(reason) or reason in ('RGBD_STALE_OR_REPEATED', 'RGBD_TF_FEEDBACK_STALE')

TERMINAL = {'MISSION_COMPLETED_UNVERIFIED','GRASP_PRESUMED_SUCCESS','TARGET_NOT_GRASPED', 'OBJECT_NOT_FOUND', 'GRASP_UNREACHABLE', 'BLOCKED', 'CANCELLED'}
ARM_PHASES = {'BASE_SETTLE','ARM_SCAN_POSE','ARM_SEARCH','VIEW_REFINEMENT','PLAN_GRASP',
              'PREGRASP','GRASP','WAIT_RETREAT_APPROVAL','RETREAT','RETURN_HOME','RETURN_SEARCH'}

class Mission:
    def __init__(self, cfg, session):
        self.cfg, self.session = cfg, session
        self.ready_observe=cfg.get('test_mode','full')=='ready_observe'
        self.last_observation=None
        self.state, self.reason = 'IDLE', 'DISARMED'
        self.entered = self.started = 0.
        self.inputs, self.received = {}, {}
        self.effects, self.events = [], []
        self.nav_session = self.r3_session = self.owner_session = ''
        self.revision = self.goal_stamp = self.last_revision = 0
        self.armed_ros_ns = 0
        self.request_id = 0
        self.pending = None
        self.scan_index, self.detect_at = 0, 0.
        self.stable_since, self.base_anchor = None, None
        self.leg_anchor = None
        self.scene_anchor=None;self.goal_reached=None;self.window_pose=None
        self.stability_reason='';self.stability_restarts=0
        self.result_after_home = 'MISSION_COMPLETED_UNVERIFIED'
        self.grasp_plan = None
        self.object_anchor=None;self.q_search=None;self.grasp_attempt=0
        self.attempt_segments=[];self.last_grasp_error=''
        self.last_now = 0.
        self.last_compute_metrics = {}
        self.stale_input_diagnostic=None
        self.wait_goal_stale_since=None
        self.recovery_since=None;self.recovery_reason='';self.retry_operation=None;self.recovery_retry_active=False
        self.retry_at=0.;self.observation_retries=0
        self.input_rejections={}
        self.availability={}
        self.new_goal_after_recovery=False

    def observe(self, name, data, now):
        # Duplicate/out-of-order or future producer samples never refresh leases.
        stamp = data.get('observed_ns')
        ttl=source_ttl(self.cfg,name)
        if type(stamp) is not int or not 0 <= now-stamp/1e9 <= ttl:
            reason=('INVALID_STAMP' if type(stamp) is not int else 'FUTURE_STAMP' if now<stamp/1e9 else 'SOURCE_STATUS_STALE')
            self.input_rejections[name]=input_diagnostic(self.cfg,name,data,now,phase=self.state,reason=reason,action='REJECT_INPUT')
            return False
        previous=self.inputs.get(name,{})
        if name=='scene' and stamp==previous.get('observed_ns'):
            # Same measured geometry can remain valid while newer, real joint
            # receipts confirm stream health. Refresh only those receipts,
            # never the geometry timestamp or its lease.
            fixed=('body','q7','legs','feedback_ns','legs_ns','owner_session_id','frame','world_frame','anchor_ns')
            if any(data.get(key)!=previous.get(key) for key in fixed):return False
            keys=('live_feedback_ns','live_legs_ns')
            if any(type(data.get(key)) is not int
                   or data[key]<previous.get(key,0)
                   or not 0<=now-data[key]/1e9<=(self.cfg['limits'].get('legs_age_s',self.cfg['limits']['arm_age_s']) if key=='live_legs_ns' else self.cfg['limits']['arm_age_s'])
                   for key in keys):return False
            for key in keys:previous[key]=data[key]
            return True
        if stamp <= self.inputs.get(name, {}).get('observed_ns', 0):
            self.input_rejections[name]=input_diagnostic(self.cfg,name,data,now,phase=self.state,reason='DUPLICATE_OR_OUT_OF_ORDER',action='REJECT_INPUT')
            return False
        self.inputs[name] = copy.deepcopy(data)
        self.received[name] = stamp/1e9
        return True

    def fresh(self, name, now):
        ttl=source_ttl(self.cfg,name)
        return name in self.received and 0 <= now-self.received[name] <= ttl

    def wait_goal_step(self, now, ros_now_ns, active):
        """Recoverable task admission; this never grants leg or arm authority."""
        for name in ('r3','owner'):
            if not self.fresh(name,now):
                if self.stale_input_diagnostic is None:
                    self.stale_input_diagnostic=input_diagnostic(self.cfg,name,self.inputs.get(name,{}),now,
                        phase=self.state,reason='STALE_INPUT:'+name,action='CANCEL_MISSION')
                    self.stale_input_diagnostic.update(state_before=self.state,stale_sources=[name],
                        inputs={name:dict(accepted_stamp_ns=self.inputs.get(name,{}).get('observed_ns'),
                            accepted_age_s=now-self.received[name] if name in self.received else None)})
                self.cancel(now,'STALE_INPUT:'+name,True);return self.drain()
        r3,owner=self.inputs['r3'],self.inputs['owner']
        if not active:
            self.cancel(now,'R3_AUTHORITY_REVOKED');return self.drain()
        if (r3.get('controller_session_id')!=self.r3_session or owner.get('owner_session_id')!=self.owner_session
                or r3.get('navigation_session_id')!=self.nav_session):
            self.cancel(now,'SESSION_CHANGED');return self.drain()
        if not owner.get('feedback_ready') or not owner.get('target_valid'):
            self.cancel(now,'ARM_FEEDBACK_OR_TARGET_INVALID',True);return self.drain()
        nav=self.inputs.get('nav',{})
        nav_ok=self.fresh('nav',now) and nav.get('navigation_ready') is True
        vision_ok=self.fresh('perception',now) and self.inputs['perception'].get('ready') is True
        self.availability={'navigation':bool(nav_ok),'manipulation':bool(vision_ok)}
        if not nav_ok:
            reason='NAV_STATUS_UNAVAILABLE' if not self.fresh('nav',now) else 'NAV_LOCALIZATION_UNAVAILABLE'
            if self.wait_goal_stale_since is None:
                self.wait_goal_stale_since=now;self.new_goal_after_recovery=True
                if self.stale_input_diagnostic is None:
                    self.stale_input_diagnostic=input_diagnostic(self.cfg,'nav',nav,now,phase=self.state,reason=reason,action='BLOCK_NEW_GOAL')
                    self.stale_input_diagnostic.update(state_before=self.state,stale_sources=['nav'])
                self.effects.append(('invalidate_navigation_goal',{'reason':reason}))
                self.events.append(dict(state=self.state,time=now,reason=reason,severity='BLOCKED',operation='NAVIGATION'))
            self.reason=reason
            return self.drain()
        if nav.get('session_id')!=self.nav_session:
            self.cancel(now,'SESSION_CHANGED');return self.drain()
        if self.new_goal_after_recovery:
            self.last_revision=nav['goal']['revision'];self.armed_ros_ns=ros_now_ns
            self.wait_goal_stale_since=None;self.new_goal_after_recovery=False
            self.reason='RECOVERED_NEW_GOAL_REQUIRED'
            self.events.append(dict(state=self.state,time=now,reason=self.reason,severity='RECOVERED',operation='NAVIGATION'))
            return self.drain()
        self.reason='' if vision_ok else 'PERCEPTION_STATUS_UNAVAILABLE' if not self.fresh('perception',now) else 'PERCEPTION_NOT_READY'
        goal=nav['goal']
        if goal['revision']>self.last_revision and goal.get('stamp_ns',0)>self.armed_ros_ns and goal['state'] in ('REQUESTED','ACTIVE'):
            self.started=now;self.revision,self.goal_stamp=goal['revision'],goal['stamp_ns']
            self.transition('NAVIGATING',now)
        return self.drain()

    def scene_error(self,now):
        try:validate_scene_data(self.inputs['scene'],self.cfg,int(now*1e9),self.nav_session,self.owner_session)
        except (ValueError,KeyError,TypeError) as error:return str(error)
        return ''

    def transition(self, state, now, reason=''):
        self.state, self.entered, self.reason = state, now, reason
        self.events.append(dict(state=state, time=now, reason=reason, revision=self.revision))

    def cancel(self, now, reason, blocked=False):
        self.pending = None  # late CUDA/trajectory results cannot advance this FSM
        self.grasp_plan=None
        self.effects.append(('cancel', {'reason': reason}))
        self.transition('BLOCKED' if blocked else 'CANCELLED', now, reason)

    def request(self, operation, now):
        if self.ready_observe and operation not in ('scan_pose','observe_grasp','home'):
            raise ValueError('READY_OBSERVE_MOTION_FORBIDDEN')
        self.request_id += 1
        self.pending = dict(id=self.request_id, operation=operation, kind='compute', at=now)
        self.effects.append(('compute', dict(self.pending, session_id=self.session,
            navigation_session_id=self.nav_session, goal_revision=self.revision,
            owner_session_id=self.owner_session, owner=copy.deepcopy(self.inputs['owner']),
            grasp_plan=self.grasp_plan, object_anchor=self.object_anchor,q_search=self.q_search,
            grasp_attempt=self.grasp_attempt,reverse_segment=copy.deepcopy(self.attempt_segments[-1]) if operation in ('reverse_search','retry_open') and self.attempt_segments else None,
            scan_index=self.scan_index)))

    def failed_grasp_attempt(self, reason, now):
        self.last_grasp_error=reason
        if self.inputs['owner'].get('gripper_command_active') and self.inputs['owner'].get('object_hold_state')!='EMPTY' and self.inputs['owner'].get('grasp_assessment')!='NOT_GRASPED':
            self.cancel(now,'OBJECT_HOLD_UNKNOWN_NO_AUTOMATIC_RELEASE',True);return
        if self.grasp_attempt>=2:
            self.go_home('TARGET_NOT_GRASPED',now);return
        self.grasp_attempt+=1;self.grasp_plan=None
        if self.attempt_segments:
            self.transition('RETURN_SEARCH',now,reason)
            self.request('retry_open' if self.inputs['owner'].get('gripper_command_active') else 'reverse_search',now)
        elif self.q_search is not None and max(abs(a-b) for a,b in zip(self.inputs['owner']['q'][:6],self.q_search))<=self.cfg['execution']['start_tolerance_rad']:
            self.transition('PLAN_GRASP',now,reason);self.request('grasp_plan',now)
        else:self.cancel(now,'Q_SEARCH_RETURN_PATH_MISSING',True)

    def defer_observation(self, reason, now, operation=None):
        if self.recovery_since is None:self.recovery_since=now
        self.recovery_reason=reason
        if operation is not None:
            self.recovery_retry_active=True
            self.retry_operation=operation
            self.retry_at=now+self.cfg['execution'].get('observation_retry_delay_s',.25)
            self.observation_retries+=1

    def clear_recovery(self):
        self.recovery_since=None;self.recovery_reason='';self.retry_operation=None;self.recovery_retry_active=False

    def wait_for_observation(self, now):
        """Wait only at rest, never renew a scene lease or resume a trajectory."""
        if self.state in ('WAIT_GOAL','NAVIGATING'):return False
        perception=self.inputs['perception']
        reason='' if perception.get('ready') else perception.get('reason','PERCEPTION_INVALID')
        stationary=(self.state in ('BASE_SETTLE','ARM_SEARCH','PLAN_GRASP')
                    and (not self.pending or (self.pending['kind']=='compute'
                         and self.pending['operation'] in OBSERVATIONS)))
        if stationary:
            r3=self.inputs['r3'];command=r3.get('motion_command',[])
            stationary=(r3.get('phase')=='RL_ZERO' and len(command)==3
                        and all(math.isfinite(v) and abs(v)<=1e-6 for v in command)
                        and self.fresh('legs',now))
        if not reason and stationary:
            if not self.fresh('base',now):reason='SCENE_SNAPSHOT_STALE:body'
            elif not self.fresh('scene',now):reason='SCENE_SNAPSHOT_STALE:body'
            else:reason=self.scene_error(now)
        # Waiting for a goal/navigation can tolerate perception geometry lag,
        # but NAV readiness/owner/R3 authority are checked separately first.
        eligible=stationary or self.state in ('WAIT_GOAL','NAVIGATING')
        if reason:
            if not eligible or not transient_scene_error(reason):
                self.cancel(now,reason,True);return True
            self.defer_observation(reason,now)
        if self.retry_operation and not stationary:
            self.cancel(now,'OBSERVATION_RETRY_NOT_STATIONARY',True);return True
        if self.recovery_since is not None:
            if now-self.recovery_since>=self.cfg['execution'].get('observation_recovery_timeout_s',30.):
                self.cancel(now,'OBSERVATION_RECOVERY_TIMEOUT:'+self.recovery_reason,True);return True
            if now-self.entered>self.cfg['execution']['state_timeout_s'] and self.state in ARM_PHASES:
                self.cancel(now,'PHASE_TIMEOUT',True);return True
            if reason:
                return True
            if self.retry_operation:
                if now>=self.retry_at and not self.pending:
                    operation=self.retry_operation;self.retry_operation=None
                    self.request(operation,now)
                return True
            # Preserve the deadline only for an actual retry after a failed
            # observation, not an original compute with a recovered health blip.
            if (self.recovery_retry_active and self.pending
                    and self.pending['operation'] in OBSERVATIONS):return False
            self.clear_recovery()
        return False

    def perception_service_ready(self, now):
        # Coordinator health means it can supervise a bounded stationary retry;
        # perception's own ready flag and all measurement stamps stay unchanged.
        if not self.fresh('perception',now):return False
        data=self.inputs['perception']
        return bool(data.get('ready') or (
            self.state not in TERMINAL and self.recovery_since is not None
            and now-self.recovery_since<self.cfg['execution'].get('observation_recovery_timeout_s',30.)
            and transient_scene_error(data.get('reason',''))))

    def done_motion(self, operation, now):
        if operation == 'scan_pose':
            self.transition('ARM_SEARCH', now)
            self.detect_at = now + self.cfg['search']['settle_s']
        elif operation == 'scan_step':
            self.detect_at = now + self.cfg['search']['settle_s']
        elif operation == 'scan_return':
            self.go_home('OBJECT_NOT_FOUND', now)
        elif operation == 'refine':
            self.transition('PLAN_GRASP', now)
            self.detect_at = now + self.cfg['search']['settle_s']
        elif operation == 'pregrasp':
            self.transition('GRASP', now)
            self.request('approach', now)
        elif operation == 'approach':
            self.request('close', now)
        elif operation == 'close':
            assessment=self.inputs['owner'].get('grasp_assessment','UNKNOWN')
            if assessment=='GRASPED':
                self.transition('WAIT_RETREAT_APPROVAL' if self.cfg.get('operator_trial') else 'RETREAT',now,
                    'OPERATOR_RETREAT_APPROVAL_REQUIRED' if self.cfg.get('operator_trial') else '')
                self.request('retreat',now)
            elif assessment=='NOT_GRASPED':self.failed_grasp_attempt('TORQUE_WIDTH_NOT_SATISFIED',now)
            else:self.cancel(now,'GRASP_ASSESSMENT_UNKNOWN:'+self.inputs['owner'].get('grasp_assessment_reason','NO_VALID_ASSESSMENT'),True)
        elif operation=='retry_open':
            self.request('reverse_search',now)
        elif operation=='reverse_search':
            self.attempt_segments.pop()
            if self.attempt_segments:self.request('reverse_search',now)
            elif self.q_search is not None and max(abs(a-b) for a,b in zip(self.inputs['owner']['q'][:6],self.q_search))<=self.cfg['execution']['start_tolerance_rad']:
                self.transition('PLAN_GRASP',now,'RETRY_AT_Q_SEARCH');self.request('grasp_plan',now)
            else:self.cancel(now,'Q_SEARCH_NOT_REACHED',True)
        elif operation == 'retreat':
            self.go_home('GRASP_PRESUMED_SUCCESS', now)
        elif operation == 'home':
            owner = self.inputs['owner']
            if not owner.get('arm_home_ready'):
                self.cancel(now, 'HOME_UNCONFIRMED', True)
                return
            self.transition(self.result_after_home, now)

    def go_home(self, result, now):
        self.result_after_home = result
        self.transition('RETURN_HOME', now, result)
        self.request('home', now)

    def next_search(self, now):
        self.scan_index += 1
        if self.scan_index >= self.cfg['search']['max_attempts']:
            self.result_after_home='OBJECT_NOT_FOUND'
            self.transition('RETURN_HOME',now,'RETURN_SEARCH_TO_ZERO')
            self.request('scan_return',now)
        else:
            if self.state!='ARM_SEARCH':self.transition('ARM_SEARCH', now, 'NEXT_OBSERVATION')
            self.request('scan_step', now)

    def result(self, result, now):
        p = self.pending
        if (not p or p['kind'] != 'compute' or result.get('id') != p['id']
                or result.get('session_id') != self.session
                or result.get('navigation_session_id') != self.nav_session
                or result.get('goal_revision') != self.revision
                or result.get('owner_session_id') != self.owner_session):
            return False
        age = now-result.get('observed_ns', 0)/1e9
        # Trajectory transport/decoding is not a heartbeat. Its request identity
        # above and the owner's live admission checks determine applicability.
        if result.get('observed_ns',0)<=0 or not age>=0:
            return False
        if 'trajectory' not in result and age>self.cfg['execution']['heartbeat_ttl_s']:
            return False
        if (self.recovery_since is not None
                and now-self.recovery_since>=self.cfg['execution'].get('observation_recovery_timeout_s',30.)):
            self.cancel(now,'OBSERVATION_RECOVERY_TIMEOUT:'+self.recovery_reason,True)
            return False
        self.last_compute_metrics=result.get('compute_metrics',{})
        operation = p['operation']
        self.pending = None
        if not result.get('ok'):
            reason = result.get('reason', 'PLAN_REJECTED')
            if operation in OBSERVATIONS and retryable_observation(reason):
                self.defer_observation(reason,now,operation)
                return True
            if operation=='grasp_plan' and reason=='OBJECT_LOST_AFTER_REFINEMENT':
                self.next_search(now)
                return True
            # Unsafe HOME never falls through to a direct zero joint target.
            if operation=='grasp_plan' and (reason in ('IK_FAIL','NO_CANDIDATES','CANDIDATE_GEOMETRY_REJECTED') or reason.startswith('NO_COLLISION_FREE_CANDIDATE:')):
                # Planning rejection is diagnostic: wait for the operator.
                # Physical failed-grasp retries retain their existing policy.
                self.cancel(now, reason, True)
            else:
                self.cancel(now, reason, True)
            return True
        if operation in OBSERVATIONS:self.clear_recovery()
        if operation == 'observe_grasp':
            self.last_observation=result.get('observation',{})
            self.detect_at=now+self.cfg['search']['settle_s']
        elif operation == 'detect':
            if result.get('found'):
                self.object_anchor=copy.deepcopy(result.get('object_anchor'))
                self.q_search=list(result.get('q_search',self.inputs['owner']['q'][:6]))
                self.grasp_attempt=1
                self.attempt_segments=[]
                refine=self.cfg['refinement'].get('enabled',True)
                self.transition('VIEW_REFINEMENT' if refine else 'PLAN_GRASP', now)
                self.request('refine' if refine else 'grasp_plan', now)
            else:
                self.next_search(now)
        elif operation == 'grasp_plan':
            if result.get('object_anchor') is not None:self.object_anchor=copy.deepcopy(result['object_anchor'])
            self.grasp_plan = result['grasp_plan']
            self.transition('PREGRASP', now)
            self.request('pregrasp', now)
        else:
            # The owner does its own bounds/authority/start validation. It does
            # not take arbitrary joint targets from a CUDA process directly.
            self.pending = dict(p, kind='motion', at=now,
                                trajectory=copy.deepcopy(result['trajectory']),
                                scene=copy.deepcopy(result['trajectory'].get('scene_snapshot',self.inputs.get('scene'))))
            self.effects.append(('trajectory', dict(result['trajectory'], id=p['id'],operation=operation,
                session_id=self.session, navigation_session_id=self.nav_session,
                goal_revision=self.revision, owner_session_id=self.owner_session)))
        return True

    def step(self, now, ros_now_ns):
        self.last_now = now
        if self.entered==0:self.entered=now
        if self.state in TERMINAL:
            return self.drain()
        r3, nav, owner = (self.inputs.get(n, {}) for n in ('r3','nav','owner'))
        active = (r3.get('operation_profile') == 'full_mission' and r3.get('state') == 'ACTIVE'
                  and r3.get('phase') in ('RL_ZERO','RL_ACTIVE') and r3.get('capabilities', {}).get('arm_motion') is True
                  and not r3.get('fault_latched'))
        if self.state == 'IDLE':
            if not active or r3.get('phase')!='RL_ZERO' or not all(self.fresh(n, now) for n in ('r3','nav','owner')):
                return self.drain()
            self.nav_session, self.r3_session = nav['session_id'], r3['controller_session_id']
            self.owner_session = owner['owner_session_id']
            self.armed_ros_ns = ros_now_ns
            self.last_revision = nav['goal']['revision']
            self.started = now
            self.transition('WAIT_GOAL', now, 'new operator goal required')
            return self.drain()
        if self.state=='WAIT_GOAL':return self.wait_goal_step(now,ros_now_ns,active)
        if self.state=='BASE_SETTLE' and self.stable_since is None:
            command=r3.get('motion_command',[])
            if (self.fresh('r3',now) and r3.get('phase')=='RL_ZERO' and len(command)==3
                    and all(math.isfinite(x) and abs(x)<=1e-6 for x in command)
                    and r3.get('observed_ns',0)/1e9>=self.goal_reached):
                self.stable_since=now
        if (self.state=='NAVIGATING' and self.fresh('r3',now) and self.fresh('owner',now) and active
                and (not self.fresh('nav',now) or not nav.get('navigation_ready')
                     or nav.get('goal',{}).get('state')=='LOCALIZATION_LOST')):
            self.pending=None;self.grasp_plan=None
            self.effects.append(('invalidate_navigation_goal',{'reason':'NAV_LOCALIZATION_OR_STATUS_LOST'}))
            self.new_goal_after_recovery=True;self.wait_goal_stale_since=now
            self.transition('WAIT_GOAL',now,'NAV_LOCALIZATION_OR_STATUS_LOST')
            return self.drain()
        required=('r3','nav','owner') if self.state=='NAVIGATING' else ('r3','nav','owner','perception')
        if not all(self.fresh(n, now) for n in required):
            names=required
            diagnostic=dict(observed_ns=int(now*1e9),state_before=self.state,
                stale_sources=[n for n in names if not self.fresh(n,now)],
                heartbeat_ttl_s=self.cfg['execution']['heartbeat_ttl_s'],
                inputs={n:dict(accepted_stamp_ns=self.inputs.get(n,{}).get('observed_ns'),
                    lease_present=n in self.received,
                    ttl_s=source_ttl(self.cfg,n),
                    accepted_age_s=now-self.received[n] if n in self.received else None)
                    for n in names})
            if self.stale_input_diagnostic is None:self.stale_input_diagnostic=diagnostic
            stale=diagnostic['stale_sources']
            self.cancel(now, 'STALE_INPUT', True)
        elif not active:
            self.cancel(now, 'R3_AUTHORITY_REVOKED')
        elif (nav.get('session_id') != self.nav_session or r3.get('controller_session_id') != self.r3_session
              or owner.get('owner_session_id') != self.owner_session
              or r3.get('navigation_session_id') != self.nav_session):
            self.cancel(now, 'SESSION_CHANGED')
        elif not nav.get('navigation_ready') or not owner.get('feedback_ready') or not owner.get('target_valid'):
            self.cancel(now, 'TF_ODOM_ARM_OR_PERCEPTION_INVALID', True)
        elif self.state!='WAIT_GOAL' and now-self.started > self.cfg['execution']['mission_timeout_s']:
            self.cancel(now, 'MISSION_TIMEOUT', True)
        elif self.state!='WAIT_GOAL' and (nav['goal']['revision'] != self.revision or nav['goal'].get('stamp_ns')!=self.goal_stamp):
            self.cancel(now, 'GOAL_REPLACED')
        elif self.wait_for_observation(now):
            pass
        elif self.state == 'NAVIGATING':
            goal_state = nav['goal']['state']
            if goal_state == 'SUCCEEDED':
                self.goal_reached=now;self.stable_since=None;self.window_pose=None
                self.scene_anchor=None;self.base_anchor=None;self.leg_anchor=None
                self.stability_reason='WAITING_FOR_STABLE_WINDOW'
                self.transition('BASE_SETTLE', now)
            elif goal_state not in ('REQUESTED','ACTIVE'):
                self.cancel(now, 'NAV_' + goal_state)
            elif now-self.entered > self.cfg['execution']['navigation_timeout_s']:
                self.cancel(now, 'NAV_TIMEOUT', True)
        elif self.state in ARM_PHASES:
            command = r3.get('motion_command', [])
            command_valid=len(command)==3 and all(math.isfinite(v) for v in command)
            zero=(command_valid and r3.get('phase')=='RL_ZERO' and all(abs(v)<=1e-6 for v in command))
            if not command_valid or (self.state!='BASE_SETTLE' and not zero):
                self.cancel(now, 'BASE_COMMAND_NONZERO', True)
            elif not self.fresh('legs',now) or len(self.inputs.get('legs',{}).get('q',[]))!=12 or not all(math.isfinite(v) for v in self.inputs['legs']['q']):
                self.cancel(now,'COLLISION_LEGS_STALE',True)
            elif not self.fresh('base', now):
                self.cancel(now, 'BASE_MEASUREMENT_STALE', True)
            elif not self.fresh('scene',now):
                self.cancel(now,'COHERENT_SCENE_STALE',True)
            elif self.scene_error(now):
                self.cancel(now,self.scene_error(now),True)
            elif self.state == 'BASE_SETTLE':
                base = self.inputs['base'];scene=self.inputs['scene']
                pose = matrix_pose(scene['body']['map_body'])
                if not zero or r3['observed_ns']/1e9<self.goal_reached:
                    # NAV status and R3 status are independent. Lock the base,
                    # wait for a post-goal zero acknowledgement, then start
                    # the fixed five-second delay. Sway does not restart it.
                    self.stable_since=None
                    self.stability_reason='WAITING_FOR_RL_ZERO'
                    if now-self.entered>self.cfg['settle']['timeout_s']:
                        self.cancel(now,'BASE_ZERO_ACK_TIMEOUT',True)
                elif len(pose) != 7 or not all(math.isfinite(v) for v in pose):
                    self.cancel(now, 'BASE_POSE_INVALID', True)
                elif base.get('tilt_rad', math.inf) > self.cfg['settle']['max_tilt_rad']:
                    self.cancel(now,'BASE_TILT_UNSAFE',True)
                elif min(scene['feedback_ns'],scene['legs_ns'],scene['body']['mono_ns'])/1e9<self.goal_reached:
                    self.stability_reason='WAITING_FOR_POST_GOAL_SNAPSHOT'
                elif self.stable_since is not None and now-self.stable_since>=self.cfg['settle']['min_stable_s']:
                    self.scene_anchor=copy.deepcopy(scene)
                    self.scene_anchor.update(captured_ns=int(now*1e9),goal_revision=self.revision,mission_session_id=self.session)
                    self.base_anchor=matrix_pose(scene['body']['map_body'])
                    self.leg_anchor=list(scene['legs'])
                    self.stability_reason='FIXED_WAIT_COMPLETE'
                    self.transition('ARM_SCAN_POSE',now)
                    self.request('scan_pose',now)
                else:
                    self.stability_reason='FIXED_WAIT_AFTER_RL_ZERO'
            else:
                # The five-second stability window ends before arm motion.
                # Ordinary measured base/leg changes do not revoke a segment.
                # Keep session, gross discontinuity, tilt and feedback guards.
                scene=self.inputs['scene']
                pe,re=pose_distance(self.base_anchor,matrix_pose(scene['body']['map_body']))
                if (scene['body']['navigation_session_id']!=self.nav_session
                        or scene['body']['tf_revision']!=self.scene_anchor['body']['tf_revision']):
                    self.cancel(now,'WORLD_ANCHOR_EPOCH_CHANGED',True)
                elif (pe>self.cfg['frames']['continuity_translation_m'] or re>self.cfg['frames']['continuity_rotation_rad']
                      or self.inputs['base']['tilt_rad']>self.cfg['settle']['max_tilt_rad']):
                    self.cancel(now,'SCENE_ANCHOR_DISCONTINUITY',True)
                elif self.pending and self.pending['kind']=='motion':
                    p = self.pending
                    if owner.get('mission_id') == p['id']:
                        if self.state=='WAIT_RETREAT_APPROVAL' and p['operation']=='retreat' and owner.get('mission_state') in ('EXECUTING','SUCCEEDED'):
                            self.transition('RETREAT',now,'ONE_SHOT_OPERATOR_APPROVAL_ACCEPTED')
                        if owner.get('mission_state') == 'SUCCEEDED':
                            if p['operation'] in ('refine','pregrasp','approach'):
                                approved=copy.deepcopy(p['trajectory'])
                                approved['id']=p['id']
                                approved.setdefault('scene_snapshot',p['scene'])
                                self.attempt_segments.append(approved)
                            self.pending = None
                            self.done_motion(p['operation'], now)
                        elif owner.get('mission_state') in ('FAILED','CANCELLED'):
                            self.cancel(now, owner.get('mission_error','OWNER_REJECTED'), True)
                    elif now-p['at'] > self.cfg['execution']['state_timeout_s']:
                        self.cancel(now, 'OWNER_PLAN_NOT_ACCEPTED:'+owner.get('mission_blocker',''), True)
                elif not self.pending and now >= self.detect_at:
                    if self.state == 'ARM_SEARCH':
                        self.request('observe_grasp' if self.ready_observe else 'detect', now)
                    elif self.state == 'PLAN_GRASP':
                        self.request('grasp_plan', now)
            timeout={'ARM_SEARCH':self.cfg['search']['timeout_s'],'VIEW_REFINEMENT':self.cfg['execution']['state_timeout_s']}.get(self.state,self.cfg['execution']['state_timeout_s'])
            if self.state not in TERMINAL and now-self.entered > timeout:
                self.cancel(now, 'PHASE_TIMEOUT', True)
        return self.drain()

    def drain(self):
        effects, self.effects = self.effects, []
        return [(kind,data) for kind,data in effects if self.state not in TERMINAL or kind=='cancel']

    def status(self, now):
        # Periodic health/status is consumed on sensor callback executors.
        # Keep the full approved trajectory in self.pending and its command;
        # LegPlanTrace records it once. Never resend thousands of points here.
        pending=None if self.pending is None else {
            key:self.pending[key] for key in ('id','operation','kind','at') if key in self.pending}
        return dict(operator_trial=bool(self.cfg.get('operator_trial')),retreat_operator_approval_required=self.state=='WAIT_RETREAT_APPROVAL',availability=self.availability,input_rejections=self.input_rejections,wait_goal_stale_since=self.wait_goal_stale_since,stale_input_diagnostic=self.stale_input_diagnostic,observation_recovery=dict(reason=self.recovery_reason,since=self.recovery_since,retry_operation=self.retry_operation,retry_active=self.recovery_retry_active,retries=self.observation_retries),test_mode='ready_observe' if self.ready_observe else 'full',last_observation=self.last_observation,last_compute_metrics=self.last_compute_metrics,grasp_verification='torque_width',grasp_assessment=self.inputs.get('owner',{}).get('grasp_assessment','UNKNOWN'),object_grasped=None,observed_ns=int(now*1e9), session_id=self.session, state=self.state,
                    reason=self.reason, navigation_session_id=self.nav_session,
                    goal_revision=self.revision, goal_stamp_ns=self.goal_stamp,
                    elapsed_s=max(0.,now-self.started) if self.started else 0.,
                    phase_elapsed_s=max(0.,now-self.entered), pending=pending,
                    scan_index=self.scan_index,goal_reached_monotonic=self.goal_reached,
                    stable_since=self.stable_since,stable_duration_s=max(0.,now-self.stable_since) if self.stable_since is not None else 0.,
                    goal_reached_duration_s=max(0.,now-self.goal_reached) if self.goal_reached is not None else 0.,
                    stability_reason=self.stability_reason,stability_restarts=self.stability_restarts,
                    scene_anchor=self.scene_anchor,object_anchor=self.object_anchor,q_search=self.q_search,
                    grasp_attempt=self.grasp_attempt,last_grasp_error=self.last_grasp_error)
