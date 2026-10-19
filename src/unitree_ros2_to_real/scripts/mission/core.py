"""Pure mission FSM. Effects are serviced outside R3 and outside inference.

All times here are host MONOTONIC. ROS capture time stays in stamped TF data.
No command is sent merely because an old SUCCEEDED is replayed.
"""
import copy
import math
from .scene import matrix_pose, validate_scene_data
from .transforms import pose_distance

TERMINAL = {'MISSION_COMPLETED_UNVERIFIED', 'OBJECT_NOT_FOUND', 'GRASP_UNREACHABLE', 'BLOCKED', 'CANCELLED'}
ARM_PHASES = {'BASE_SETTLE','ARM_SCAN_POSE','ARM_SEARCH','VIEW_REFINEMENT','PLAN_GRASP',
              'PREGRASP','GRASP','RETREAT','RETURN_HOME'}

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
        self.close_stage = False
        self.last_now = 0.
        self.last_compute_metrics = {}

    def observe(self, name, data, now):
        # Duplicate/out-of-order or future producer samples never refresh leases.
        stamp = data.get('observed_ns')
        ttl=self.cfg['limits']['sensor_age_s'] if name in ('base','scene') else self.cfg['execution']['heartbeat_ttl_s']
        if type(stamp) is not int or not 0 <= now-stamp/1e9 <= ttl:
            return False
        if stamp <= self.inputs.get(name, {}).get('observed_ns', 0):
            return False
        self.inputs[name] = copy.deepcopy(data)
        self.received[name] = stamp/1e9
        return True

    def fresh(self, name, now):
        ttl=self.cfg['limits']['sensor_age_s'] if name in ('base','scene') else self.cfg['execution']['heartbeat_ttl_s']
        return name in self.received and 0 <= now-self.received[name] <= ttl

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
            grasp_plan=self.grasp_plan, scan_index=self.scan_index)))

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
            self.transition('RETREAT', now)
            self.request('retreat', now)
        elif operation == 'retreat':
            self.go_home('MISSION_COMPLETED_UNVERIFIED', now)
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
        self.last_compute_metrics=result.get('compute_metrics',{})
        operation = p['operation']
        self.pending = None
        if not result.get('ok'):
            reason = result.get('reason', 'PLAN_REJECTED')
            if operation=='grasp_plan' and reason=='OBJECT_LOST_AFTER_REFINEMENT':
                self.next_search(now)
                return True
            # Unsafe HOME never falls through to a direct zero joint target.
            if operation in ('grasp_plan',) and reason in ('IK_FAIL','NO_CANDIDATES'):
                self.go_home('GRASP_UNREACHABLE', now)
            else:
                self.cancel(now, reason, True)
            return True
        if operation == 'observe_grasp':
            self.last_observation=result.get('observation',{})
            self.detect_at=now+self.cfg['search']['settle_s']
        elif operation == 'detect':
            if result.get('found'):
                self.transition('VIEW_REFINEMENT', now)
                self.request('refine', now)
            else:
                self.next_search(now)
        elif operation == 'grasp_plan':
            self.grasp_plan = result['grasp_plan']
            self.transition('PREGRASP', now)
            self.request('pregrasp', now)
        else:
            # The owner does its own bounds/authority/start validation. It does
            # not take arbitrary joint targets from a CUDA process directly.
            self.pending = dict(p, kind='motion', at=now,
                                scene=copy.deepcopy(result['trajectory'].get('scene_snapshot',self.inputs.get('scene'))))
            self.effects.append(('trajectory', dict(result['trajectory'], id=p['id'],
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
        if not all(self.fresh(n, now) for n in ('r3','nav','owner','perception')):
            self.cancel(now, 'STALE_INPUT', True)
        elif not active:
            self.cancel(now, 'R3_AUTHORITY_REVOKED')
        elif (nav.get('session_id') != self.nav_session or r3.get('controller_session_id') != self.r3_session
              or owner.get('owner_session_id') != self.owner_session
              or r3.get('navigation_session_id') != self.nav_session):
            self.cancel(now, 'SESSION_CHANGED')
        elif not nav.get('navigation_ready') or not owner.get('feedback_ready') or not owner.get('target_valid') or not self.inputs['perception'].get('ready'):
            self.cancel(now, 'TF_ODOM_ARM_OR_PERCEPTION_INVALID', True)
        elif self.state!='WAIT_GOAL' and now-self.started > self.cfg['execution']['mission_timeout_s']:
            self.cancel(now, 'MISSION_TIMEOUT', True)
        elif self.state == 'WAIT_GOAL':
            goal = nav['goal']
            if goal['revision'] > self.last_revision and goal.get('stamp_ns', 0) > self.armed_ros_ns and goal['state'] in ('REQUESTED','ACTIVE'):
                self.started=now
                self.revision, self.goal_stamp = goal['revision'], goal['stamp_ns']
                self.transition('NAVIGATING', now)
        elif nav['goal']['revision'] != self.revision or nav['goal'].get('stamp_ns')!=self.goal_stamp:
            self.cancel(now, 'GOAL_REPLACED')
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
                    # the existing continuous stability window.
                    self.stable_since,self.window_pose=None,None
                    self.stability_reason='WAITING_FOR_RL_ZERO'
                elif len(pose) != 7 or not all(math.isfinite(v) for v in pose):
                    self.cancel(now, 'BASE_POSE_INVALID', True)
                elif base.get('tilt_rad', math.inf) > self.cfg['settle']['max_tilt_rad']:
                    self.stable_since,self.window_pose=None,None
                    self.stability_reason='ABSOLUTE_TILT';self.stability_restarts+=1
                elif min(scene['feedback_ns'],scene['legs_ns'],scene['body']['mono_ns'])/1e9<self.goal_reached:
                    self.stability_reason='WAITING_FOR_POST_GOAL_SNAPSHOT'
                elif self.window_pose is None:
                    self.window_pose,self.stable_since=list(pose),now
                    self.stability_reason='STABLE_WINDOW'
                else:
                    pe, re = pose_distance(self.window_pose, pose)
                    if pe > self.cfg['settle']['max_translation_m']+1e-12 or re > self.cfg['settle']['max_rotation_rad']+1e-12:
                        self.window_pose,self.stable_since=list(pose),now
                        self.stability_reason='TRANSLATION' if pe>self.cfg['settle']['max_translation_m']+1e-12 else 'ROTATION'
                        self.stability_restarts+=1
                    elif (now-self.stable_since>=self.cfg['settle']['min_stable_s']
                          and now-self.goal_reached>=self.cfg['settle']['min_stable_s']):
                        # All anchors come from this single coherent epoch.
                        self.scene_anchor=copy.deepcopy(scene)
                        self.scene_anchor.update(captured_ns=int(now*1e9),goal_revision=self.revision,mission_session_id=self.session)
                        self.base_anchor=matrix_pose(scene['body']['map_body'])
                        self.leg_anchor=list(scene['legs'])
                        self.stability_reason='SCENE_ANCHOR_CAPTURED'
                        self.transition('ARM_SCAN_POSE', now)
                        self.request('scan_pose', now)
                if self.state == 'BASE_SETTLE' and now-self.entered > self.cfg['settle']['timeout_s']:
                    self.cancel(now, 'BASE_SETTLE_TIMEOUT', True)
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
                        if owner.get('mission_state') == 'SUCCEEDED':
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
        return dict(test_mode='ready_observe' if self.ready_observe else 'full',last_observation=self.last_observation,last_compute_metrics=self.last_compute_metrics,grasp_verification='stub',object_grasped=None,observed_ns=int(now*1e9), session_id=self.session, state=self.state,
                    reason=self.reason, navigation_session_id=self.nav_session,
                    goal_revision=self.revision, goal_stamp_ns=self.goal_stamp,
                    elapsed_s=max(0.,now-self.started) if self.started else 0.,
                    phase_elapsed_s=max(0.,now-self.entered), pending=self.pending,
                    scan_index=self.scan_index,goal_reached_monotonic=self.goal_reached,
                    stable_since=self.stable_since,stable_duration_s=max(0.,now-self.stable_since) if self.stable_since is not None else 0.,
                    goal_reached_duration_s=max(0.,now-self.goal_reached) if self.goal_reached is not None else 0.,
                    stability_reason=self.stability_reason,stability_restarts=self.stability_restarts,
                    scene_anchor=self.scene_anchor)
