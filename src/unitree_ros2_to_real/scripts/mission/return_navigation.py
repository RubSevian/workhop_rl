"""One operator-requested return after measured HOME, within one localization epoch."""
import copy
import math

RETURN_PHASES = frozenset(('RETURN_NAV_PREPARE', 'RETURN_NAVIGATING', 'RETURN_NAV_SETTLE'))
HOME_RESULTS = frozenset(('GRASP_PRESUMED_SUCCESS', 'TARGET_NOT_GRASPED',
                          'OBJECT_NOT_FOUND', 'GRASP_UNREACHABLE', 'MISSION_COMPLETED_UNVERIFIED'))


class ReturnNavigation:
    def __init__(self):
        self.candidate = None
        self.start = None
        self.request_ns = 0
        self.goal_stamp_ns = 0
        self.goal_revision = None
        self.last_send = 0.
        self.loaded = False
        self.reached_at = None
        self.zero_since = None
        self.finished_ns = None
        self.closure = None

    def observe_departure(self, mission, now, goal_stamp_ns=None):
        """Cache a recent zero-command pose BEFORE the outbound request stamp."""
        scene = mission.inputs.get('scene', {})
        r3 = mission.inputs.get('r3', {})
        body = scene.get('body', {})
        if (mission.fresh('scene', now) and not mission.scene_error(now)
                and r3.get('phase') == 'RL_ZERO' and r3.get('motion_command') == [0, 0, 0]
                and (goal_stamp_ns is None or body.get('ros_ns', 0) < goal_stamp_ns)):
            self.candidate = copy.deepcopy(body)
        if goal_stamp_ns is not None:
            body = self.candidate or {}
            if (body.get('ros_ns', 0) < goal_stamp_ns
                    and 0 <= now-body.get('mono_ns', 0)/1e9 <= mission.cfg['limits']['sensor_age_s']):
                self.start = copy.deepcopy(body)
            else:
                self.start = None

    def blocker(self, m, now, *, admission=False):
        if self.start is None:
            return 'RETURN_START_POSE_UNAVAILABLE'
        if admission and (m.state not in HOME_RESULTS or not m.base_motion_window['home_completed_ns']):
            return 'RETURN_REQUIRES_COMPLETED_HOME'
        if not all(m.fresh(name, now) for name in ('r3', 'nav', 'owner', 'scene')):
            return 'RETURN_INPUT_STALE'
        r3, nav, owner, scene = (m.inputs[name] for name in ('r3', 'nav', 'owner', 'scene'))
        if (r3.get('state') != 'ACTIVE' or r3.get('fault_latched')
                or r3.get('operation_profile') != 'full_mission'
                or r3.get('phase') not in ('RL_ZERO', 'RL_ACTIVE')):
            return 'RETURN_R3_AUTHORITY_REVOKED'
        if (not owner.get('arm_home_ready') or not owner.get('feedback_ready')
                or not owner.get('target_valid') or m.pending is not None):
            return 'RETURN_ARM_NOT_HOME'
        if (not nav.get('navigation_ready') or nav.get('cancel_latched')
                or nav.get('far_cancel_pending')):
            return 'RETURN_NAV_UNAVAILABLE_OR_CANCELLED'
        if (nav.get('session_id') != m.nav_session or r3.get('navigation_session_id') != m.nav_session
                or r3.get('controller_session_id') != m.r3_session
                or owner.get('owner_session_id') != m.owner_session
                or scene['body'].get('navigation_session_id') != self.start['navigation_session_id']
                or scene['body'].get('tf_revision') != self.start['tf_revision']):
            return 'RETURN_LOCALIZATION_OR_OWNER_EPOCH_CHANGED'
        error = m.scene_error(now)
        if error:
            return error
        if self.loaded and owner.get('grasp_assessment') != 'GRASPED':
            return 'RETURN_GRASP_NO_LONGER_CONFIRMED'
        if admission and (r3.get('phase') != 'RL_ZERO' or r3.get('motion_command') != [0, 0, 0]):
            return 'RETURN_REQUIRES_RL_ZERO'
        return ''

    def request(self, m, now):
        if self.request_ns:
            return False, 'RETURN_ALREADY_REQUESTED'
        error = self.blocker(m, now, admission=True)
        if error:
            return False, error
        self.loaded = m.state == 'GRASP_PRESUMED_SUCCESS'
        if self.loaded and m.inputs['owner'].get('grasp_assessment') != 'GRASPED':
            return False, 'RETURN_GRASP_NO_LONGER_CONFIRMED'
        self.request_ns = int(now*1e9)
        m.transition('RETURN_NAV_PREPARE', now, 'OPERATOR_RETURN_REQUEST')
        return True, 'RETURN_ACCEPTED_WAITING_FOR_NAV_HANDSHAKE'

    def target(self):
        return [self.start['map_vehicle'][i][3] for i in range(3)] if self.start else None

    def step(self, m, now, ros_now_ns):
        error = self.blocker(m, now)
        if error:
            m.cancel(now, error, True)
            return m.drain()
        nav = m.inputs['nav']; goal = nav['goal']
        if m.state == 'RETURN_NAV_PREPARE':
            if now-self.request_ns/1e9 > 5.:
                m.cancel(now, 'RETURN_GOAL_HANDSHAKE_TIMEOUT', True)
            elif (nav.get('return_request_ns') == self.request_ns and nav.get('base_locked') is False):
                if not self.goal_stamp_ns:
                    self.goal_stamp_ns = ros_now_ns
                if goal.get('stamp_ns') == self.goal_stamp_ns and goal.get('state') in ('REQUESTED', 'ACTIVE', 'SUCCEEDED'):
                    self.goal_revision = goal['revision']
                    m.revision, m.goal_stamp = self.goal_revision, self.goal_stamp_ns
                    m.transition('RETURN_NAVIGATING', now)
                elif now-self.last_send >= .2:
                    self.last_send = now
                    m.effects.append(('return_navigation_goal', dict(target=self.target(), stamp_ns=self.goal_stamp_ns)))
        else:
            if goal.get('stamp_ns') != self.goal_stamp_ns or goal.get('revision') != self.goal_revision:
                m.cancel(now, 'RETURN_GOAL_REPLACED', True)
            elif goal.get('state') == 'SUCCEEDED':
                if self.reached_at is None:
                    self.reached_at = now
                    m.transition('RETURN_NAV_SETTLE', now)
                r3 = m.inputs['r3']
                zero = (r3.get('phase') == 'RL_ZERO' and r3.get('motion_command') == [0, 0, 0]
                        and r3.get('observed_ns', 0) >= int(self.reached_at*1e9))
                inside = (goal.get('error_xy_m') is not None and goal.get('tolerance_m') is not None
                          and math.isfinite(goal['error_xy_m']) and goal['error_xy_m'] <= goal['tolerance_m'])
                if not inside:
                    m.cancel(now, 'RETURN_LEFT_GOAL_TOLERANCE', True)
                    return m.drain()
                self.zero_since = (now if self.zero_since is None else self.zero_since) if zero else None
                if self.zero_since is not None and now-self.zero_since >= m.cfg['settle']['min_stable_s']:
                    current = m.inputs['scene']['body']['map_vehicle']; start = self.start['map_vehicle']
                    delta = [current[i][3]-start[i][3] for i in range(3)]
                    yaw = lambda matrix: math.atan2(matrix[1][0], matrix[0][0])
                    self.closure = dict(delta_xyz_m=delta, distance_xy_m=math.hypot(*delta[:2]),
                        yaw_error_rad=math.remainder(yaw(current)-yaw(start), 2*math.pi),
                        orientation_restored=False, external_ground_truth=False)
                    self.finished_ns = int(now*1e9)
                    m.transition('RETURNED_TO_START', now)
            elif goal.get('state') not in ('REQUESTED', 'ACTIVE'):
                m.cancel(now, 'RETURN_NAV_'+str(goal.get('state')), True)
            timeout = m.cfg['execution']['navigation_timeout_s']
            if timeout is not None and now-self.request_ns/1e9 > timeout:
                m.cancel(now, 'RETURN_NAV_TIMEOUT', True)
        return m.drain()

    def status(self):
        return dict(start=self.start, request_ns=self.request_ns, goal_stamp_ns=self.goal_stamp_ns,
                    goal_revision=self.goal_revision, loaded=self.loaded, finished_ns=self.finished_ns,
                    closure=self.closure)
