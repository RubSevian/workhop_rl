"""Shared stamped mechanical-base and coherent scene contracts; no ROS/SDK."""
from collections import deque
from dataclasses import dataclass, asdict
import numpy as np
from scipy.spatial.transform import Rotation
from .transforms import rigid, xyzrpy, fixed_joint


def matrix_pose(t):
    t=rigid(t)
    return [*t[:3,3].tolist(), *Rotation.from_matrix(t[:3,:3]).as_quat().tolist()]


def distance(a,b):
    a,b=rigid(a),rigid(b)
    return float(np.linalg.norm(a[:3,3]-b[:3,3])), float(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())


class FrameChain:
    """The same T_map_B provider for coordinator, perception and collision."""
    def __init__(self,cfg):
        self.vehicle_mount=cfg['frames']['vehicle_to_mount']
        self.body_mount=fixed_joint(cfg['paths']['full_urdf'],'go2_to_rars01_mount')
        self.mount_arm=fixed_joint(cfg['paths']['arm_description'],'arm_mount_to_base_link')
        # These calibration transforms are immutable for this provider/session.
        self.vehicle_body=xyzrpy(self.vehicle_mount)@np.linalg.inv(self.body_mount)

    def body(self,map_vehicle):
        # No analytical-candidate or identity fallback in a physical profile.
        return rigid(rigid(map_vehicle)@self.vehicle_body)

    def arm(self,map_vehicle):
        return rigid(self.body(map_vehicle)@self.body_mount@self.mount_arm)


@dataclass(frozen=True)
class BodyPose:
    ros_ns: int
    mono_ns: int
    navigation_session_id: str
    tf_revision: str
    map_vehicle: list
    map_body: list
    frame: str = 'base'
    world_frame: str = 'map'

    def data(self):
        # Provider owns validated list matrices. Copy their rows explicitly;
        # recursive dataclass/deepcopy traversal at every tick adds no safety.
        return dict(ros_ns=self.ros_ns,mono_ns=self.mono_ns,
            navigation_session_id=self.navigation_session_id,tf_revision=self.tf_revision,
            map_vehicle=[row[:] for row in self.map_vehicle],map_body=[row[:] for row in self.map_body],
            frame=self.frame,world_frame=self.world_frame)


class BodyPoseProvider:
    def __init__(self,cfg):
        self.cfg=cfg;self.chain=FrameChain(cfg);self.last=None;self.fault=''

    def observe(self,map_vehicle,ros_ns,mono_ns,now_ns,navigation_session_id,tf_revision):
        if self.fault:raise ValueError(self.fault)
        if (type(ros_ns) is not int or ros_ns<=0 or type(mono_ns) is not int
                or not 0<=(now_ns-mono_ns)/1e9<=self.cfg['limits']['sensor_age_s']
                or not navigation_session_id or not tf_revision):
            raise ValueError('BODY_TF_STALE_OR_UNBOUND')
        body=self.chain.body(map_vehicle)
        if self.last:
            previous=self.last
            if ros_ns==previous.ros_ns:return None # duplicate cannot renew freshness
            if ros_ns<previous.ros_ns or mono_ns<=previous.mono_ns:
                self.fault='ODOMETRY_TIME_RESET'
            elif (navigation_session_id!=previous.navigation_session_id or tf_revision!=previous.tf_revision):
                self.fault='BODY_TF_EPOCH_CHANGED'
            else:
                pe,re=distance(previous.map_body,body)
                if pe>self.cfg['frames']['continuity_translation_m'] or re>self.cfg['frames']['continuity_rotation_rad']:
                    self.fault='ODOMETRY_DISCONTINUITY'
            if self.fault:raise ValueError(self.fault)
        self.last=BodyPose(ros_ns,mono_ns,navigation_session_id,tf_revision,
                           rigid(map_vehicle).tolist(),body.tolist())
        return self.last


@dataclass(frozen=True)
class SceneSnapshot:
    body: BodyPose
    q7: tuple
    legs: tuple
    feedback_ns: int
    legs_ns: int
    owner_session_id: str
    live_feedback_ns: int = None
    live_legs_ns: int = None

    def data(self):
        return dict(body=self.body.data(),q7=tuple(self.q7),legs=tuple(self.legs),
            feedback_ns=self.feedback_ns,legs_ns=self.legs_ns,owner_session_id=self.owner_session_id,
            live_feedback_ns=self.feedback_ns if self.live_feedback_ns is None else self.live_feedback_ns,
            live_legs_ns=self.legs_ns if self.live_legs_ns is None else self.live_legs_ns,
            frame='base',world_frame='map',anchor_ns=self.body.mono_ns)


def validate_scene_data(scene,cfg,now_ns,nav_session,owner_session):
    body=scene['body']
    if (body['frame']!='base' or body['world_frame']!='map'
            or body['navigation_session_id']!=nav_session or not body['tf_revision']
            or scene['owner_session_id']!=owner_session):raise ValueError('SCENE_EPOCH_OR_FRAME_MISMATCH')
    rigid(body['map_body']);rigid(body['map_vehicle'])
    if (np.asarray(scene['q7']).shape!=(7,) or np.asarray(scene['legs']).shape!=(12,)
            or not np.isfinite([*scene['q7'],*scene['legs']]).all()):raise ValueError('INVALID_SCENE_JOINTS')
    stamps=[body['mono_ns'],scene['feedback_ns'],scene['legs_ns']]
    if type(stamps[0]) is not int or not 0<=(now_ns-stamps[0])/1e9<=cfg['limits']['sensor_age_s']:
        raise ValueError('SCENE_SNAPSHOT_STALE:body')
    for name,key in (('feedback_ns','live_feedback_ns'),('legs_ns','live_legs_ns')):
        stamp=scene.get(key,scene[name])
        if type(stamp) is not int or not 0<=(now_ns-stamp)/1e9<=cfg['limits']['arm_age_s']:
            raise ValueError('SCENE_SNAPSHOT_STALE:'+key)
    if any(type(stamp) is not int or stamp<=0 or stamp>now_ns for stamp in stamps):
        raise ValueError('SCENE_SNAPSHOT_STALE:historical')
    if (max(stamps)-min(stamps))/1e9>cfg['frames']['max_state_skew_s']:raise ValueError('SCENE_SNAPSHOT_SKEW')


class SceneBuffer:
    """Histories are copied under the caller's lock; all picks are causal."""
    def __init__(self,cfg):
        self.cfg=cfg;self.bodies=deque(maxlen=400);self.arms=deque(maxlen=400);self.legs=deque(maxlen=1000)
        self.selection={}

    def arm(self,q,stamp,owner):
        q=np.asarray(q,dtype=float)
        if q.shape!=(7,) or not np.isfinite(q).all() or not owner or type(stamp) is not int or stamp<=0:raise ValueError('INVALID_ARM_SNAPSHOT')
        if not self.arms or stamp>self.arms[-1][0]:self.arms.append((stamp,tuple(q),owner))

    def leg(self,q,stamp):
        q=np.asarray(q,dtype=float)
        if q.shape!=(12,) or not np.isfinite(q).all() or type(stamp) is not int or stamp<=0:raise ValueError('INVALID_LEG_SNAPSHOT')
        if not self.legs or stamp>self.legs[-1][0]:self.legs.append((stamp,tuple(q)))

    def diagnostics(self,now_ns):
        return dict(body_samples=len(self.bodies),arm_samples=len(self.arms),leg_samples=len(self.legs),
            body_age_s=(now_ns-self.bodies[-1].mono_ns)/1e9 if self.bodies else None,
            arm_stream_age_s=(now_ns-self.arms[-1][0])/1e9 if self.arms else None,
            leg_stream_age_s=(now_ns-self.legs[-1][0])/1e9 if self.legs else None,
            selection=dict(self.selection))

    def snapshot(self,now_ns,capture_ns=None):
        historical=capture_ns is not None and capture_ns!=now_ns
        live=None
        if historical:
            if not 0<=(now_ns-capture_ns)/1e9<=self.cfg['execution']['camera_max_age_s']:
                raise ValueError('RGBD_STALE_OR_REPEATED')
            # A historical pose is not a substitute for a healthy live stream.
            live=self.snapshot(now_ns)
        capture_ns=now_ns if capture_ns is None else capture_ns
        reference_ns=capture_ns if historical else now_ns
        # Current stream health is independent of the age of causal samples
        # selected to match delayed odometry or a camera exposure.
        for name,stream in (('arm',self.arms),('legs',self.legs)):
            if not stream or not 0<=(now_ns-stream[-1][0])/1e9<=self.cfg['limits']['arm_age_s']:
                raise ValueError('SCENE_SNAPSHOT_STALE:live_'+name)
        candidates=(b for b in reversed(self.bodies) if b.mono_ns<=capture_ns)
        newest=None;last_error='BODY_SNAPSHOT_MISSING';skipped=0
        for body in candidates:
            if newest is None:newest=body
            if (body.navigation_session_id!=newest.navigation_session_id
                    or body.tf_revision!=newest.tf_revision):
                raise ValueError('SCENE_SNAPSHOT_EPOCH_CHANGED')
            if not 0<=(reference_ns-body.mono_ns)/1e9<=self.cfg['limits']['sensor_age_s']:
                # Older candidates cannot regain freshness.
                last_error='SCENE_SNAPSHOT_STALE:body';break
            # Live requests choose the newest complete measured tuple, rather
            # than rejecting every tuple when the newest body lands in a joint
            # receipt gap. Camera requests keep their fixed exposure target.
            target=capture_ns if historical else body.mono_ns
            arm=next((a for a in reversed(self.arms) if a[0]<=target),None)
            leg=next((q for q in reversed(self.legs) if q[0]<=target),None)
            if arm is None or leg is None:
                last_error='COHERENT_JOINT_SNAPSHOT_MISSING'
            else:
                if arm[2]!=self.arms[-1][2]:raise ValueError('SCENE_SNAPSHOT_EPOCH_CHANGED')
                if live is not None and (body.navigation_session_id!=live.body.navigation_session_id
                        or body.tf_revision!=live.body.tf_revision or arm[2]!=live.owner_session_id):
                    raise ValueError('SCENE_SNAPSHOT_EPOCH_CHANGED')
                stamps=[body.mono_ns,arm[0],leg[0]]
                skew=(max(stamps)-min(stamps))/1e9
                self.selection=dict(reference_ns=reference_ns,body_ns=stamps[0],
                    arm_ns=stamps[1],legs_ns=stamps[2],skew_s=skew,
                    skipped_newer_incomplete=skipped,selected_age_s=(now_ns-body.mono_ns)/1e9)
                if any(stamp>reference_ns for stamp in stamps):raise ValueError('SCENE_SNAPSHOT_STALE:future')
                if skew<=self.cfg['frames']['max_state_skew_s']:
                    return SceneSnapshot(body,arm[1],leg[1],arm[0],leg[0],arm[2],self.arms[-1][0],self.legs[-1][0])
                last_error='SCENE_SNAPSHOT_SKEW'
            if historical:break # Do not move camera exposure to another body time.
            skipped+=1
        raise ValueError(last_error)



@dataclass(frozen=True)
class WorldGraspAnchor:
    """A selected grasp in local map; not an object pose or a moving target."""
    map_grasp: list
    capture_ns: int
    capture_ros_ns: int
    navigation_session_id: str
    tf_revision: str
    goal_revision: int
    confidence: float
    frame: str = 'map'

    def validate(self,body,now_ns,max_age_s,goal_revision,body_max_age_s):
        if (self.frame!='map' or self.navigation_session_id!=body.navigation_session_id
                or self.tf_revision!=body.tf_revision or self.goal_revision!=goal_revision):
            raise ValueError('WORLD_ANCHOR_EPOCH_CHANGED')
        if not 0<=(now_ns-self.capture_ns)/1e9<=max_age_s:raise ValueError('WORLD_ANCHOR_STALE')
        if not 0<=(now_ns-body.mono_ns)/1e9<=body_max_age_s:raise ValueError('WORLD_ANCHOR_TF_STALE')
        rigid(self.map_grasp)
        if not np.isfinite(self.confidence) or not 0<=self.confidence<=1:raise ValueError('WORLD_ANCHOR_CONFIDENCE')

    def rebase(self,map_arm,body,now_ns,max_age_s,goal_revision,body_max_age_s):
        self.validate(body,now_ns,max_age_s,goal_revision,body_max_age_s)
        return rigid(np.linalg.inv(rigid(map_arm))@rigid(self.map_grasp))

    def data(self):return asdict(self)
