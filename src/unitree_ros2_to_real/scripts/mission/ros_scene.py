"""Single ROS adapter for the common mechanical-base provider; publishes no TF."""
import hashlib
import json
import time
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from .scene import BodyPoseProvider
from .transforms import pose_matrix, rigid


def transform_matrix(message):
    p,q=message.translation,message.rotation
    return pose_matrix([p.x,p.y,p.z,q.x,q.y,q.z,q.w])


class PairCandidateMismatch(ValueError):
    """Reject this revision, without invalidating another verified fresh pair."""


class BodyTransformBuffer(Buffer):
    def __init__(self,capacity):
        super().__init__()
        self.samples=deque(maxlen=capacity)

    def set_transform(self,transform,authority):
        super().set_transform(transform,authority)
        if transform.child_frame_id=='aft_mapped' and transform.header.frame_id in ('map','camera_init'):
            self.samples.append(transform)


def report_scene(node,reason):
    """Persist useful changes, with bounded logging during flapping inputs."""
    now=time.monotonic()
    if (reason!=getattr(node,'_logged_scene_reason',None)
            and now-getattr(node,'_scene_log_time',float('-inf'))>=node.cfg['execution']['diagnostic_log_interval_s']):
        node._logged_scene_reason=reason;node._scene_log_time=now
        node.get_logger().info('Scene readiness '+json.dumps(dict(reason=reason,
            tf_pair=node.body_source.diagnostics,streams=node.scene_buffer.diagnostics(time.monotonic_ns()))))


class ROSBodySource:
    def __init__(self,node,cfg):
        self.node,self.cfg=node,cfg
        self.buffer=BodyTransformBuffer(cfg['frames']['odometry_buffer_size']);self.listener=TransformListener(self.buffer,node)
        self.provider=BodyPoseProvider(cfg)
        self.pending=deque(maxlen=cfg['frames']['odometry_buffer_size'])
        self.last_input_stamp=0;self.input_fault=''
        self.diagnostics=dict(reason='ODOMETRY_MISSING',detail='',queued=0,
                              latest_odom_stamp_ns=None,latest_tf_stamp_ns=None,
                              selected_stamp_ns=None,selected_age_s=None)

    def push(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp<=0:self.input_fault='ODOMETRY_STAMP_INVALID';return
        if stamp<self.last_input_stamp:self.input_fault='ODOMETRY_TIME_RESET';return
        # Point-LIO can publish corrected poses at the same stamp. Preserve
        # those revisions; arrival does not change measurement freshness.
        self.last_input_stamp=stamp;self.pending.append(msg)

    def select(self,navigation_session_id):
        """Choose the newest available same-time pair, never renew its stamp."""
        if self.input_fault:
            self.provider.fault=self.input_fault
            raise ValueError(self.input_fault)
        if self.provider.fault:raise ValueError(self.provider.fault)
        ros_now=self.node.get_clock().now().nanoseconds
        history=int(self.cfg['frames']['odometry_history_s']*1e9)
        while self.pending and ros_now-(self.pending[0].header.stamp.sec*10**9+self.pending[0].header.stamp.nanosec)>history:
            self.pending.popleft()
        d=self.diagnostics
        d.update(queued=len(self.pending),latest_odom_stamp_ns=self.last_input_stamp or None)
        if self.provider.last:
            d['selected_age_s']=(time.monotonic_ns()-self.provider.last.mono_ns)/1e9
        if not self.pending:d.update(reason='ODOMETRY_MISSING_OR_STALE',detail='');return None
        if not navigation_session_id:d.update(reason='BODY_NAV_SESSION_MISSING',detail='');return None
        try:
            latest=self.buffer.lookup_transform('map','vehicle',Time())
        except TransformException as error:
            d.update(reason='BODY_TF_WAITING',detail=str(error));return None
        latest_stamp=latest.header.stamp.sec*10**9+latest.header.stamp.nanosec
        d['latest_tf_stamp_ns']=latest_stamp
        attempts=0
        received={}
        for transform in self.buffer.samples:
            t=transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
            received[(t,transform.header.frame_id,transform.child_frame_id)]=transform
        d.update(tf_samples=len(received),candidate_reason='',candidate_position_error_m=None,candidate_quaternion_error=None,
                 using_previous_pair=self.provider.last is not None)
        d.update(reason='BODY_TF_WAITING',detail='No fresh odometry with an available TF')
        for msg in reversed(self.pending):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp>latest_stamp:continue
            if self.provider.last and stamp<=self.provider.last.ros_ns:break
            if not 0<=(ros_now-stamp)/1e9<=self.cfg['limits']['sensor_age_s']:continue
            actual=received.get((stamp,msg.header.frame_id,msg.child_frame_id))
            if actual is None:continue # lookup alone could interpolate a missing sample
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            expected=pose_matrix([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            matrix=transform_matrix(actual.transform)
            a=Rotation.from_matrix(matrix[:3,:3]).as_quat();b=Rotation.from_matrix(expected[:3,:3]).as_quat()
            pe=float(np.max(np.abs(matrix[:3,3]-expected[:3,3])))
            qe=float(min(np.max(np.abs(a-b)),np.max(np.abs(a+b))))
            if pe>self.cfg['limits']['tf_position_tolerance_m'] or qe>self.cfg['limits']['tf_quaternion_tolerance']:
                d.update(candidate_reason='TF_ODOMETRY_REVISION_MISMATCH',candidate_position_error_m=pe,candidate_quaternion_error=qe)
                continue
            attempts+=1
            try:
                body=self.read(msg,navigation_session_id)
            except (TransformException,PairCandidateMismatch) as error:
                d.update(reason='BODY_TF_WAITING',detail=str(error))
                d['candidate_reason']=str(error)
                if attempts>=self.cfg['frames']['tf_lookup_attempts_per_tick']:break
                continue
            if body:
                d.update(reason='READY',detail='',selected_stamp_ns=body.ros_ns,
                         using_previous_pair=False,
                         selected_age_s=(time.monotonic_ns()-body.mono_ns)/1e9)
                return body
        return None

    def read(self,msg,navigation_session_id):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp<=0:raise ValueError('ODOMETRY_STAMP_INVALID')
        if msg.header.frame_id not in ('map','camera_init') or msg.child_frame_id!='aft_mapped':
            raise ValueError('ODOMETRY_FRAME_MISMATCH')
        odom=self.node.get_publishers_info_by_topic('/state_estimation')
        if len(odom)!=1:raise ValueError('ODOMETRY_OWNER_NOT_UNIQUE')
        # Epoch follows the odometry owner and the two fixed edges used here.
        # Unrelated /tf writers (including late discovery) do not change frames.
        # Timestamped lookups below still require the actual transform chain.
        graph=[bytes(odom[0].endpoint_gid)]
        at=Time(nanoseconds=stamp)
        vehicle=self.buffer.lookup_transform('map','vehicle',at)
        state=self.buffer.lookup_transform('map','aft_mapped',at)
        for parent,child in (('map','camera_init'),('aft_mapped','vehicle')):
            fixed=transform_matrix(self.buffer.lookup_transform(parent,child,at).transform)
            graph.append(np.round(fixed,12).tobytes())
        revision=hashlib.sha256(b'|'.join(graph)).hexdigest()
        if (vehicle.header.frame_id!='map' or vehicle.child_frame_id!='vehicle'
                or vehicle.header.stamp!=msg.header.stamp or state.header.stamp!=msg.header.stamp):
            raise PairCandidateMismatch('TF_STAMP_OR_FRAME_MISMATCH')
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        expected=pose_matrix([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        if msg.header.frame_id!='map':
            expected=transform_matrix(self.buffer.lookup_transform('map',msg.header.frame_id,at).transform)@expected
        actual=transform_matrix(state.transform)
        a=Rotation.from_matrix(actual[:3,:3]).as_quat();b=Rotation.from_matrix(expected[:3,:3]).as_quat()
        if (np.max(np.abs(actual[:3,3]-expected[:3,3]))>self.cfg['limits']['tf_position_tolerance_m']
                or min(np.max(np.abs(a-b)),np.max(np.abs(a+b)))>self.cfg['limits']['tf_quaternion_tolerance']):
            raise PairCandidateMismatch('TF_ODOMETRY_REVISION_MISMATCH')
        now=time.monotonic_ns();ros_now=self.node.get_clock().now().nanoseconds
        return self.provider.observe(rigid(transform_matrix(vehicle.transform)),stamp,
            now-(ros_now-stamp),now,navigation_session_id,revision)
