#!/usr/bin/env python3
"""Non-RT coordinator. The only motion effect goes to the existing arm owner."""
import json
from pathlib import Path
import time
import uuid
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
from scipy.spatial.transform import Rotation
from mission.scene import SceneBuffer, matrix_pose
from mission.status_codec import decode_status
from mission.input_diagnostics import InputDiagnostics
from mission.subscription_batch import take_batch
from mission.native_leg_input import NativeLegInput
from mission.arm_fast_state import to_mapping
from mission.leg_state_reader import LegStateReader
from mission.ros_scene import ROSBodySource, report_scene
from mission.config import load, blockers
from mission.core import Mission, TERMINAL, ARM_PHASES
from mission.health_contract import source_ttl

class Coordinator(Node):
    def __init__(self):
        super().__init__('go2_grasp_mission')
        self.input_diagnostics=InputDiagnostics();self.stale_fault_reported=False
        self.last_leg_tick=None
        self.leg_reader=LegStateReader()
        self.cfg=load(self.declare_parameter('mission_config_path','').value,self.declare_parameter('workspace','').value)
        self.arm_state_transport=self.declare_parameter('arm_state_transport','yaml').value
        if self.arm_state_transport not in ('yaml','typed'):raise ValueError('ARM_STATE_TRANSPORT')
        self.physical=self.declare_parameter('physical',False).value
        self.blockers=blockers(self.cfg)
        if not self.cfg['physical_enabled']:self.blockers.append('mission_physical_disabled')
        self.engine=Mission(self.cfg,uuid.uuid4().hex)
        self.grip_observer=None;self.grip_log=None;self.grip_error=''
        # Diagnostics are isolated: setup or recording failure never changes readiness.
        try:
            observation=self.cfg.get('gripper_observation',{})
            if observation.get('enabled'):
                from mission.gripper_observation import GripperObservation,ObservationLog
                sdk=yaml.safe_load(Path(self.cfg['paths']['sdk_config']).read_text())
                self.grip_observer=GripperObservation(observation,sdk['robot']['gripper']['rars01'],
                    sdk['robot']['motion']['motion_feedback']['max_feedback_age_s'])
                if self.physical and self.cfg['physical_enabled']:
                    directory=Path(self.get_parameter('workspace').value)/observation['directory']
                    self.grip_log=ObservationLog(directory/(self.engine.session+'.jsonl'),observation)
        except Exception as error:self.grip_error=str(error)
        self.body_source=ROSBodySource(self,self.cfg);self.scene_buffer=SceneBuffer(self.cfg);self.scene_error=''
        self.engine.request_id=time.monotonic_ns()
        self.status_pub=self.create_publisher(String,'/go2/mission/status',1)
        self.health_pub=self.create_publisher(String,'/go2/mission/health',1)
        self.compute_pub=self.create_publisher(String,'/rars01/mission/compute',1)
        self.trajectory_pub=self.create_publisher(String,'/rars01/mission/trajectory',1)
        self.cancel=self.create_client(Trigger,'/rars01/mission/cancel_manipulation')
        self.cancel_nav=self.create_client(Trigger,'/go2/mission/cancel_navigation')
        self.invalidate_nav=self.create_client(Trigger,'/go2/mission/invalidate_navigation_goal')
        self.home=self.create_client(Trigger,'/rars01/control/return_home')
        self.topics={'r3':'/go2/locomotion_status','nav':'/go2/navigation/status',
                     'owner':'/rars01/commissioning/state','perception':'/rars01/mission/perception_health'}
        if self.arm_state_transport=='typed':self.topics['owner']='/rars01/commissioning/fast_state'
        self.gids={};self.gid_since={};self.base_stamp=0;self.stop_home=None;self.stop_home_attempted=False;self.cancel_future=None;self.owner_cancel_ack=False
        for name,topic in self.topics.items():
            kind=String
            if name=='owner' and self.arm_state_transport=='typed':
                from rars01_state_interfaces.msg import ArmFastState
                kind=ArmFastState
            depth=self.cfg['frames'].get('arm_history_depth',1) if name=='owner' else 1
            subscription=self.create_subscription(kind,topic,lambda msg,info,n=name:self.observe(n,msg,info),depth)
            if name=='owner':self.owner_subscription=subscription
        self.create_subscription(String,'/rars01/mission/computed',self.result,1)
        self.native_leg_input=None
        if self.cfg['frames'].get('leg_state_transport','raw')=='native_batch':
            from go2_scene_interfaces.msg import SceneLegBatch
            self.native_leg_input=NativeLegInput(self.scene_buffer,self.cfg)
            self.native_leg_subscription=self.create_subscription(SceneLegBatch,'/go2/scene/leg_samples',self.leg_batch,32)
        else:
            self.create_subscription(LowState,'/lowstate',self.legs,qos_profile_sensor_data,raw=True)
        self.odometry_subscription=self.create_subscription(Odometry,'/state_estimation',self.base,
            QoSProfile(depth=self.cfg['frames'].get('scene_receive_depth',32),reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(self.cfg['execution']['tick_s'],self.tick)
        self.get_logger().info('Mission preloaded DISARMED; physical blockers: '+', '.join(self.blockers))

    def observe(self,name,msg,info):
        started=time.monotonic();value=None;outcome='decode_error';publisher_count=None;parse_error=None
        try:
            typed=name=='owner' and getattr(self,'arm_state_transport','yaml')=='typed'
            if not typed and len(msg.data)>2000000:raise ValueError('oversized input')
            publishers=self.get_publishers_info_by_topic(self.topics[name])
            publisher_count=len(publishers)
            if len(publishers)!=1:
                outcome='publisher_count_invalid'
                self.engine.received.pop(name,None);return
            gid=bytes(publishers[0].endpoint_gid)
            old=self.gids.get(name)
            if old is not None and old!=gid and self.engine.state not in ('IDLE',*TERMINAL):
                self.engine.cancel(time.monotonic(),'PUBLISHER_CHANGED:'+name)
            if old!=gid:
                self.gid_since[name]=time.monotonic_ns()
                self.engine.received.pop(name,None)
            self.gids[name]=gid
            # NAV and perception are JSON producers; C++ arm/R3 use flow YAML.
            owner_values=[]
            if name=='owner':
                batch=take_batch(getattr(self,'owner_subscription',None),msg,self.cfg['frames'].get('arm_history_depth',1))
                owner_values=[to_mapping(m) if typed else decode_status(m.data) for m in batch]
                value=owner_values[-1]
            else:value=json.loads(msg.data) if name in ('nav','perception') else decode_status(msg.data)
            if not isinstance(value,dict):raise ValueError('status must be a mapping')
            # Jazzy MessageInfo exposes timestamps, not the sender GID. Bind to
            # the unique graph endpoint and require a post-discovery producer
            # sample, so queued data from an old owner cannot regain authority.
            if value.get('observed_ns',0)<=self.gid_since[name]:
                outcome='predates_publisher_discovery';return
            outcome='accepted' if self.engine.observe(name,value,time.monotonic()) else 'stamp_rejected'
            if outcome=='stamp_rejected':
                stamp=value.get('observed_ns');age=time.monotonic()-stamp/1e9 if type(stamp) is int else None
                outcome=('invalid_stamp' if age is None else 'future_stamp' if age<0
                         else 'producer_stamp_stale' if age>source_ttl(self.cfg,name)
                         else 'duplicate_or_out_of_order')
            if name=='owner':self.observe_gripper(value)
            if name=='owner' and value.get('feedback_ready'):
                for historical in owner_values:
                    if (historical.get('feedback_ready')
                            and historical.get('observed_ns',0)>self.gid_since[name]
                            and historical.get('owner_session_id')==value['owner_session_id']):
                        self.scene_buffer.arm(historical['q'],historical['frame_received_ns'],historical['owner_session_id'])
        except (ValueError,TypeError,KeyError,yaml.YAMLError) as error:
            parse_error=str(error)
            outcome='invalid_input:'+type(error).__name__
            self.engine.received.pop(name,None)
        finally:
            if outcome!='accepted':
                detail=self.engine.input_rejections.get(name,{})
                detail.update(subsystem=name,source_topic=self.topics[name],publisher_count=publisher_count,
                    publisher_gid=self.gids.get(name,b'').hex(),parse_exception=parse_error,
                    receive_mono_ns=int(started*1e9),action_taken='REJECT_INPUT')
                self.engine.input_rejections[name]=detail
            diagnostics=getattr(self,'input_diagnostics',None)
            if diagnostics is not None:
                diagnostics.callback(name,started,time.monotonic(),outcome,value,publisher_count)

    def observe_gripper(self, owner):
        try:
            if self.grip_observer is not None:
                row=self.grip_observer.observe(owner,time.monotonic_ns(),self.engine.state)
                if row is not None and self.grip_log is not None:
                    row['last_compute_metrics']=dict(self.engine.last_compute_metrics)
                    self.grip_log.put(row)
        except Exception as error:self.grip_error=str(error)

    def destroy_node(self):
        if self.grip_log is not None:self.grip_log.close()
        return super().destroy_node()

    def leg_batch(self,msg):
        if self.count_publishers('/go2/scene/leg_samples')!=1:return
        latest=None
        for batch in take_batch(self.native_leg_subscription,msg,32):
            try:
                sample=self.native_leg_input.apply(batch,time.monotonic_ns())
                if sample is not None:latest=sample
            except (ValueError,TypeError):continue
        if latest is not None:
            stamp,q=latest
            self.engine.observe('legs',dict(observed_ns=stamp,q=list(q)),time.monotonic())

    def legs(self,msg):
        if self.count_publishers('/lowstate')!=1:return
        try:tick,q=self.leg_reader.read(msg)
        except (ValueError,TypeError,RuntimeError):return
        if self.last_leg_tick is not None and not 0<(tick-self.last_leg_tick)%2**32<2**31:return
        self.last_leg_tick=tick
        stamp=time.monotonic_ns();q=list(q)
        try:self.scene_buffer.leg(q,stamp)
        except ValueError:return
        self.engine.observe('legs',dict(observed_ns=stamp,q=q),time.monotonic())

    def base(self,msg):
        self.body_source.push_batch(self.odometry_subscription,msg)

    def refresh_scene(self):
        try:
            body=self.body_source.select(self.engine.inputs.get('nav',{}).get('session_id',''))
            if body:self.scene_buffer.bodies.append(body)
            scene=self.scene_buffer.snapshot(time.monotonic_ns())
            body=scene.body
            rotation=Rotation.from_matrix(np.asarray(body.map_body)[:3,:3])
            tilt=float(np.arccos(np.clip(rotation.as_matrix()[2,2],-1,1)))
            now=time.monotonic()
            self.engine.observe('base',dict(observed_ns=body.mono_ns,pose=matrix_pose(body.map_body),tilt_rad=tilt),now)
            self.engine.observe('scene',dict(scene.data(),observed_ns=body.mono_ns),now)
            self.scene_error=''
        except Exception as error:
            self.scene_error=str(error) or type(error).__name__
            self.engine.received.pop('base',None);self.engine.received.pop('scene',None)
            if self.body_source.provider.fault and self.engine.state not in ('IDLE',*TERMINAL):
                self.engine.cancel(time.monotonic(),str(error),True)
        report_scene(self,self.scene_error or 'READY')

    def result(self,msg):
        if not self.physical or self.blockers:return
        if self.count_publishers('/rars01/mission/computed')!=1:return
        try:
            data=json.loads(msg.data)
            if self.stop_home and all(data.get(k)==self.stop_home.get(k) for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')):
                pending=self.stop_home;self.stop_home=None
                r3=self.engine.inputs.get('r3',{})
                age=(time.monotonic_ns()-data.get('observed_ns',0))/1e9
                if data.get('ok') and r3.get('state')=='CONTROLLED_STOP' and self.engine.fresh('r3',time.monotonic()) and data.get('observed_ns',0)>0 and age>=0:
                    trajectory=dict(data['trajectory'],**{k:pending[k] for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')},observed_ns=time.monotonic_ns())
                    self.trajectory_pub.publish(String(data=json.dumps(trajectory,allow_nan=False)))
                else:self.get_logger().error('Controlled-stop HOME blocked: '+data.get('reason','authority revoked'))
            else:self.engine.result(data,time.monotonic())
        except (ValueError,TypeError,KeyError):self.engine.cancel(time.monotonic(),'INVALID_COMPUTE_RESULT',True)

    def tick(self):
        tick_started=time.monotonic();self.input_diagnostics.tick(tick_started)
        self.refresh_scene()
        now=time.monotonic();e=self.engine
        effects=e.step(now,self.get_clock().now().nanoseconds) if self.physical and not self.blockers else e.drain()
        if e.stale_input_diagnostic is not None and not self.stale_fault_reported:
            detail=e.stale_input_diagnostic
            detail['callbacks']=self.input_diagnostics.snapshot(now)
            detail['scene_refresh_elapsed_s']=now-tick_started
            detail['session_id']=e.session
            self.get_logger().error('STALE_INPUT_DIAGNOSTIC '+json.dumps(detail,allow_nan=False))
            self.stale_fault_reported=True
        ready=(self.physical and not self.blockers and e.wait_goal_stale_since is None and e.fresh('perception',now)
               and e.perception_service_ready(now)
               and e.fresh('owner',now)
               and e.inputs['owner'].get('mission_capable') is True and e.inputs['owner'].get('feedback_ready') is True
               and e.inputs['owner'].get('target_valid') is True
               and self.cancel.service_is_ready() and self.home.service_is_ready()
               and all(self.count_publishers(topic)==1 for topic in self.topics.values()))
        health=dict(observed_ns=time.monotonic_ns(),session_id=e.session,
                    navigation_session_id=e.inputs.get('nav',{}).get('session_id',''),
                    goal_revision=e.revision,phase=e.state,ready=bool(ready),physical=bool(self.physical),base_locked=e.state in ARM_PHASES|TERMINAL,blockers=self.blockers,
                    scene_error=self.scene_error,observation_recovery=e.status(now)['observation_recovery'],lowstate_decode=(self.native_leg_input.status() if getattr(self,'native_leg_input',None) else self.leg_reader.status()),tf_pair=dict(self.body_source.diagnostics),streams=self.scene_buffer.diagnostics(time.monotonic_ns()))
        self.health_pub.publish(String(data=json.dumps(health)))
        # X uses the same collision planner and owner, never a blind zero target.
        # Owner acknowledges the HOME request, while R3 still waits for measured
        # HOME. B/fault cannot use this path and always revokes execution.
        if self.cancel_future is not None and self.cancel_future.done():
            try:self.owner_cancel_ack=self.cancel_future.result().success
            except Exception:self.owner_cancel_ack=False
            self.cancel_future=None
        owner=e.inputs.get('owner',{});r3=e.inputs.get('r3',{})
        if ready and self.owner_cancel_ack and owner.get('home_requested') and r3.get('state')=='CONTROLLED_STOP' and not r3.get('fault_latched') and self.stop_home is None and not self.stop_home_attempted and not e.inputs.get('perception',{}).get('busy',True):
            self.stop_home_attempted=True
            e.request_id+=1
            self.stop_home=dict(id=e.request_id,session_id=e.session,navigation_session_id=e.inputs['nav']['session_id'],owner_session_id=owner['owner_session_id'],goal_revision=e.revision,operation='home',owner=owner,grasp_plan=None,scan_index=0,observed_ns=time.monotonic_ns())
            self.compute_pub.publish(String(data=json.dumps(self.stop_home)))
        for kind,data in effects:
            if kind=='invalidate_navigation_goal':
                if self.invalidate_nav.service_is_ready():self.invalidate_nav.call_async(Trigger.Request())
            elif kind=='cancel_navigation':
                if self.cancel_nav.service_is_ready():self.cancel_nav.call_async(Trigger.Request())
            elif kind=='cancel':
                if self.cancel.service_is_ready():
                    self.owner_cancel_ack=False;self.cancel_future=self.cancel.call_async(Trigger.Request())
                if self.cancel_nav.service_is_ready():self.cancel_nav.call_async(Trigger.Request())
            elif ready:
                data['observed_ns']=time.monotonic_ns()
                if kind=='compute':self.compute_pub.publish(String(data=json.dumps(data,allow_nan=False)))
                elif kind=='trajectory':self.trajectory_pub.publish(String(data=json.dumps(data,allow_nan=False)))
            else:e.cancel(now,'MISSION_HEALTH_INVALID',True)
        status=e.status(now)
        status['gripper_observation']=self.grip_observer.status(time.monotonic_ns()) if self.grip_observer else {'state':'DISABLED'}
        status['gripper_observation']['error']=self.grip_error
        if self.grip_log is not None:
            status['gripper_observation']['recording']=dict(path=self.grip_log.path,error=self.grip_log.error,
                written=self.grip_log.written,dropped=self.grip_log.dropped)
        status['payload_collision_enabled']=self.cfg['collision'].get('payload_enabled',True)
        status['environment_collision_enabled']=self.cfg['collision'].get('environment_enabled',True)
        if self.blockers or not self.physical:
            status.update(state='BLOCKED',reason='PHYSICAL_NOT_READY',blockers=self.blockers+([] if self.physical else ['read_only_launch']))
        self.status_pub.publish(String(data=json.dumps(status)))

def main():
    from mission.ros_main import spin
    spin(Coordinator)
if __name__=='__main__':main()
