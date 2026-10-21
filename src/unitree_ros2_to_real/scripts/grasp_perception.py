#!/usr/bin/env python3
"""Perception/planning process, with a bounded work queue and no serial SDK.

CUDA waits never block ROS callbacks or the independent arm-owner cancel port.
"""
import json
import queue
import threading
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
from mission.config import load,blockers
from mission.collision import measured_gripper_width
from mission.health_contract import source_ttl,CONTROLLED_HOME_PHASES
from mission.scene import SceneBuffer
from mission.status_codec import decode_status
from mission.subscription_batch import take_batch
from mission.native_leg_input import NativeLegInput
from mission.scene_wait import camera_context, transient_scene_error
from mission.arm_fast_state import to_mapping
from mission.leg_state_reader import LegStateReader
from mission.ros_scene import ROSBodySource, report_scene
from mission.startup import MARKER,verified

class Perception(Node):
    def __init__(self):
        super().__init__('rars01_grasp_perception')
        self.last_leg_tick=None
        self.leg_reader=LegStateReader()
        self.cfg=load(self.declare_parameter('mission_config_path','').value,self.declare_parameter('workspace','').value)
        self.body_source=ROSBodySource(self,self.cfg);self.scene_buffer=SceneBuffer(self.cfg)
        self.arm_state_transport=self.declare_parameter('arm_state_transport','yaml').value
        if self.arm_state_transport not in ('yaml','typed'):raise ValueError('ARM_STATE_TRANSPORT')
        self.owner_topic='/rars01/commissioning/fast_state' if self.arm_state_transport=='typed' else '/rars01/commissioning/state'
        self.navigation_session='';self.scene_error='';self.last_navigation_stamp=0
        self.controlled_stop=False;self.r3_output_stopped=False;self.r3_state='';self.vision_reloading=False;self.r3_phase='';self.last_r3_stamp=0;self.vision_cancel_complete=False;self.cancel_thread=None
        physical=self.declare_parameter('physical',False).value
        self.reason='READ_ONLY';self.backend=None;self.busy=False;self.started=0.;self.last_compute={}
        self.startup_checks=dict(geometry=False,yolo=False,graspnet=False,camera=False)
        self.lock=threading.Lock();self.last_request=0;self.request_session=''
        self.jobs=queue.Queue(maxsize=1);self.completed=queue.Queue(maxsize=1)
        self.stop_event=threading.Event();self.worker_thread=None
        self.health=self.create_publisher(String,'/rars01/mission/perception_health',1)
        self.results=self.create_publisher(String,'/rars01/mission/computed',1)
        self.joints=self.create_publisher(JointState,'/rars01/joint_states',1)
        owner_type=String
        if self.arm_state_transport=='typed':
            from rars01_state_interfaces.msg import ArmFastState
            owner_type=ArmFastState
        self.owner_subscription=self.create_subscription(owner_type,self.owner_topic,self.owner,self.cfg['frames'].get('arm_history_depth',1))
        self.odometry_subscription=self.create_subscription(Odometry,'/state_estimation',self.odom,
            QoSProfile(depth=self.cfg['frames'].get('scene_receive_depth',32),reliability=ReliabilityPolicy.BEST_EFFORT))
        self.native_leg_input=None
        if self.cfg['frames'].get('leg_state_transport','raw')=='native_batch':
            from go2_scene_interfaces.msg import SceneLegBatch
            self.native_leg_input=NativeLegInput(self.scene_buffer,self.cfg)
            self.native_leg_subscription=self.create_subscription(SceneLegBatch,'/go2/scene/leg_samples',self.leg_batch,32)
        else:
            self.create_subscription(LowState,'/lowstate',self.lowstate,qos_profile_sensor_data,raw=True)
        self.create_subscription(String,'/go2/navigation/status',self.nav_status,1)
        self.create_subscription(String,'/rars01/mission/compute',self.request,1)
        self.create_subscription(String,'/go2/locomotion_status',self.r3_status,1)
        self.create_timer(self.cfg['execution']['tick_s'],self.tick)
        if physical and self.cfg['physical_enabled'] and not blockers(self.cfg):
            self.reason='INITIALIZING'
            self.worker_thread=threading.Thread(target=self.work,daemon=True,name='mission-planning')
            self.worker_thread.start()
        elif physical:self.reason='PHYSICAL_NOT_READY:'+','.join(blockers(self.cfg))

    def r3_status(self,msg):
        if self.count_publishers('/go2/locomotion_status')!=1:return
        try:
            data=decode_status(msg.data);stamp=data['observed_ns'];now=time.monotonic_ns()
            if type(stamp) is not int or stamp<=self.last_r3_stamp or not 0<=(now-stamp)/1e9<=self.cfg['execution']['heartbeat_ttl_s']:return
            self.last_r3_stamp=stamp;self.r3_phase=data.get('phase','');self.r3_state=data.get('state','')
            self.r3_output_stopped=(data.get('output_enabled') is False and data.get('output_stop_confirmed') is True and not data.get('fault_latched',True))
            if data.get('state') not in ('CONTROLLED_STOP','EMERGENCY_FAULT') or self.controlled_stop:return
            self.controlled_stop=True
            self.cancel_thread=threading.Thread(target=self.cancel_task_perception,daemon=True,name='mission-inference-cancel')
            self.cancel_thread.start()
        except (ValueError,KeyError,TypeError):return

    def cancel_task_perception(self):
        backend=self.backend
        if backend is None:return # Startup worker checks the latch before release.
        try:
            if self.busy:backend.collision.work_deadline_ns=time.monotonic_ns()
            backend.stop_task_perception()
            self.vision_cancel_complete=True
        except Exception as error:
            self.get_logger().error('VISION_STOP_FAILED:'+str(error))

    def resume_task_perception(self):
        # Only the planning worker calls this after confirmed output stop.
        # Re-preload models for a later operator session; no actuator command.
        if (not getattr(self,'controlled_stop',False) or self.r3_state!='SYSTEM_HOLD'
                or not self.vision_cancel_complete or self.busy or not self.r3_output_stopped
                or not 0<=(time.monotonic_ns()-self.last_r3_stamp)/1e9<=self.cfg['execution']['heartbeat_ttl_s']
                or self.count_publishers('/go2/locomotion_status')!=1):return
        self.vision_reloading=True
        try:
            self.backend.geometry_only_controlled_stop=False
            for key in ('yolo','graspnet','camera'):self.backend.startup_checks[key]=False
            self.backend._preload_perception(lambda stage:self.get_logger().info('Re-preload:'+stage))
            if self.stop_event.is_set() or self.r3_state!='SYSTEM_HOLD':
                self.cancel_task_perception();return
            if not verified(self.backend.startup_checks):raise RuntimeError('PRELOAD_CHECKS_INCOMPLETE')
            self.request_session='';self.last_request=0
            self.vision_cancel_complete=False;self.controlled_stop=False
        finally:self.vision_reloading=False

    def owner(self,msg):
        if self.count_publishers(getattr(self,'owner_topic','/rars01/commissioning/state'))!=1:return
        try:
            batch=take_batch(getattr(self,'owner_subscription',None),msg,self.cfg['frames'].get('arm_history_depth',1))
            values=[to_mapping(m) if getattr(self,'arm_state_transport','yaml')=='typed' else decode_status(m.data) for m in batch]
            d=values[-1]
            if not d['feedback_ready']:return
            with self.lock:
                for historical in values:
                    if historical.get('feedback_ready') and historical.get('owner_session_id')==d['owner_session_id']:
                        self.scene_buffer.arm(historical['q'],historical['frame_received_ns'],historical['owner_session_id'])
            if self.backend:
                q=d['q'];joint=JointState();joint.name=['joint'+str(i) for i in range(1,7)]+['gripper_left_joint','gripper_right_joint']
                width=measured_gripper_width(self.backend.geometry,q[6],self.cfg['collision'].get('gripper_closed_measurement_tolerance_rad',0.));joint.position=q[:6]+[width/2,width/2]
                age=(time.monotonic_ns()-d['frame_received_ns'])/1e9
                joint.header.stamp=Time(nanoseconds=max(0,self.get_clock().now().nanoseconds-int(age*1e9))).to_msg()
                self.joints.publish(joint)
        except (KeyError,ValueError,TypeError,yaml.YAMLError):return

    def odom(self,msg):
        self.body_source.push_batch(self.odometry_subscription,msg)

    def nav_status(self,msg):
        if self.count_publishers('/go2/navigation/status')!=1:return
        try:
            data=json.loads(msg.data)
            stamp=data['observed_ns']
            if (type(stamp) is int and stamp>getattr(self,'last_navigation_stamp',0)
                    and 0<=(time.monotonic_ns()-stamp)/1e9<=source_ttl(self.cfg,'nav')):
                self.navigation_session=data['session_id']
                self.last_navigation_stamp=stamp
        except (ValueError,KeyError,TypeError):return

    def leg_batch(self,msg):
        if self.count_publishers('/go2/scene/leg_samples')!=1:return
        latest=None
        for batch in take_batch(self.native_leg_subscription,msg,32):
            try:
                with self.lock:self.native_leg_input.apply(batch,time.monotonic_ns())
            except (ValueError,TypeError):continue

    def lowstate(self,msg):
        if self.count_publishers('/lowstate')!=1:return
        try:tick,q=self.leg_reader.read(msg)
        except (ValueError,TypeError,RuntimeError):self.scene_error='INVALID_LEG_SNAPSHOT';return
        if self.last_leg_tick is not None and not 0<(tick-self.last_leg_tick)%2**32<2**31:return
        self.last_leg_tick=tick
        with self.lock:
            try:self.scene_buffer.leg(q,time.monotonic_ns())
            except ValueError:self.scene_error='INVALID_LEG_SNAPSHOT'

    def sample_context(self,capture_ns,ros_ns,*,wait=True):
        def read():
            with self.lock:
                if self.body_source.provider.fault:raise ValueError(self.body_source.provider.fault)
                if self.scene_error and not transient_scene_error(self.scene_error):raise ValueError(self.scene_error)
                return self.scene_buffer.snapshot(time.monotonic_ns(),capture_ns)
        # None explicitly requests a coherent live scene at its body measurement.
        # Camera requests retain their exposure timestamp while delayed odometry
        # arrives. Only the planning worker waits; the ROS executor stays free.
        snapshot=(read() if capture_ns is None or not wait else
                  camera_context(read,capture_ns,self.cfg['execution']['camera_max_age_s'],self.cfg['execution']['tick_s']))
        body=snapshot.body
        if capture_ns is None:capture_ns,ros_ns=body.mono_ns,body.ros_ns
        return dict(capture_ns=capture_ns,capture_ros_ns=ros_ns,feedback_ns=snapshot.feedback_ns,
                    q=list(snapshot.q7[:6]),q7=list(snapshot.q7),legs=list(snapshot.legs),legs_ns=snapshot.legs_ns,
                    owner_session_id=snapshot.owner_session_id,odom_mono_ns=body.mono_ns,
                    odom_vehicle=body.map_vehicle,body=body,scene_snapshot=snapshot.data())

    def request(self,msg):
        if self.backend is None or self.busy or self.count_publishers('/rars01/mission/compute')!=1:return
        try:
            if len(msg.data)>2000000:raise ValueError('oversized')
            d=json.loads(msg.data);stamp=d['observed_ns']
            if getattr(self,'controlled_stop',False) and (d.get('operation')!='home' or self.r3_phase not in CONTROLLED_HOME_PHASES or not self.vision_cancel_complete):return
            if not 0<=(time.monotonic_ns()-stamp)/1e9<=self.cfg['execution']['heartbeat_ttl_s']:return
            if d['id']<=self.last_request:return
            if self.request_session and d['session_id']!=self.request_session:return
            self.request_session=d['session_id'];self.last_request=d['id'];self.busy=True;self.started=time.monotonic()
            self.jobs.put_nowait(d)
        except (ValueError,KeyError,TypeError,queue.Full):return

    def work(self):
        try:
            from mission.compute_scheduling import inference_thread_nice
            self.inference_nice=inference_thread_nice(self.cfg.get('compute',{}).get('inference_nice',0))
            from mission.planning import Planning
            def progress(stage):
                if self.stop_event.is_set():raise RuntimeError('PERCEPTION_SHUTDOWN')
                self.reason='INITIALIZING:'+stage
                self.get_logger().info(self.reason)
            self.backend=Planning(self.cfg,progress=progress,startup_checks=self.startup_checks)
            if self.stop_event.is_set():return
            if not verified(self.backend.startup_checks):raise RuntimeError('PRELOAD_CHECKS_INCOMPLETE')
            self.reason='READY'
            if getattr(self,'controlled_stop',False):self.cancel_task_perception()
            self.get_logger().info('Preload verified; camera standby; waiting for fresh coherent scene')
            print(MARKER,flush=True)
            while not self.stop_event.is_set():
                try:request=self.jobs.get(timeout=.1)
                except queue.Empty:
                    self.resume_task_perception();continue
                if self.stop_event.is_set():break
                out={k:request[k] for k in ('id','session_id','navigation_session_id','owner_session_id','goal_revision')}
                if getattr(self,'controlled_stop',False) and (request['operation']!='home' or self.r3_phase not in CONTROLLED_HOME_PHASES):
                    self.busy=False;continue
                compute_started=time.monotonic();self.backend.compute_metrics={}
                # Bound all collision passes for this request, including a
                # candidate loop/rebuild. This does not change candidate cost.
                try:
                    budget={'detect':self.cfg['search']['timeout_s']}.get(request['operation'],self.cfg['execution']['state_timeout_s'])
                    self.backend.collision.work_deadline_ns=request['observed_ns']+int(budget*1e9)
                    self.backend.collision.check_deadline()
                    result=self.backend.execute(request,self.sample_context)
                    self.backend.validate_result(result,self.sample_context,request)
                    self.backend.collision.check_deadline()
                    out.update(result,ok=True)
                except Exception as e:out.update(ok=False,reason=str(e) or type(e).__name__)
                finally:
                    self.backend.collision.work_deadline_ns=None
                    if request['operation']=='home':self.backend.pause_camera()
                self.last_compute=dict(operation=request['operation'],elapsed_s=time.monotonic()-compute_started,**self.backend.compute_metrics)
                out['compute_metrics']=dict(self.last_compute)
                out['observed_ns']=time.monotonic_ns()
                if getattr(self,'controlled_stop',False) and request['operation']!='home':self.busy=False
                elif not self.stop_event.is_set():self.completed.put_nowait(out)
        except Exception as e:
            self.reason='PERCEPTION_INIT_FAILED:'+str(e)
            self.get_logger().error(self.reason)
        finally:
            if self.backend is not None:
                try:self.backend.close()
                except Exception as error:self.get_logger().error(str(error))

    def destroy_node(self):
        self.stop_event.set()
        if self.backend is not None:
            self.backend.collision.work_deadline_ns=time.monotonic_ns()
            # Only terminate our own inference worker, never an arm/leg owner.
            worker=getattr(self.backend,'worker',None)
            if worker is not None:worker.close(force=True)
        if self.worker_thread is not None:
            self.worker_thread.join(timeout=self.cfg['startup']['shutdown_timeout_s'])
            if self.worker_thread.is_alive():
                self.get_logger().warning('Perception shutdown still computing; process exit will release remaining resources')
            else:self.backend=None
        cancel_thread=getattr(self,'cancel_thread',None)
        if cancel_thread is not None:cancel_thread.join(timeout=self.cfg['startup']['shutdown_timeout_s'])
        for pending in (self.jobs,self.completed):
            while True:
                try:pending.get_nowait()
                except queue.Empty:break
        return super().destroy_node()

    def tick(self):
        try:
            body=self.body_source.select(self.navigation_session)
            if body:
                with self.lock:self.scene_buffer.bodies.append(body)
            with self.lock:self.scene_buffer.snapshot(time.monotonic_ns())
            self.scene_error=''
        except Exception as error:self.scene_error=str(error) or type(error).__name__
        now=time.monotonic()
        ready=(self.backend is not None and not getattr(self,'vision_reloading',False) and self.reason=='READY'
               and not self.scene_error
               and (not getattr(self,'controlled_stop',False) or self.r3_phase in CONTROLLED_HOME_PHASES)
               and self.backend.sensors_ready()
               and (not self.busy or now-self.started<self.cfg['execution']['state_timeout_s']))
        reason=(self.scene_error or self.reason) if self.backend is not None else self.reason
        if self.backend is not None and not self.backend.sensors_ready():
            reason=self.backend.camera_error or 'PERCEPTION_RUNTIME_NOT_READY'
        report_scene(self,reason)
        camera_mode=self.backend.camera_mode if self.backend else 'unverified'
        self.health.publish(String(data=json.dumps(dict(observed_ns=time.monotonic_ns(),ready=bool(ready),reason=reason,scene_error=self.scene_error,busy=self.busy,last_compute=self.last_compute,busy_elapsed_s=now-self.started if self.busy else 0.,
            model_ready=bool(self.backend is not None and self.reason=='READY' and not getattr(self,'controlled_stop',False)),
            controlled_stop_geometry_only=bool(getattr(self,'vision_cancel_complete',False)),
            camera_state={'standby':'CAMERA_STANDBY','active':'CAMERA_ACTIVE','starting':'CAMERA_STARTING','failed':'CAMERA_FAULT'}.get(camera_mode,'CAMERA_UNVERIFIED'),
            scene_ready=bool(self.backend is not None and not self.scene_error),
            startup_checks=dict(self.startup_checks),
            inference_nice=getattr(self,'inference_nice',None),
            preload_ready=self.backend is not None and verified(self.backend.startup_checks),
            camera_mode=camera_mode,
            lowstate_decode=(self.native_leg_input.status() if getattr(self,'native_leg_input',None) else self.leg_reader.status()),tf_pair=dict(self.body_source.diagnostics),streams=self.scene_buffer.diagnostics(time.monotonic_ns())))))
        try:
            out=self.completed.get_nowait();self.busy=False
            if not getattr(self,'controlled_stop',False) or out.get('compute_metrics',{}).get('operation')=='home':
                self.results.publish(String(data=json.dumps(out,allow_nan=False)))
        except queue.Empty:pass

def main():
    from mission.ros_main import spin
    spin(Perception)
if __name__=='__main__':main()
