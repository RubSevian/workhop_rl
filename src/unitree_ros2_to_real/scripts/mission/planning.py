"""Read-only adapters around existing real GraspNet/IK/trajectory mathematics.

No GraspDriver/RarsArmAdapter instance is constructed: this process never opens
serial. All motor execution belongs to rars_r3_owner.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import copy
import time
import numpy as np
from .transforms import EyeInHand, fixed_joint
from .collision import Collision
from .scene import FrameChain, WorldGraspAnchor, WorldObjectAnchor
from .approved_motion import reverse_segment
from .scene_wait import select_camera_frame
from .config import environment_checks_enabled, search_angles

class CandidatePathRejected(ValueError):
    """A geometrically rejected candidate, not a stale-state/backend failure."""

class CameraPump:
    """Acquisition continues while GraspNet is pending; stamps stay with pixels."""
    def __init__(self, camera, stop_timeout_s=2., processing_mode='continuous'):
        import threading
        if processing_mode not in ('continuous','latest_raw'):raise ValueError('CAMERA_PROCESSING_MODE')
        if processing_mode=='latest_raw' and not all(callable(getattr(camera,k,None)) for k in ('acquire_frames','decode_frames','frame_timing')):
            raise ValueError('CAMERA_RAW_INTERFACE_UNAVAILABLE')
        self.source=camera;self.K=camera.K.copy();self.last_frame_timing=None
        self.condition=threading.Condition();self.latest=None;self.sequence=0;self.consumed=0
        self.processing_mode=processing_mode;self.requested=0;self.completed=0
        self.raw_timing=None;self.decode_valid=False
        self.acquire_started_ns=0;self.acquire_completed_ns=0;self.acquire_wall_s=0.;self.max_acquire_wall_s=0.
        self.raw_frames=0;self.processed_frames=0;self.acquisition_cpu_s=0.;self.processing_cpu_s=0.
        self.running=True
        self.stop_timeout_s=stop_timeout_s
        self.error=''
        self.thread=threading.Thread(target=self.run,daemon=True,name='rgbd-acquisition')
        self.thread.start()
    def run(self):
        try:
            if self.processing_mode=='latest_raw':
                self.run_raw()
                return
            while self.running:
                cpu=time.thread_time()
                color,depth=self.source.get_frame();timing=getattr(self.source,'last_frame_timing',None)
                self.processing_cpu_s+=time.thread_time()-cpu
                if color is not None and depth is not None and timing:
                    with self.condition:
                        self.processed_frames+=1
                        self.latest=(color,depth,dict(timing));self.sequence+=1;self.condition.notify_all()
        except Exception as error:
            with self.condition:self.error=str(error);self.condition.notify_all()
    def run_raw(self):
        failures=0
        while self.running:
            cpu=time.thread_time()
            try:
                self.acquire_started_ns=time.monotonic_ns()
                frames=self.source.acquire_frames()
                self.acquire_completed_ns=time.monotonic_ns()
                self.acquire_wall_s=(self.acquire_completed_ns-self.acquire_started_ns)/1e9
                self.max_acquire_wall_s=max(self.max_acquire_wall_s,self.acquire_wall_s)
                timing=self.source.frame_timing(frames)
            except Exception:
                self.acquisition_cpu_s+=time.thread_time()-cpu
                failures+=1
                if failures>=self.source._FRAME_FAIL_LIMIT:raise
                continue
            self.acquisition_cpu_s+=time.thread_time()-cpu
            if (not timing or timing['color_system_ns']<=0 or timing['depth_system_ns']<=0
                    or abs(timing['color_system_ns']-timing['depth_system_ns'])>1e9/self.source._fps):
                failures+=1
                if failures>=self.source._FRAME_FAIL_LIMIT:raise RuntimeError('RGBD_RAW_ACQUISITION_FAILED')
                continue
            failures=0
            with self.condition:
                self.raw_frames+=1;self.raw_timing=dict(timing)
                request=self.requested
                wanted=request>self.completed
            if not wanted:continue
            # The SDK frameset remains strongly owned by this thread through
            # decode. There is one latest raw sample, no queue of image arrays.
            cpu=time.thread_time()
            color,depth=self.source.decode_frames(frames)
            self.processing_cpu_s+=time.thread_time()-cpu
            timing=getattr(self.source,'last_frame_timing',None)
            with self.condition:
                self.decode_valid=color is not None and depth is not None and bool(timing)
                if self.decode_valid:
                    self.processed_frames+=1
                    self.latest=(color,depth,dict(timing));self.sequence+=1;self.completed=request
                    self.condition.notify_all()
    def stats(self):
        with self.condition:
            return dict(processing_mode=self.processing_mode,raw_frames=self.raw_frames,
                processed_frames=self.processed_frames,acquisition_cpu_s=self.acquisition_cpu_s,
                processing_cpu_s=self.processing_cpu_s)
    def fresh(self,ttl):
        with self.condition:
            if self.processing_mode=='latest_raw':
                return (not self.error and self.decode_valid and self.raw_timing is not None
                    and 0<=(time.time_ns()-self.raw_timing['color_system_ns'])/1e9<=ttl
                    and 0<=(time.time_ns()-self.raw_timing['depth_system_ns'])/1e9<=ttl)
            return not self.error and self.latest is not None and 0<=(time.time_ns()-self.latest[2]['color_system_ns'])/1e9<=ttl
    def get_frame(self):
        with self.condition:
            request=0
            if self.processing_mode=='latest_raw':
                self.requested+=1;request=self.requested
            if not self.condition.wait_for(lambda:(self.completed>=request if request else self.sequence>self.consumed) or self.error or not self.running,timeout=1.):raise ValueError('RGBD_TIMEOUT')
            if self.error:raise ValueError('RGBD_ACQUISITION_FAILED:'+self.error)
            if not self.running:raise ValueError('RGBD_CLOSED')
            self.consumed=self.sequence;color,depth,self.last_frame_timing=self.latest
            return color,depth
    def close(self):
        with self.condition:self.running=False;self.condition.notify_all()
        self.thread.join(timeout=self.stop_timeout_s)
        if self.thread.is_alive():raise RuntimeError('RGBD_ACQUISITION_STOP_TIMEOUT')
        self.source.close()
        with self.condition:self.latest=None;self.last_frame_timing=None;self.raw_timing=None

class Planning:
    def __init__(self, cfg, initialize_perception=True, progress=None, startup_checks=None):
        report=progress or (lambda stage:None)
        report('geometry_imports')
        self.cfg=cfg;self.compute_metrics={};self._closed=False
        self.startup_checks=startup_checks if startup_checks is not None else dict(geometry=False,yolo=False,graspnet=False,camera=False)
        self.camera_mode='unverified';self.camera=None;self.camera_error=''
        self.frames=FrameChain(cfg)
        sys.path.insert(0,cfg['paths']['grasp_root'])
        __import__('os').environ['GRASPNET_ROOT']=cfg['paths']['graspnet_root']
        from rars01_graspnet.config import load_config
        from rars01_graspnet.kinematics import RarsKinematics
        from rars01_graspnet.gripper_geometry import RarsGripperGeometry
        from rars01_graspnet.real_grasp_ik import IkChecker
        self.grasp=load_config(cfg['paths']['grasp_config'])
        self.fk=RarsKinematics(cfg['paths']['arm_urdf'],'base_link','End_link')
        gcfg=self.grasp['robot']['gripper']['rars01']
        geometry_fields=('linkage_radius_m','connecting_rod_length_m','carriage_width_m','jaw_center_End_link_m','jaw_depth_m','jaw_height_m','R_grasp_End_link')
        self.geometry=RarsGripperGeometry(**{k:gcfg[k] for k in geometry_fields},maximum_width_m=self.grasp['robot']['rars01']['max_grasp_width_m'],motor_angle_limit_rad=abs(gcfg['angle_open']))
        self.angles=np.asarray(search_angles(cfg['search']))
        if len(self.angles)!=cfg['search']['max_attempts']:raise ValueError('scan limits/attempts disagree')
        self.joints=np.zeros(6)
        class ConfigOnlyArm:
            groups={'arm':SimpleNamespace(num_joints=6)}
            def load_kinematic_model(inner):
                import pinocchio
                return pinocchio.buildModelFromUrdf(cfg['paths']['arm_urdf'])
            def get_state(inner,request_feedback=False):return self.joints.copy(),np.zeros(6)
        ikcfg=self.grasp['grasp_pipeline']['grasp']
        cart=ikcfg['cartesian_ik']
        self.ik=IkChecker(ConfigOnlyArm(),ikcfg['ik_retry_count'],cart['ik_position_tolerance_m'],np.deg2rad(cart['ik_orientation_tolerance_deg']))
        nav=__import__('yaml').safe_load(Path(cfg['paths']['navigation']).read_text())
        self.feedback_ttl=self.grasp['robot']['motion']['motion_feedback']['max_feedback_age_s']
        self.nav_ttl=nav['bridge']['sensor_timeout_s']
        self.eye=EyeInHand(cfg['paths']['hand_eye'],self.fk,cfg['frames']['vehicle_to_mount'],self.feedback_ttl,fixed_joint(cfg['paths']['arm_description'],'arm_mount_to_base_link'),self.frames,compensation=self.grasp.get('calibration',{}).get('hand_eye_compensation_m',{}))
        self.eye.odom_max_age_ns=int(self.nav_ttl*1e9)
        report('collision_geometry')
        self.collision=Collision(cfg,self.geometry)
        self.startup_checks['geometry']=True
        report(f'collision_geometry_ready cache_hits={self.collision.cache_hits} cache_misses={self.collision.cache_misses}')
        self.last_detection=None;self.scan_joints=None;self.last_frame_mono_ns=0
        if not initialize_perception:return
        try:
            self._preload_perception(report)
        except Exception:
            self.close()
            raise

    def _preload_perception(self,report):
        cfg=self.cfg
        from utils.yolo_utils import load_yolo,resolve_yolo_model_path
        from utils.graspnet_worker import GraspNetWorker
        report('torch_import')
        import cv2,torch
        cv2.setNumThreads(cfg['compute']['cpu_threads'])
        torch.set_num_threads(cfg['compute']['cpu_threads'])
        if not resolve_yolo_model_path(self.grasp['yolo']['model_name'],Path(cfg['paths']['grasp_root'])).is_file():
            raise ValueError('YOLO_MODEL_MISSING_NO_AUTODOWNLOAD')
        report('yolo_load')
        self.yolo,self.yolo_options=load_yolo(self.grasp,project_root=Path(cfg['paths']['grasp_root']),extra_classes=[cfg['object_class'],*cfg.get('object_class_aliases',[])])
        report('yolo_warmup')
        from utils.yolo_utils import detect_objects
        detect_objects(self.yolo,np.zeros((self.grasp['camera']['color_height'],self.grasp['camera']['color_width'],3),dtype=np.uint8),self.yolo_options)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
            # Keep warmed models/kernels, release only unused YOLO activation
            # cache before starting GraspNet and idle NAV on the shared RAM.
            torch.cuda.empty_cache()
        self.startup_checks['yolo']=True
        checkpoint=Path(cfg['paths']['grasp_root'])/self.grasp['graspnet']['checkpoint']
        report('graspnet_worker_start')
        self.worker=GraspNetWorker(checkpoint,300,cfg['startup']['model_timeout_s'])
        report('graspnet_warmup')
        self.worker.warmup(self.grasp['graspnet']['num_point'],cfg['startup']['model_timeout_s'])
        self.startup_checks['graspnet']=True
        report('camera_probe')
        self.resume_camera()
        self.startup_checks['camera']=True
        self.pause_camera()
        report('camera_standby')
        self.last_detection=None;self.scan_joints=None;self.last_frame_mono_ns=0

    def resume_camera(self):
        if self.camera_mode=='active':return
        from .camera_process import CameraProcess
        self.camera_mode='starting';self.camera_start_ns=time.monotonic_ns();self.camera_error=''
        try:
            self.camera=CameraProcess(self.grasp,self.cfg['startup']['camera_probe_timeout_s'],
                self.cfg['startup']['camera_stop_timeout_s'],
                self.grasp['camera'].get('processing_mode','continuous'))
            deadline=self.camera_start_ns/1e9+self.cfg['startup']['camera_probe_timeout_s']
            count=0;last=0
            while count<self.cfg['startup']['camera_probe_frames']:
                if time.monotonic()>deadline:raise ValueError('CAMERA_PROBE_TIMEOUT')
                try:color,depth=self.camera.get_frame()
                except ValueError as error:
                    if str(error)=='RGBD_TIMEOUT':continue
                    raise
                timing=self.camera.last_frame_timing
                stamp=timing['color_system_ns'];depth_stamp=timing['depth_system_ns']
                if (stamp<=last or not 0<=(time.time_ns()-stamp)/1e9<=self.cfg['execution']['camera_max_age_s']
                        or abs(stamp-depth_stamp)>1e9/self.grasp['camera']['fps']):continue
                if (color.ndim!=3 or color.shape[2]!=3 or depth.shape!=color.shape[:2]
                        or not np.isfinite(color).all() or not np.isfinite(depth).all()
                        or not np.isfinite(self.camera.K).all() or self.camera.K.shape!=(3,3)
                        or self.camera.K[0,0]<=0 or self.camera.K[1,1]<=0):
                    raise ValueError('CAMERA_PROBE_INVALID_FRAME')
                last=stamp;count+=1
            self.camera_mode='active'
            self.startup_checks['camera']=True
        except Exception as error:
            self.startup_checks['camera']=False
            self.camera_error=str(error);self.camera_mode='failed'
            if self.camera is not None:self.camera.close()
            raise

    def pause_camera(self):
        if self.camera is not None:self.camera.close()
        self.camera=None;self.camera_mode='standby'
        self.last_detection=None

    def stop_task_perception(self):
        # Called off the ROS executor; this affects only our inference child/camera.
        if getattr(self,'worker',None) is not None:self.worker.close(force=True)
        self.pause_camera();self.yolo=None
        self.geometry_only_controlled_stop=True

    def readiness(self):
        if getattr(self,'geometry_only_controlled_stop',False):
            ready=not getattr(self,'_closed',False) and self.startup_checks.get('geometry') is True and self.camera_mode=='standby'
            return dict(ready=ready,reason='READY' if ready else 'CONTROLLED_STOP_GEOMETRY_NOT_READY')
        reason=('PERCEPTION_CLOSED' if getattr(self,'_closed',False) else
                'PERCEPTION_STARTUP_INCOMPLETE' if not all(self.startup_checks.values()) else
                'GRASPNET_WORKER_EXITED' if not self.worker.is_alive() else '')
        if reason:return dict(ready=False,reason=reason)
        if self.camera_mode=='standby':return dict(ready=True,reason='READY')
        if self.camera_mode=='starting':
            ready=(time.monotonic_ns()-self.camera_start_ns)/1e9<=self.cfg['startup']['camera_probe_timeout_s']
            return dict(ready=ready,reason='READY' if ready else 'CAMERA_START_TIMEOUT')
        if self.camera_mode!='active' or self.camera is None:return dict(ready=False,reason='CAMERA_NOT_ACTIVE')
        ttl=self.cfg['execution']['camera_max_age_s']
        if hasattr(self.camera,'diagnostics'):
            return self.camera.diagnostics(ttl)
        ready=self.camera.fresh(ttl)
        return dict(ready=ready,reason='READY' if ready else 'CAMERA_NOT_READY')

    def sensors_ready(self):
        return self.readiness()['ready']

    def frame(self, context):
        # A delayed/discarded exposure does not invalidate the next real frame.
        # Each attempt must independently satisfy the original camera/scene TTL;
        # the surrounding operation deadline still bounds the whole request.
        attempts=self.cfg['execution'].get('camera_admission_attempts',1)
        for attempt in range(attempts):
            self.collision.check_deadline()
            try:
                count=self.cfg['execution'].get('camera_admission_candidates',1)
                if count==1:return self._frame_once(context)
                frames=[]
                for _ in range(count):
                    self.collision.check_deadline()
                    frame=self._capture_frame()
                    frames.append((frame[2],frame))
                self.compute_metrics['camera_candidate_frames']=self.compute_metrics.get('camera_candidate_frames',0)+len(frames)
                return select_camera_frame(frames,
                    lambda frame:self._admit_frame(frame,lambda cap,ros:context(cap,ros,wait=False)),
                    self.collision.check_deadline,self.cfg['execution']['camera_max_age_s'],self.cfg['execution']['tick_s'])
            except ValueError as error:
                if str(error)!='RGBD_STALE_OR_REPEATED':raise
                metrics=self.compute_metrics
                metrics['camera_admission_discarded']=metrics.get('camera_admission_discarded',0)+1
                if attempt+1==attempts:raise

    def _frame_once(self, context):
        return self._admit_frame(self._capture_frame(),context)

    def _capture_frame(self):
        color,depth=self.camera.get_frame()
        timing=getattr(self.camera,'last_frame_timing',None)
        if color is None or depth is None or timing is None:raise ValueError('RGBD_TIMESTAMPS_MISSING')
        stamp=timing['color_system_ns'];depth_stamp=timing['depth_system_ns']
        # Both timestamps must identify the same paired frame. The tolerance is
        # one configured camera period, not an invented fresh stamp after CUDA.
        if abs(stamp-depth_stamp)>1e9/self.grasp['camera']['fps']:raise ValueError('RGBD_PAIR_SKEW')
        wall,mono=time.time_ns(),time.monotonic_ns()
        capture_mono=mono-(wall-stamp)
        if capture_mono<=self.last_frame_mono_ns or not 0<=(mono-capture_mono)/1e9<=self.cfg['execution']['camera_max_age_s']:
            raise ValueError('RGBD_STALE_OR_REPEATED')
        self.last_frame_mono_ns=capture_mono
        return color,depth,capture_mono,stamp,dict(timing)

    def _admit_frame(self,frame,context):
        color,depth,capture_mono,stamp,timing=frame
        sample=context(capture_mono,stamp)
        arm_camera,odom_arm=self.eye.chain(capture_mono,sample['feedback_ns'],sample['q'],
            sample['odom_mono_ns'],sample['odom_vehicle'])
        self.joints=np.asarray(sample['q'])
        return color,depth,dict(sample,frame_timing=timing,arm_camera=arm_camera,odom_arm=odom_arm,capture_mono_ns=capture_mono,capture_ros_ns=stamp,frame_age_s=(time.monotonic_ns()-capture_mono)/1e9)

    def detect(self, context, count, required=None):
        from utils.yolo_utils import detect_objects
        from itertools import combinations
        strict=required is None
        required=count if strict else required
        selected=[];centers=[];valid_frames=[]
        metrics=getattr(self,'compute_metrics',{});self.compute_metrics=metrics
        observations=[];metrics['detection_observations']=observations
        metrics.update(detection_required=required,detection_budget=count,detection_confirmed=False)
        for index in range(count):
            frame_started=time.monotonic()
            color,depth,sample=self.frame(context)
            frame_wait_s=time.monotonic()-frame_started
            started=time.monotonic()
            _,detections=detect_objects(self.yolo,color,self.yolo_options)
            metrics['yolo_calls']=metrics.get('yolo_calls',0)+1
            metrics['yolo_elapsed_s']=metrics.get('yolo_elapsed_s',0.)+time.monotonic()-started
            record=dict(frame=index+1,detections=[dict(class_name=d.class_name,confidence=float(d.conf),bbox_xyxy=list(d.bbox_xyxy) if hasattr(d,'bbox_xyxy') else None) for d in detections])
            record.update(yolo_s=time.monotonic()-started,frame_wait_s=frame_wait_s)
            observations.append(record)
            def debug_frame(confirmed=False):
                writer=getattr(self,'detection_debug',None)
                if writer is None:return
                try:
                    request=getattr(self,'debug_request',{})
                    writer.submit(color,dict(record,operation_id=request.get('id'),operation=request.get('operation'),
                        scan_index=request.get('scan_index'),required=required,budget=count,confirmed=confirmed,
                        capture_mono_ns=sample.get('capture_mono_ns'),
                        capture_age_at_result_s=(time.monotonic_ns()-sample['capture_mono_ns'])/1e9 if sample.get('capture_mono_ns') else None))
                except Exception as error:metrics['debug_error']=str(error)
            candidates=[d for d in detections if d.class_name in {self.cfg['object_class'],*self.cfg.get('object_class_aliases',[])} and d.conf>=self.cfg['search']['confidence']]
            if not candidates:
                record['reason']='NO_TARGET_ABOVE_CONFIDENCE'
                debug_frame()
                if strict:return None
                continue
            d=max(candidates,key=lambda x:x.conf)
            if d.mask is None:
                record['reason']='MASK_MISSING'
                debug_frame()
                if strict:return None
                continue
            mask=d.mask.astype(bool)
            valid=mask&(depth>0)&(depth>=1000*self.grasp['graspnet']['min_depth'])&(depth<=1000*self.grasp['graspnet']['max_depth'])
            yy,xx=np.nonzero(valid)
            if not len(xx):
                record['reason']='VALID_DEPTH_MISSING'
                debug_frame()
                if strict:return None
                continue
            center=np.array([np.median(xx),np.median(yy)])
            z=float(np.median(depth[valid]))/1000
            point=z*np.linalg.solve(self.camera.K,np.r_[center,1.])
            anchor=sample['odom_arm']@sample['arm_camera']@np.r_[point,1.]
            selected.append((color,depth,d,sample,anchor[:3]));centers.append(center);valid_frames.append(index+1)
            record.update(reason='VALID_TARGET',center_px=center.tolist(),depth_m=z)
            if not strict and len(selected)>=required:
                # Two non-consecutive valid views may confirm the target. A
                # mismatched early view must not poison a later agreeing pair.
                for indices in combinations(range(len(selected)),required):
                    if all(np.linalg.norm(centers[i]-centers[j])<=self.cfg['search']['max_center_shift_px'] for i,j in combinations(indices,2)):
                        metrics.update(detection_confirmed=True,confirmation_frames=[valid_frames[i] for i in indices])
                        debug_frame(True)
                        self.last_detection=selected[indices[-1]]
                        return self.last_detection
            debug_frame(strict and len(selected)==count and max(np.linalg.norm(c-centers[0]) for c in centers)<=self.cfg['search']['max_center_shift_px'])
        if strict and len(selected)==count and max(np.linalg.norm(c-centers[0]) for c in centers)<=self.cfg['search']['max_center_shift_px']:
            metrics['detection_confirmed']=True
            self.last_detection=selected[-1]
            return self.last_detection
        metrics['detection_reason']='INSUFFICIENT_AGREEING_FRAMES'
        return None

    def samples(self, start, targets, duration, gripper_end=None, *, search=False):
        from rars01_graspnet.real_grasp_trajectory import waypoint_motion_samples
        # Measured overshoot is valid geometry, but commanded points must stay
        # inside the unchanged owner's URDF/closed-gripper bounds.
        start=np.asarray(start,dtype=float).copy()
        if getattr(self,'_command_joint_bounds',None) is None:
            import xml.etree.ElementTree as ET
            root=ET.parse(self.cfg['paths']['arm_urdf']).getroot()
            limits=[root.find("joint[@name='joint%d']/limit"%j) for j in range(1,7)]
            self._command_joint_bounds=np.array([[float(v.get('lower')),float(v.get('upper'))] for v in limits])
        bounds=self._command_joint_bounds
        clipped=np.clip(start[:6],bounds[:,0],bounds[:,1])
        if np.any(np.abs(clipped-start[:6])>self.cfg['collision'].get('arm_joint_measurement_tolerance_rad',0.)+1e-9):
            raise ValueError('ARM_MEASUREMENT_OUTSIDE_TOLERANCE')
        start[:6]=clipped
        closed=self.grasp['robot'].get('gripper',{}).get('rars01',{}).get('closed_position_rad',0.)
        if closed-self.cfg['collision'].get('gripper_closed_measurement_tolerance_rad',0.)<=start[6]<closed:
            start[6]=closed
        velocity=np.asarray(self.grasp['robot']['rars01']['position_velocity_limits_rad_s'][:6])
        velocity=np.minimum(velocity,self.cfg['search']['speed_rad_s'] if search else self.cfg['execution']['arm_speed_rad_s'])
        points,_=waypoint_motion_samples(start[:6],targets,duration,1/self.grasp['robot']['rars01']['command_rate_hz'],velocity)
        p=np.vstack([start[:6],*points]);grip=np.full(len(p),start[6])
        if gripper_end is not None:
            phase=np.linspace(0,1,len(p));blend=10*phase**3-15*phase**4+6*phase**5
            grip=start[6]+blend*(gripper_end-start[6])
        return np.column_stack((p,grip))

    def collision_scene(self,sample):
        # NAV vehicle is not presumed to be the mechanical Go2 base.
        map_body=self.frames.body(sample['odom_vehicle'])
        if not np.allclose(map_body,sample['body'].map_body,atol=1e-12,rtol=0):
            raise ValueError('BODY_CONTEXT_FRAME_MISMATCH')
        return dict(legs=list(sample['legs']),legs_ns=sample['legs_ns'],feedback_ns=sample['feedback_ns'],
            scene_snapshot=copy.deepcopy(sample['scene_snapshot']),
            base_ns=sample['odom_mono_ns'],map_body=map_body.tolist(),owner_session_id=sample['owner_session_id'],
            scene_revision=self.cfg['collision']['scene_revision'],
            command_rate_hz=self.grasp['robot']['rars01']['command_rate_hz'],
            velocity_limits=self.grasp['robot']['rars01']['position_velocity_limits_rad_s'],
            navigation_session_id=sample['body'].navigation_session_id,tf_revision=sample['body'].tf_revision)

    def checked(self, points, sample, payload, kind='move', *, reference_ns=None, gripper_reference=None):
        payload=bool(payload and self.cfg['collision'].get('payload_enabled',True))
        scene=self.collision_scene(sample)
        options={} if reference_ns is None else dict(reference_ns=reference_ns)
        command_points=np.asarray(points,dtype=float)
        collision_points=command_points.copy()
        if gripper_reference is not None:
            if (command_points.ndim!=2 or command_points.shape[1]!=7 or len(command_points)<2
                    or not np.isfinite(command_points).all()):raise ValueError('INVALID_COLLISION_PATH')
            collision_points[:,6]=gripper_reference['motor_angle_rad']
        if not self.collision.check(collision_points,scene,payload,**options):
            reason=self.collision.last_reason
            if reason.startswith(('COLLISION:', 'SOLID_CONTAINMENT_COLLISION', 'ENVIRONMENT_')):
                raise CandidatePathRejected(str(self.collision.last_failure))
            raise ValueError(str(self.collision.last_failure))
        trajectory=dict(kind=kind,points=command_points.tolist(),collision_checked=True,
                    payload_checked=bool(payload),collision_context=scene,scene_snapshot=copy.deepcopy(sample['scene_snapshot']))
        if gripper_reference is not None:
            trajectory.update(collision_gripper_reference=copy.deepcopy(gripper_reference),
                collision_points=collision_points.tolist(),physical_geometry_qualified=False)
        return trajectory

    def validate_result(self,result,context,request=None):
        if result.get('grasp_plan'):
            fresh=context(None,None)
            anchor=WorldGraspAnchor(**result['grasp_plan']['world_grasp_anchor'])
            anchor.validate(fresh['body'],time.monotonic_ns(),self.cfg['execution']['mission_timeout_s'],anchor.goal_revision,self.cfg['limits']['sensor_age_s'])
        trajectory=result.get('trajectory')
        if trajectory is None:return
        fresh=context(None,None);scene=self.collision_scene(fresh)
        old=trajectory['collision_context']
        if any(old[key]!=scene[key] for key in ('owner_session_id','scene_revision','navigation_session_id','tf_revision')):
            raise ValueError('WORLD_ANCHOR_EPOCH_CHANGED')
        start_error=np.max(np.abs(np.asarray(fresh['q'])-trajectory['points'][0][:6]))
        if start_error>self.cfg['execution']['start_tolerance_rad']:
            # Match the owner's existing start tolerance. Ordinary sensor
            # variations do not cause an equality/replan loop.
            if request is not None and not trajectory.get('reused_approval'):
                updated=self.execute(request,context)
                self.validate_result(updated,context)
                result.clear();result.update(updated)
                return
            raise ValueError('TRAJECTORY_START_CHANGED_REPLAN_REQUIRED')
        self.collision.validated_scene(scene,trajectory['payload_checked'])
        # The attempt's collision approval uses frozen leg geometry. Ordinary
        # RL_ZERO leg feedback changes are not a reason to run geometry again.
        # A moved external scene still needs checking if that scope is enabled.
        if (trajectory.get('collision_checked') is True and environment_checks_enabled(self.cfg['collision'])
                and not np.array_equal(old['map_body'],scene['map_body'])):
            if trajectory.get('reused_approval'):raise ValueError('APPROVED_WORLD_GEOMETRY_CHANGED')
            if not self.collision.check(trajectory['points'],scene,trajectory['payload_checked']):
                raise ValueError(str(self.collision.last_failure))
        trajectory.setdefault('approval_collision_context',copy.deepcopy(old))
        trajectory.setdefault('approval_scene_snapshot',copy.deepcopy(trajectory.get('scene_snapshot')))
        trajectory['collision_context']=scene
        trajectory['scene_snapshot']=fresh['scene_snapshot']

    def execute(self, request, context):
        from utils.transforms import mat4_to_pose6d,pose6d_to_mat4
        op=request['operation'];owner=request['owner']
        self.debug_request={k:request.get(k) for k in ('id','operation','scan_index','session_id')}
        camera_started_ns=time.monotonic_ns()
        if op in ('detect','observe_grasp','grasp_plan'):
            self.resume_camera()
            self.compute_metrics['camera_resume_elapsed_s']=(time.monotonic_ns()-camera_started_ns)/1e9
        payload=owner.get('gripper_command_active') is True
        sample=context(None,None)
        planning_reference_ns=None
        if op in ('scan_pose','refine'):
            # Check acquisition freshness before IK; later geometry passes use
            # this same scene/time. validate_result still admits live data.
            planning_reference_ns=time.monotonic_ns()
            self.collision.validated_scene(self.collision_scene(sample),payload,reference_ns=planning_reference_ns)
            self.compute_metrics.update(planning_reference_ns=planning_reference_ns,
                scene_body_ns=sample['body'].mono_ns,
                scene_body_age_at_accept_s=(planning_reference_ns-sample['body'].mono_ns)/1e9)
        trace=getattr(self,'plan_trace',None)
        if trace is not None:trace.begin(request,sample,reference_ns=planning_reference_ns)
        if sample['owner_session_id']!=request['owner_session_id'] or sample['body'].navigation_session_id!=request['navigation_session_id']:
            raise ValueError('PLANNING_SESSION_CHANGED')
        start=np.asarray(sample['q7'],dtype=float).copy()
        if owner.get('gripper_command_active') is True:
            start[6]=float(owner['gripper_hold_target'])
        legs=sample['legs'];self.joints=start[:6].copy()
        motion=self.grasp['robot']['motion'];target=None;grip=None
        if op=='detect':
            detection=self.detect(context,self.cfg['search']['observations'],
                required=self.cfg['search'].get('confirmation_frames',self.cfg['search']['observations']))
            if detection is None:return dict(found=False)
            _,_,det,at,point=detection
            anchor=WorldObjectAnchor(np.asarray(point).tolist(),at['capture_mono_ns'],at['capture_ros_ns'],
                at['body'].navigation_session_id,at['body'].tf_revision,request['goal_revision'],request['owner_session_id'],float(det.conf))
            return dict(found=True,object_anchor=anchor.data(),q_search=list(at.get('q',sample['q'])))
        if op=='scan_pose':
            ready=self.grasp['robot']['ready_pose'];pose=[ready[k] for k in ('x','y','z','roll','pitch')]+[ready.get('yaw',0.)]
            ik_started_ns=time.monotonic_ns()
            ik=self.ik.solve(*pose,reference_joints=start[:6])
            self.compute_metrics['scan_pose_ik_elapsed_s']=(time.monotonic_ns()-ik_started_ns)/1e9
            if not ik.success:raise ValueError('SCAN_POSE_IK_FAIL')
            self.scan_joints=ik.joints.copy();self.scan_joints[0]=self.angles[0]
            target=self.scan_joints;grip=self.geometry.motor_angle_for_width(self.grasp['robot']['rars01']['max_grasp_width_m'])
        elif op in ('scan_step','scan_return'):
            if self.scan_joints is None:raise ValueError('NO_SCAN_POSE')
            target=start[:6].copy();target[0]=self.angles[0] if op=='scan_return' else self.angles[request['scan_index']]
        elif op=='refine':
            refine_started_ns=time.monotonic_ns()
            if self.last_detection is None:raise ValueError('NO_STABLE_OBJECT')
            # Recompute the fixed odom object in current arm coordinates.
            _,_,_,old,anchor=self.last_detection
            arm_camera,odom_arm=self.eye.chain(sample['capture_ns'],sample['feedback_ns'],sample['q'],sample['odom_mono_ns'],sample['odom_vehicle'])
            point_camera=np.linalg.inv(odom_arm@arm_camera)@np.r_[anchor,1.]
            delta_camera=np.array([point_camera[0],point_camera[1],0.])*self.cfg['refinement']['gain']
            bound=self.cfg['refinement']['max_translation_m'];delta_camera*=min(1.,bound/max(np.linalg.norm(delta_camera),1e-12))
            pose=self.fk.forward(start[:6]);pose[:3,3]+=arm_camera[:3,:3]@delta_camera
            ik=self.ik.solve(*mat4_to_pose6d(pose),reference_joints=start[:6])
            # Recentring is optional: an unavailable IK must not reject a
            # visible target. Fresh detection still precedes GraspNet.
            target=ik.joints if ik.success else start[:6].copy()
            self.compute_metrics['refine_prepare_elapsed_s']=(time.monotonic_ns()-refine_started_ns)/1e9
        elif op in ('grasp_plan','observe_grasp'):
            detection=self.detect(context,self.cfg['refinement']['fresh_frames'])
            if detection is None:
                if op=='observe_grasp':return dict(observation=dict(found=False,graspnet_called=False))
                raise ValueError('OBJECT_LOST_AFTER_REFINEMENT')
            color,depth,d,at,anchor=detection
            opts=self.grasp['graspnet'];robot=self.grasp['robot']['rars01']
            result=self.worker.infer(color,depth,self.camera.K,num_point=opts['num_point'],min_depth=opts['min_depth'],max_depth=opts['max_depth'],collision_thresh=opts['collision_thresh'],voxel_size=.01,bbox_xyxy=d.bbox_xyxy,target_margin_px=opts['target_margin_px'],target_expand_ratio=opts['target_expand_ratio'],max_grasp_width_m=robot['max_grasp_width_m'],max_grasp_depth_m=robot['max_grasp_depth_m'],timeout_s=opts['worker_timeout_s'],include_diagnostics=self.cfg['compute'].get('grasp_full_diagnostics',True))
            self.compute_metrics['graspnet_worker_elapsed_s']=result.get('elapsed_s')
            if op=='observe_grasp':
                return dict(observation=dict(found=True,confidence=float(d.conf),graspnet_called=True,grasp_count=len(result['grasps']),capture_mono_ns=at['capture_mono_ns'],capture_ros_ns=at['capture_ros_ns'],frame_age_s=at['frame_age_s'],scene_snapshot=at['scene_snapshot']))
            # Inference may take seconds; rebase the RGB-D anchored transform at
            # fresh odom/feedback rather than asking for stale 'latest' TF.
            current=copy.deepcopy(context(None,None))
            planning_reference_ns=time.monotonic_ns()
            self.collision.validated_scene(self.collision_scene(current),False,reference_ns=planning_reference_ns)
            if trace is not None:trace.begin(request,current,reference_ns=planning_reference_ns)
            if current['owner_session_id']!=request['owner_session_id']:
                raise ValueError('PLANNING_SESSION_CHANGED')
            if current['body'].tf_revision!=at['body'].tf_revision or current['body'].navigation_session_id!=at['body'].navigation_session_id:
                raise ValueError('WORLD_ANCHOR_EPOCH_CHANGED')
            start=np.asarray(current['q7'],dtype=float);self.joints=start[:6].copy()
            _,odom_now=self.eye.chain(current['capture_ns'],current['feedback_ns'],current['q'],current['odom_mono_ns'],current['odom_vehicle'])
            cam_to_arm=np.linalg.inv(odom_now)@at['odom_arm']@at['arm_camera']
            from utils import graspnet_utils
            from graspnetAPI import GraspGroup
            from rars01_graspnet.real_grasp_selection import _select_executable_grasp
            geometry=self.geometry
            class GeometryOnly:
                def grasp_tcp_transform(inner,width):return geometry.solve_opening(width).T_grasp_End_link
                def minimum_jaw_height(inner,pose,width):
                    points=geometry.jaw_collision_points(width)
                    return float((pose6d_to_mat4(*pose)@np.column_stack((points,np.ones(len(points)))).T)[2].min())
            g=self.grasp['grasp_pipeline']['grasp']
            rejected_paths=[];approved_candidates={}
            reference_gripper=dict(model='open_gripper_approximation',opening_width_m=robot['max_grasp_width_m'],
                motor_angle_rad=float(self.geometry.motor_angle_for_width(robot['max_grasp_width_m'])))
            def check_candidate_path(points,has_payload,kind='move',open_reference=False):
                # Freeze one coherent measured scene for the whole attempt.
                # validate_result separately checks live admission after planning.
                options=dict(reference_ns=planning_reference_ns)
                if open_reference:options['gripper_reference']=reference_gripper
                return self.checked(points,current,has_payload,kind,**options)
            def validate(candidate):
                try:
                    approved={}
                    phases=[('pregrasp',[candidate.pregrasp_joints],motion['pregrasp_duration_s'],False),
                            ('approach',candidate.approach_joint_waypoints,motion['grasp_duration_s'],False),
                            ('retreat',candidate.retreat_joint_waypoints,motion['retreat_duration_s'],True)]
                    for name,waypoints,duration,has_payload in phases:
                        begin=start.copy() if name=='pregrasp' else np.r_[candidate.pregrasp_joints if name=='approach' else candidate.grasp_joints,start[6]]
                        approved[name]=check_candidate_path(self.samples(begin,waypoints,duration),has_payload,open_reference=name in ('approach','retreat'))
                        approved[name]['operation']=name
                    name='close'
                    closure_start=np.r_[candidate.grasp_joints,start[6]]
                    closed=self.grasp['robot']['gripper']['rars01']['closed_position_rad']
                    approved['close']=check_candidate_path(self.samples(closure_start,[candidate.grasp_joints],motion['grasp_duration_s'],closed),False,'close')
                    approved['close']['points']=[closure_start.tolist(),closure_start.tolist()]
                    name='home'
                    check_candidate_path(self.samples(np.r_[candidate.retreat_joints,start[6]],[np.zeros(6)],motion['retreat_duration_s']),True,'home')
                    approved_candidates[id(candidate)]=approved
                    return True
                except CandidatePathRejected as error:
                    rejected_paths.append(str(error))
                    self.compute_metrics.setdefault('collision_rejections',[]).append(dict(stage=name,detail=str(error)))
                    return False
            rejections=[]
            self.compute_metrics['grasp_rejections']=rejections
            candidate=_select_executable_grasp(self.ik,GeometryOnly(),GraspGroup(result['grasps']),cam_to_arm,g['pregrasp_offset_m'],g.get('retreat_offset_m',g['pregrasp_offset_m']),g['insertion_depth_m'],robot['max_grasp_depth_m'],g['min_base_z_m'],g['min_jaw_z_m'],'rars01',True,True,np.array([g['position_compensation_base_m'][axis] for axis in 'xyz']),g['ik_candidate_limit'],g['cartesian_ik'],g['candidate_selection'],path_validator=validate,rejection_diagnostics=rejections,local_height_filters_enabled=self.cfg.get('local_height_filters_enabled',True))
            if candidate is None:
                if rejected_paths:raise ValueError('NO_COLLISION_FREE_CANDIDATE: '+rejected_paths[-1])
                if any(item['reason'] in ('DEPTH_LIMIT','BASE_HEIGHT_LIMIT','JAW_HEIGHT_LIMIT') for item in rejections):
                    raise ValueError('CANDIDATE_GEOMETRY_REJECTED')
                if any(item['reason']=='NO_CANDIDATES' for item in rejections):raise ValueError('NO_CANDIDATES')
                raise ValueError('IK_FAIL')
            poses_map={name:(odom_now@pose6d_to_mat4(*pose)).tolist() for name,pose in [('pregrasp',candidate.pregrasp6d),('grasp',candidate.grasp6d),('retreat',candidate.retreat6d)]}
            world=WorldGraspAnchor(poses_map['grasp'],at['capture_mono_ns'],at['capture_ros_ns'],
                at['body'].navigation_session_id,at['body'].tf_revision,request['goal_revision'],float(d.conf))
            # Preserve world poses computed in the candidate frame; refresh only
            # the scene used for final freshness/epoch validation after selection.
            final_scene=context(None,None)
            world.validate(final_scene['body'],time.monotonic_ns(),self.cfg['execution']['mission_timeout_s'],request['goal_revision'],self.cfg['limits']['sensor_age_s'])
            object_anchor=WorldObjectAnchor(np.asarray(anchor).tolist(),at['capture_mono_ns'],at['capture_ros_ns'],
                at['body'].navigation_session_id,at['body'].tf_revision,request['goal_revision'],request['owner_session_id'],float(d.conf))
            return dict(object_anchor=object_anchor.data(),grasp_plan=dict(approved_segments=approved_candidates[id(candidate)],planning_scene_snapshot=copy.deepcopy(current['scene_snapshot']),pregrasp=candidate.pregrasp_joints.tolist(),approach=[q.tolist() for q in candidate.approach_joint_waypoints],retreat=[q.tolist() for q in candidate.retreat_joint_waypoints],cost=candidate.cost,object_map=anchor.tolist(),camera_to_arm=cam_to_arm.tolist(),poses_map=poses_map,world_grasp_anchor=world.data(),pose_kind='End_link_TCP'))
        elif op=='retry_open':
            from .approved_motion import release_for_retry
            trajectory=release_for_retry(request['reverse_segment'],sample,self.cfg,
                open_angle=float(request['reverse_segment']['points'][-1][6]),
                rate=self.grasp['robot']['rars01']['command_rate_hz'],velocity=self.grasp['robot']['rars01']['position_velocity_limits_rad_s'][6],assessment=owner.get('grasp_assessment'))
            return dict(trajectory=trajectory)
        elif op=='reverse_search':
            known_empty=(not owner.get('gripper_command_active') or owner.get('object_hold_state')=='EMPTY')
            trajectory=reverse_segment(request['reverse_segment'],sample,self.cfg,known_empty=known_empty)
            return dict(trajectory=trajectory)
        elif op in ('pregrasp','approach','retreat','close') and request.get('grasp_plan',{}).get('approved_segments'):
            plan=request['grasp_plan']
            world=WorldGraspAnchor(**plan['world_grasp_anchor'])
            world.validate(sample['body'],time.monotonic_ns(),self.cfg['execution']['mission_timeout_s'],request['goal_revision'],self.cfg['limits']['sensor_age_s'])
            trajectory=copy.deepcopy(plan['approved_segments'][op])
            old=trajectory['collision_context'];fresh=self.collision_scene(sample)
            if any(old[key]!=fresh[key] for key in ('owner_session_id','scene_revision','navigation_session_id','tf_revision')):
                raise ValueError('WORLD_ANCHOR_EPOCH_CHANGED')
            from .scene import distance
            pe,re=distance(old['map_body'],fresh['map_body'])
            if pe>self.cfg['frames']['continuity_translation_m'] or re>self.cfg['frames']['continuity_rotation_rad']:
                raise ValueError('APPROVED_BODY_DISCONTINUITY')
            # Reuse the approved six-axis path. The gripper reference describes
            # only an open-jaw approximation, never actual-opening clearance.
            if np.max(np.abs(start[:6]-np.asarray(trajectory['points'][0][:6])))>self.cfg['execution']['start_tolerance_rad']:
                raise ValueError('TRAJECTORY_START_CHANGED_REPLAN_REQUIRED')
            if op=='retreat':
                reference=trajectory.get('collision_gripper_reference',{})
                if reference.get('model')!='open_gripper_approximation':
                    raise ValueError('RETREAT_APPROXIMATION_METADATA_MISSING')
                from .collision import measured_gripper_width
                width=measured_gripper_width(self.geometry,float(sample['q7'][6]),self.cfg['collision'].get('gripper_closed_measurement_tolerance_rad',0.))
                if width < -1e-9 or width>self.grasp['robot']['rars01']['max_grasp_width_m']+1e-9:
                    raise ValueError('RETREAT_GRIPPER_OUTSIDE_EXISTING_RANGE')
                # Retain the owner's existing hold command; no automatic release.
                grip=float(start[6])
                closed=self.grasp['robot']['gripper']['rars01']['closed_position_rad']
                if closed-self.cfg['collision'].get('gripper_closed_measurement_tolerance_rad',0.)<=grip<closed:grip=closed
                self.geometry.opening_width(grip) # existing motor-angle limits
                trajectory.setdefault('collision_points',copy.deepcopy(trajectory['points']))
                for point in trajectory['points']:point[6]=grip
                trajectory['physical_geometry_qualified']=False
            elif abs(start[6]-trajectory['points'][0][6])>self.cfg['execution']['start_tolerance_rad']:
                raise ValueError('TRAJECTORY_START_CHANGED_REPLAN_REQUIRED')
            trajectory['operation']=op
            trajectory['reused_approval']=True
            if trace is not None:
                trace.begin(request,sample,plan_scene=trajectory.get('approval_scene_snapshot',trajectory['scene_snapshot']))
            return dict(trajectory=trajectory)
        elif op=='home':target=np.zeros(6)
        elif op=='close':
            # Check the entire jaw sweep; contact changes command mode only in
            # the serial owner, where a bounded closing command finishes without object verification.
            closed=self.grasp['robot']['gripper']['rars01']['closed_position_rad']
            sweep=self.samples(start,[start[:6]],motion['grasp_duration_s'],closed)
            trajectory=self.checked(sweep,sample,payload,'close')
            # Keep initial positions for owner admission; its MIT close law is
            # the existing driver law, not position control through the object.
            trajectory['points']=[start.tolist(),start.tolist()]
            return dict(trajectory=trajectory)
        else:
            raise ValueError('APPROVED_SEGMENTS_REQUIRED')
        preparation_started_ns=time.monotonic_ns()
        points=self.samples(start,[target],self.grasp['robot']['ready_pose']['duration'],grip,search=op in ('scan_step','scan_return'))
        if op in ('scan_pose','scan_step','scan_return'):
            # Operator-approved preset search policy, NOT collision approval.
            # Freshness, epochs, start tolerance and owner limits still apply.
            self.compute_metrics.update(collision_policy='preset_search_unchecked',
                trajectory_build_elapsed_s=(time.monotonic_ns()-preparation_started_ns)/1e9)
            trajectory=dict(kind='move',points=points.tolist(),collision_checked=False,
                collision_policy='preset_search_unchecked',operation=op,
                payload_checked=bool(payload and self.cfg['collision'].get('payload_enabled',True)),
                collision_context=self.collision_scene(sample),scene_snapshot=copy.deepcopy(sample['scene_snapshot']))
            if op in ('scan_step','scan_return'):trajectory['preset_operation']=op
            return dict(trajectory=trajectory)
        try:
            trajectory=self.checked(points,sample,payload,'home' if op=='home' else 'move',
                **({} if planning_reference_ns is None else dict(reference_ns=planning_reference_ns)))
        except CandidatePathRejected:
            if op!='refine':raise
            # Do not move along a colliding recenter path. Keep the current
            # collision-checked pose and obtain fresh images for grasp planning.
            trajectory=self.checked(np.vstack([start,start]),sample,payload,reference_ns=planning_reference_ns)
        return dict(trajectory=trajectory)

    def close(self):
        if getattr(self,'_closed',False):return
        self._closed=True
        if getattr(self,'detection_debug',None) is not None:self.detection_debug.close()
        errors=[]
        try:
            if getattr(self,'camera',None) is not None:self.camera.close()
        except Exception as error:errors.append(str(error))
        finally:
            self.camera=None;self.camera_mode='closed'
        try:
            if getattr(self,'worker',None) is not None:self.worker.close(force=True)
        except Exception as error:errors.append(str(error))
        finally:
            self.worker=None;self.yolo=None;self.last_detection=None
        import gc
        gc.collect()
        torch=sys.modules.get('torch')
        if torch is not None and torch.cuda.is_initialized():torch.cuda.empty_cache()
        if errors:raise RuntimeError('PERCEPTION_CLOSE_FAILED:'+ ';'.join(errors))
