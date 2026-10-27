"""Bounded event-only journal. No measurements, image arrays or control decisions."""
import math
import json
from pathlib import Path
import queue
import threading
import time
from .terminal_status import terminal_lines


def finite(value):
    if isinstance(value,dict):return {key:finite(item) for key,item in value.items()}
    if isinstance(value,(tuple,list)):return [finite(item) for item in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def compute_summary(metrics):
    failed=[]
    calls=metrics.get('ik_calls',[])
    for call in calls:
        if call.get('success') is not False:continue
        margins=zip(call.get('lower_margin_rad',[]),call.get('upper_margin_rad',[]))
        near=[dict(joint=index+1,lower_margin_rad=low,upper_margin_rad=high)
              for index,(low,high) in enumerate(margins) if min(low,high)<=.025]
        failed.append({key:call.get(key) for key in ('candidate','stage','waypoint',
            'reason','termination','target_pose6d','position_error_m','orientation_error_rad',
            'position_tolerance_m','orientation_tolerance_rad','attempt_count')})
        failed[-1]['near_joint_limits']=near
        if call.get('attempts'):failed[-1]['last_attempt_reason']=call['attempts'][-1].get('reason')
    return dict(target_shape=metrics.get('target_shape',{}),collision_profile=metrics.get('collision_profile',{}),
        live_validation_elapsed_s=metrics.get('live_validation_elapsed_s'),
        trajectory_build_elapsed_s=metrics.get('trajectory_build_elapsed_s'),
        elapsed_s=metrics.get('elapsed_s'),ik_summary=metrics.get('ik_summary',{}),
        camera_resume_elapsed_s=metrics.get('camera_resume_elapsed_s'),
        yolo_elapsed_s=metrics.get('yolo_elapsed_s'),
        stages=[{key:stage.get(key) for key in ('stage','status','elapsed_s','thread_cpu_s','reason')}
                for stage in metrics.get('planning_stages',[])],
        filter_counts=dict(metrics.get('grasp_filter_counts',{})),
        target_binding=metrics.get('target_binding',{}),
        graspnet_target_roi=metrics.get('graspnet_target_roi',{}),
        failed_ik_count=len(failed),failed_ik=failed[:64],
        failed_ik_omitted=max(0,len(failed)-64),
        grasp_rejections=metrics.get('grasp_rejections',[])[:64],
        collision_rejections=metrics.get('collision_rejections',[])[:32])


class ShortJournal:
    def __init__(self,directory,component,session,max_bytes=10*1024*1024,terminal=None):
        self.path=Path(directory)/(component+'_'+str(session)+'.jsonl')
        self.component=component;self.session=session;self.max_bytes=max_bytes
        self.terminal=terminal;self.terminal_error='';self.phase_state=None;self.phase_since=None
        self.queue=queue.Queue(maxsize=512);self.stop=threading.Event()
        self.previous=None;self.error='';self.dropped=0;self.written=0;self.bytes=0
        self.limit_reached=False
        self.thread=threading.Thread(target=self._write,daemon=True,name='short-journal')
        self.thread.start()

    def put(self,event,**details):
        if self.stop.is_set():return
        if self.limit_reached:self.dropped+=1;return
        row=dict(event=event,component=self.component,session_id=self.session,
                 monotonic_ns=time.monotonic_ns(),wall_ns=time.time_ns(),**details)
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1

    def transition(self,engine,now):
        pending=engine.pending or {};owner=engine.inputs.get('owner',{})
        assessment=owner.get('grasp_assessment','UNKNOWN')
        key=(engine.state,engine.reason,pending.get('id'),pending.get('kind'),
             pending.get('operation'),assessment)
        if key==self.previous:return
        self.previous=key
        previous_duration=None
        if engine.state!=self.phase_state:
            if self.phase_since is not None:previous_duration=now-self.phase_since
            self.phase_state=engine.state;self.phase_since=engine.entered
        perception=engine.inputs.get('perception',{})
        runtime=perception.get('runtime_readiness',{})
        self.put('mission_state',state=engine.state,reason=engine.reason,
            previous_phase_duration_s=previous_duration,
            phase_elapsed_s=now-engine.entered,goal_revision=engine.revision,
            pending={k:pending.get(k) for k in ('id','kind','operation')},
            grasp_assessment=assessment,
            camera={key:runtime.get(key) for key in ('ready','reason','color_age_s','depth_age_s',
                'ipc_health_age_s','ipc_timing','acquisition')})

    def compute(self,request,result,metrics):
        self.put('compute_result',request_id=request['id'],
            mission_session_id=request.get('session_id'),operation=request['operation'],
            navigation_session_id=request.get('navigation_session_id'),
            goal_revision=request.get('goal_revision'),ok=bool(result.get('ok')),
            found=result.get('found',(result.get('observation') or {}).get('found')),
            reason=result.get('reason',''),summary=compute_summary(metrics))

    def stage(self,request,stage):
        self.put('planning_stage',request_id=request['id'],mission_session_id=request.get('session_id'),
            operation=request['operation'],**{key:stage.get(key) for key in
            ('stage','status','elapsed_s','reason','candidate_cost','input_points')})

    def status(self):
        return dict(path=str(self.path),written=self.written,dropped=self.dropped,
                    error=self.error,terminal_error=self.terminal_error,limit_reached=self.limit_reached)

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('x',encoding='utf-8',buffering=65536) as output:
                last_flush=time.monotonic()
                while not self.stop.is_set() or not self.queue.empty():
                    try:row=self.queue.get(timeout=.1)
                    except queue.Empty:row=None
                    if row is not None and not self.limit_reached:
                        encoded=json.dumps(finite(row),ensure_ascii=False,allow_nan=False)+'\n'
                        size=len(encoded.encode('utf-8'))
                        if self.bytes+size>self.max_bytes:
                            self.limit_reached=True;self.dropped+=1
                        else:
                            output.write(encoded);self.bytes+=size;self.written+=1
                            if self.terminal is not None:
                                try:
                                    for line in terminal_lines(row):print(line,file=self.terminal,flush=True)
                                except (OSError,ValueError) as error:
                                    self.terminal_error=str(error);self.terminal=None
                    if time.monotonic()-last_flush>=1.:
                        output.flush();last_flush=time.monotonic()
                output.flush()
        except Exception as error:self.error=str(error)

    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
        if self.thread.is_alive():self.error=self.error or 'SHORT_JOURNAL_CLOSE_TIMEOUT'


def make_short_journal(cfg,workspace,component,session):
    import sys
    if not cfg['execution'].get('short_journal_enabled',False):return None
    journal=ShortJournal(Path(workspace)/'runtime/short_journal',component,session,terminal=sys.stdout)
    journal.put('start',camera_max_age_s=cfg['execution'].get('camera_max_age_s'),mode='events_only')
    return journal
