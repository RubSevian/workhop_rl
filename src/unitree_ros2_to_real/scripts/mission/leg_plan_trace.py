"""Best-effort diagnostics only: no admission decisions and no file IO on ROS callbacks."""
import json
import math
from pathlib import Path
import queue
import threading
import time


def diagnostic_json(value):
    """Numpy payloads are converted on the writer thread, never ROS callbacks."""
    if hasattr(value,'tolist'):return value.tolist()
    raise TypeError('Unsupported diagnostic value: '+type(value).__name__)


class LegPlanTrace:
    def __init__(self, directory):
        self.path=Path(directory)/('legs_'+str(time.time_ns())+'.jsonl')
        self.queue=queue.Queue(maxsize=2048)
        self.lock=threading.Lock();self.baseline=None;self.dropped=0;self.error=''
        self.closed=False
        self.thread=threading.Thread(target=self._write,daemon=True,name='leg-plan-log')
        self.thread.start()

    def _enqueue(self, row):
        row.update(recorded_mono_ns=time.monotonic_ns(),recorded_wall_ns=time.time_ns(),
                   dropped_records=self.dropped)
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1

    def begin(self, request, sample, plan_scene=None, reference_ns=None):
        with self.lock:
            if self.closed:return
            legs=sample['legs'] if plan_scene is None else plan_scene['legs']
            legs_ns=sample['legs_ns'] if plan_scene is None else plan_scene['legs_ns']
            body_ns=sample['body'].mono_ns if plan_scene is None else plan_scene['body']['mono_ns']
            self.baseline=dict(operation_id=request['id'],operation=request['operation'],
                mission_session_id=request['session_id'],owner_session_id=sample['owner_session_id'],
                navigation_session_id=sample['body'].navigation_session_id,
                tf_revision=sample['body'].tf_revision,q_legs_plan=list(legs),
                plan_legs_ns=legs_ns,plan_body_ns=body_ns,
                plan_reference_ns=time.monotonic_ns() if reference_ns is None else reference_ns,
                joint_order=[prefix+'_'+joint for prefix in ('FR','FL','RR','RL') for joint in ('hip','thigh','calf')],units='rad')
            self._enqueue(dict(self.baseline,record_type='plan'))

    def observe(self, q, stamp_ns, **metadata):
        # Input source timestamps are kept; stale/repeated feedback is visible.
        with self.lock:
            if self.closed or self.baseline is None:return
            if len(q)!=12 or not all(math.isfinite(v) for v in q):return
            delta=[float(a)-b for a,b in zip(q,self.baseline['q_legs_plan'])]
            self._enqueue(dict(self.baseline,record_type='measurement',q_legs_live=list(q),
                live_legs_ns=stamp_ns,delta_q=delta,max_abs_delta_q=max(map(abs,delta)),**metadata))

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('x',buffering=1) as output:
                while True:
                    row=self.queue.get()
                    if row is None:break
                    output.write(json.dumps(row,allow_nan=False,default=diagnostic_json)+'\n')
        except Exception as error:self.error=str(error)

    def grasps(self, request, grasps, counts, parameters):
        with self.lock:
            if self.closed:return
            self._enqueue(dict(record_type='grasp_candidates',operation_id=request['id'],
                operation=request['operation'],mission_session_id=request.get('session_id'),
                frame='capture_camera_optical',units='m_rad',
                array_layout='score,width,height,depth,R_row_major[9],translation[3],object_id',
                stage='after_worker_filters_before_nms',grasps=grasps.copy(),
                filter_counts=dict(counts),filter_parameters=dict(parameters)))

    def stage(self, request, stage):
        with self.lock:
            if self.closed:return
            self._enqueue(dict(record_type='planning_stage',operation_id=request['id'],
                operation=request['operation'],mission_session_id=request.get('session_id'),stage=stage))

    def result(self, request, result, metrics):
        with self.lock:
            if self.closed:return
            self._enqueue(dict(record_type='compute_result',operation_id=request['id'],
                operation=request['operation'],mission_session_id=request['session_id'],
                ok=result.get('ok'),reason=result.get('reason'),metrics=dict(metrics),
                trajectory=result.get('trajectory'),grasp_plan=result.get('grasp_plan')))

    def close(self):
        with self.lock:self.closed=True
        # Called during node cleanup, never from a control callback.
        if self.thread.is_alive():
            try:self.queue.put(None,timeout=1.)
            except queue.Full:self.error=self.error or 'TRACE_CLOSE_QUEUE_FULL'
            self.thread.join(timeout=2.)
