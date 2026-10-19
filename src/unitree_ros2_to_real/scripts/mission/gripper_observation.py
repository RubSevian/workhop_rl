"""Informational motor-7 telemetry only. No motion/authority dependencies."""
import json
import math
from pathlib import Path
import queue
import threading
import time

class GripperObservation:
    def __init__(self, config, gripper, feedback_ttl):
        self.cfg=config;self.gripper=gripper;self.ttl=feedback_ttl
        self.last_stamp=0;self.session='';self.since=None;self.latest={}
        for key in ('min_abs_torque_nm','max_abs_velocity_rad_s','stable_s'):
            if not math.isfinite(config[key]) or config[key]<=0:raise ValueError('invalid observation '+key)

    def observe(self, owner, now_ns, phase):
        stamp=owner.get('frame_received_ns',0);session=owner.get('owner_session_id','')
        if session!=self.session:
            self.session=session;self.last_stamp=0;self.since=None
        if type(stamp) is not int or stamp<=self.last_stamp:return None
        previous=self.last_stamp;self.last_stamp=stamp
        row=dict(received_mono_ns=now_ns,wall_ns=time.time_ns(),feedback_mono_ns=stamp,
                 owner_session_id=session,phase=phase,mission_id=owner.get('mission_id'),
                 command_active=owner.get('gripper_command_active') is True,
                 commanded_torque_nm=owner.get('gripper_command_torque_nm'),
                 commanded_q_rad=owner.get('gripper_hold_target'),
                 state='UNKNOWN',reason='INVALID_OR_STALE_FEEDBACK',q_rad=None,dq_rad_s=None,torque_nm=None,
                 informational_only=True,thresholds_calibrated=False,
                 thresholds=dict(min_abs_torque_nm=self.cfg['min_abs_torque_nm'],
                     max_abs_velocity_rad_s=self.cfg['max_abs_velocity_rad_s'],stable_s=self.cfg['stable_s']))
        try:
            q,dq,tau=(float(owner[key][6]) for key in ('q','dq','torque'))
            valid=(owner.get('feedback_ready') is True and owner['valid7'][6] is True
                   and owner['motor_id'][6]==7 and owner['motor_status'][6]==1
                   and all(math.isfinite(v) for v in (q,dq,tau)) and 0<=now_ns-stamp<=self.ttl*1e9)
            row.update(q_rad=q if math.isfinite(q) else None,dq_rad_s=dq if math.isfinite(dq) else None,
                       torque_nm=tau if math.isfinite(tau) else None)
            if valid:
                sign=1 if self.gripper['angle_open']>self.gripper['closed_position_rad'] else -1
                at_stop=sign*(q-self.gripper['closed_position_rad'])<=self.gripper['hard_stop_angle_rad']
                candidate=(row['command_active'] and not at_stop
                           and abs(dq)<=self.cfg['max_abs_velocity_rad_s']
                           and abs(tau)>=self.cfg['min_abs_torque_nm'])
                if previous and stamp-previous>self.ttl*1e9:self.since=None
                if candidate:
                    if self.since is None:self.since=stamp
                    sustained=stamp-self.since>=self.cfg['stable_s']*1e9
                    row.update(state='PRESUMED_HELD' if sustained else 'CONTACT_CANDIDATE',reason='TORQUE_STALL_OUTSIDE_STOP')
                else:
                    self.since=None
                    row.update(state='UNCONFIRMED',reason='CLOSED_STOP' if at_stop else 'NO_HOLD_COMMAND' if not row['command_active'] else 'CONTACT_NOT_SUSTAINED')
            else:self.since=None
        except (KeyError,IndexError,TypeError,ValueError):self.since=None
        self.latest=row
        return row

    def status(self, now_ns):
        row=dict(self.latest) if self.latest else dict(state='UNKNOWN',reason='NO_FEEDBACK')
        if self.last_stamp and not 0<=now_ns-self.last_stamp<=self.ttl*1e9:
            self.since=None;row.update(state='UNKNOWN',reason='FEEDBACK_STALE')
        row['informational_only']=True;row['thresholds_calibrated']=False
        return row

class ObservationLog:
    """Bounded queue; disk errors/dropped rows are diagnostics, never faults."""
    def __init__(self, path, config):
        for key in ('queue_capacity','record_hz','flush_s'):
            if not math.isfinite(config[key]) or config[key]<=0:raise ValueError('invalid recording '+key)
        if type(config['queue_capacity']) is not int:raise ValueError('queue_capacity must be integer')
        self.path=str(path);self.error='';self.dropped=0;self.written=0
        self.queue=queue.Queue(maxsize=int(config['queue_capacity']))
        self.period_ns=int(1e9/config['record_hz']);self.flush_s=config['flush_s'];self.last=0
        self.stop=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True,name='gripper-observation-log')
        self.thread.start()
    def put(self, row):
        if row['feedback_mono_ns']-self.last<self.period_ns:return
        self.last=row['feedback_mono_ns']
        try:self.queue.put_nowait(dict(row))
        except queue.Full:self.dropped+=1
    def run(self):
        try:
            Path(self.path).parent.mkdir(parents=True,exist_ok=True)
            with open(self.path,'x',encoding='utf-8') as output:
                last_flush=time.monotonic()
                while not self.stop.is_set() or not self.queue.empty():
                    try:row=self.queue.get(timeout=min(self.flush_s,.1))
                    except queue.Empty:row=None
                    if row is not None:output.write(json.dumps(row,allow_nan=False)+'\n');self.written+=1
                    if time.monotonic()-last_flush>=self.flush_s:output.flush();last_flush=time.monotonic()
                output.flush()
        except Exception as error:self.error=str(error)
    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
