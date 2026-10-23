"""Camera SDK isolation: only the spawned child opens USB; no actuator access.

The ROS process never executes SDK open/read/close, even in a Python thread.
Original exposure timestamps accompany every frame and freshness observation.
"""
import importlib
import os
import multiprocessing as mp
import signal
import threading
import time


def sys_platform_linux():
    import sys
    return sys.platform.startswith("linux")


def _camera_child(pipe, grasp, stop_timeout, mode, factory, parent_pid):
    # Linux camera child must not retain USB if the owning process disappears.
    if sys_platform_linux():
        import ctypes
        if ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) or os.getppid()!=parent_pid:
            pipe.close();return
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    pump = source = None
    try:
        module, name = factory.split(':')
        source = getattr(importlib.import_module(module), name)(grasp)
        source.open()
        from .planning import CameraPump
        pump = CameraPump(source, stop_timeout, mode)
        pipe.send(('ready', pump.K))
        while True:
            if pipe.poll(.02):
                command = pipe.recv()
                if command == 'close':break
                if command == 'frame':
                    try:
                        color, depth = pump.get_frame()
                        pipe.send(('frame', (color, depth, pump.last_frame_timing)))
                    except Exception as error:pipe.send(('frame_error', str(error)))
            with pump.condition:
                timing = (pump.raw_timing if mode == 'latest_raw' else
                          pump.latest[2] if pump.latest else None)
                valid = pump.decode_valid if mode == 'latest_raw' else pump.latest is not None
                health = (dict(timing) if timing else None, valid, pump.error)
            pipe.send(('acquisition',dict(sent_mono_ns=time.monotonic_ns(),
                acquire_started_ns=pump.acquire_started_ns,acquire_completed_ns=pump.acquire_completed_ns,
                acquire_wall_s=pump.acquire_wall_s,max_acquire_wall_s=pump.max_acquire_wall_s,raw_frames=pump.raw_frames)))
            pipe.send(('health', health))
    except (EOFError, BrokenPipeError):pass
    except BaseException as error:
        try:pipe.send(('error', str(error)))
        except (EOFError, BrokenPipeError):pass
    finally:
        try:
            if pump is not None:pump.close()
            elif source is not None:source.close()
        finally:pipe.close()


class CameraProcess:
    def __init__(self, grasp, start_timeout_s, stop_timeout_s, processing_mode,
                 factory='drivers.camera:make_camera'):
        self.condition=threading.Condition();self.send_lock=threading.Lock()
        self.K=None;self.last_frame_timing=None;self.health=None;self.error=''
        self.frame=None;self.frame_error='';self.pending=False;self.closed=False
        self.stop_timeout_s=stop_timeout_s
        ctx=mp.get_context('spawn')
        self.pipe, child=ctx.Pipe()
        self.process=ctx.Process(target=_camera_child,
            args=(child,grasp,stop_timeout_s,processing_mode,factory,os.getpid()),daemon=True,name='rgbd-sdk')
        self.process.start();child.close()
        self.receiver=threading.Thread(target=self._receive,daemon=True,name='rgbd-ipc')
        self.receiver.start()
        try:
            with self.condition:
                if not self.condition.wait_for(lambda:self.K is not None or self.error, start_timeout_s):
                    raise ValueError('CAMERA_OPEN_TIMEOUT')
                if self.error:raise ValueError('CAMERA_PROCESS:'+self.error)
        except BaseException:
            self.close();raise

    def _receive(self):
        try:
            while True:
                kind, value=self.pipe.recv()
                with self.condition:
                    if kind=='ready':self.K=value
                    elif kind=='acquisition':self.acquisition=value
                    elif kind=='health':self.health=value;self.health_received_ns=time.monotonic_ns()
                    elif kind=='error':self.error=value
                    elif kind=='frame':self.frame=value;self.pending=False
                    elif kind=='frame_error':self.frame_error=value;self.pending=False
                    self.condition.notify_all()
        except (EOFError, OSError):
            with self.condition:
                if not self.closed and not self.error:self.error='CAMERA_PROCESS_EXITED'
                self.condition.notify_all()

    def diagnostics(self, ttl):
        with self.condition:
            now=time.time_ns();alive=self.process.is_alive()
            timing,valid,error=self.health if self.health else (None,False,'')
            ages={key:(now-timing[key])/1e9 if timing and key in timing else None
                  for key in ('color_system_ns','depth_system_ns')}
            reason=('CAMERA_CLOSED' if self.closed else
                    'CAMERA_PROCESS_ERROR' if self.error else
                    'CAMERA_PROCESS_EXITED' if not alive else
                    'CAMERA_HEALTH_MISSING' if not self.health else
                    'CAMERA_DECODE_INVALID' if not valid else
                    'CAMERA_ACQUISITION_ERROR' if error else
                    'CAMERA_TIMESTAMP_MISSING' if any(v is None for v in ages.values()) else
                    'CAMERA_TIMESTAMP_FUTURE' if any(v<0 for v in ages.values()) else
                    'CAMERA_FRAME_STALE' if any(v>ttl for v in ages.values()) else 'READY')
            received=getattr(self,'health_received_ns',None)
            return dict(ready=reason=='READY',reason=reason,process_alive=alive,
                acquisition=dict(getattr(self,'acquisition',{})),
                error=self.error or error,decode_valid=bool(valid),ttl_s=ttl,
                observed_wall_ns=now,source_timestamps=dict(timing or {}),
                color_age_s=ages['color_system_ns'],depth_age_s=ages['depth_system_ns'],
                ipc_health_age_s=(time.monotonic_ns()-received)/1e9 if received else None)

    def fresh(self, ttl):
        return self.diagnostics(ttl)['ready']

    def get_frame(self):
        with self.condition:
            if self.closed:raise ValueError('RGBD_CLOSED')
            if self.error:raise ValueError('RGBD_ACQUISITION_FAILED:'+self.error)
            if self.frame is None and not self.pending and not self.frame_error:
                self.pending=True
                with self.send_lock:self.pipe.send('frame')
            if not self.condition.wait_for(lambda:self.frame is not None or self.frame_error or self.error or self.closed,1.):
                raise ValueError('RGBD_TIMEOUT')
            if self.closed:raise ValueError('RGBD_CLOSED')
            if self.error:raise ValueError('RGBD_ACQUISITION_FAILED:'+self.error)
            if self.frame_error:
                error=self.frame_error;self.frame_error='';raise ValueError(error)
            color,depth,self.last_frame_timing=self.frame;self.frame=None
            return color,depth

    def close(self):
        with self.condition:
            if self.closed:return
            self.closed=True;self.condition.notify_all()
        try:
            with self.send_lock:self.pipe.send('close')
        except (OSError, BrokenPipeError):pass
        self.process.join(self.stop_timeout_s)
        # Only this camera child, never a robot owner or ROS controller.
        if self.process.is_alive():self.process.terminate();self.process.join(self.stop_timeout_s)
        if self.process.is_alive():self.process.kill();self.process.join(self.stop_timeout_s)
        self.pipe.close();self.receiver.join(self.stop_timeout_s)
        if self.process.is_alive() or self.receiver.is_alive():raise RuntimeError('CAMERA_PROCESS_STOP_TIMEOUT')
        with self.condition:self.frame=None;self.health=None
