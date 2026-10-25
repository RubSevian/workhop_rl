"""Bounded, replaceable planning snapshot; never participates in admission."""
import copy
import json
from pathlib import Path
import queue
import threading
import time
from .leg_plan_trace import diagnostic_json


class ArmDebug:
    def __init__(self, directory):
        self.path = Path(directory) / 'latest_arm.json'
        self.queue = queue.Queue(maxsize=1)
        self.error = ''
        self.closed = False
        self.thread = threading.Thread(target=self._write, daemon=True, name='arm-debug')
        self.thread.start()

    def submit(self, snapshot):
        if self.closed:
            return
        try:
            item = copy.deepcopy(dict(snapshot, saved_wall_ns=time.time_ns()))
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass
                self.queue.put_nowait(item)
        except Exception as error:
            self.error = str(error)

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            while True:
                item = self.queue.get()
                if item is None:
                    return
                temp = self.path.with_suffix('.tmp')
                temp.write_text(json.dumps(item, default=diagnostic_json))
                temp.replace(self.path)
        except Exception as error:
            self.error = str(error)

    def close(self):
        self.closed = True
        def finish():
            while self.thread.is_alive():
                try:
                    self.queue.put(None, timeout=.1)
                    return
                except queue.Full:
                    pass
        threading.Thread(target=finish, daemon=True).start()
        self.thread.join(timeout=2.)


def planning_snapshot(request, urdf, current_q, camera_to_arm, audit, calls, status):
    """Only display fields, without full solver attempts/images or collision scenes."""
    fields = ('candidate', 'stage', 'waypoint', 'success', 'joints',
              'position_error_m', 'orientation_error_rad')
    return dict(request_id=request['id'], session_id=request.get('session_id'),
                frame='base_link', urdf=str(urdf), current_q=list(current_q[:6]),
                camera_to_arm=camera_to_arm.tolist(), status=status,
                candidates=audit.get('orientations', []),
                ik=[{key:call[key] for key in fields if key in call} for call in calls],
                selected=audit.get('selected'),
                note='Saved planning state, not live motion. Failed IK is not an executable pose.')
