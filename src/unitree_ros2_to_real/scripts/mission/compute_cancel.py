"""Per-request cancellation; terminal status never revives an old result."""
import threading

class ComputeCancellation:
    def __init__(self):
        self.lock=threading.Lock()
        self.active=None
        self.cancelled=False

    def begin(self, request):
        with self.lock:
            self.active=dict(request)
            self.cancelled=False

    def observe(self, status):
        with self.lock:
            r=self.active
            if (r is None or r.get('operation')=='home'
                    or status.get('session_id')!=r['session_id']
                    or status.get('observed_ns',0)<r['observed_ns']):return False
            if status.get('state') in ('BLOCKED','CANCELLED'):
                self.cancelled=True
            return self.cancelled

    def check(self):
        with self.lock:
            cancelled=self.cancelled
        if cancelled:
            from .collision import CollisionCancelled
            raise CollisionCancelled('COLLISION_CANCELLED:MISSION_CANCELLED')

    def allows(self, result):
        with self.lock:
            return (not self.cancelled and self.active is not None
                    and result.get('id')==self.active['id']
                    and result.get('session_id')==self.active['session_id'])
