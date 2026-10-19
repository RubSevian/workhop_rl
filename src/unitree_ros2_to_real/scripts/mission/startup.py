"""Preload completion protocol. It releases launch actions, never actuators."""
MARKER='GO2_MISSION_PRELOAD_READY_V1'
CHECKS=('geometry','yolo','graspnet','camera')

def verified(checks):
    return all(checks.get(name) is True for name in CHECKS)

class StartupRelease:
    """ROS launch stdout arrives as arbitrary byte chunks; release once only."""
    def __init__(self):self.pending=b'';self.released=False
    def feed(self,chunk):
        if self.released:return False
        self.pending+=chunk
        while b'\n' in self.pending:
            line,self.pending=self.pending.split(b'\n',1)
            if line.strip()==MARKER.encode():
                self.released=True;return True
        self.pending=self.pending[-4096:]
        return False
