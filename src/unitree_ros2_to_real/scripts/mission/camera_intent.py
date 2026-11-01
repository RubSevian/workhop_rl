"""Camera lifetime follows mission phase; this grants no motion authority."""
from .core import ARM_PHASES


def camera_requested(status):
    # HOME closes the camera; a second full attempt reopens at BASE_SETTLE/scan.
    return status.get('state') in ARM_PHASES - {'RETURN_HOME'}
