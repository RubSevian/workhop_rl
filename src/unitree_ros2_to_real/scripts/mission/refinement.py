"""Bounded one-joint camera centering, without IK or hardware IO."""
import numpy as np


def select_scan_yaw(current, camera_point, lower, upper, max_yaw, tolerance):
    if not np.isfinite([current,lower,upper,max_yaw,tolerance]).all() or max_yaw<=0 or tolerance<0 or lower>upper:
        raise ValueError('INVALID_REFINE_LIMITS')
    def error(q):
        p=np.asarray(camera_point(float(q)),dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all() or p[2]<=0:
            return float('inf')
        return abs(float(np.arctan2(p[0],p[2])))
    before=error(current)
    if not np.isfinite(before):raise ValueError('REFINE_TARGET_BEHIND_CAMERA')
    lo=max(lower,current-max_yaw);hi=min(upper,current+max_yaw)
    if lo>hi:raise ValueError('REFINE_START_OUTSIDE_LIMITS')
    yaw=current;after=before
    if before>tolerance:
        # Include the current pose; never choose a worse view or cross limits.
        for q in np.linspace(lo,hi,101):
            e=error(q)
            if e<after: yaw=float(q);after=e
    return yaw,dict(mode='scan_joint1_yaw',before_rad=before,after_rad=after,
                    delta_rad=yaw-current,centered=after<=tolerance)
