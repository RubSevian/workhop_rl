"""Bounded FK camera framing; no actuator IO. New tilt paths require collision admission."""
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


def frame_bounds(mask, bbox, image_shape, margin_fraction):
    """Union of visible mask and detector box; cannot infer hidden object extent."""
    height, width = image_shape[:2]
    if not 0 < margin_fraction < .25:
        raise ValueError('INVALID_VIEW_MARGIN')
    b = np.asarray(bbox, dtype=float)
    m = np.asarray(mask)
    if (b.shape != (4,) or not np.isfinite(b).all() or m.shape != (height, width)
            or not np.any(m) or b[2] <= b[0] or b[3] <= b[1]):
        raise ValueError('INVALID_TARGET_VIEW')
    yy, xx = np.nonzero(m)
    b = np.array([min(b[0],xx.min()), min(b[1],yy.min()),
                  max(b[2],xx.max()), max(b[3],yy.max())])
    limits = np.array([margin_fraction*width, margin_fraction*height,
                       (1-margin_fraction)*width-1, (1-margin_fraction)*height-1])
    good = bool(np.all(b[:2] >= limits[:2]) and np.all(b[2:] <= limits[2:]))
    return dict(bounds_px=b.tolist(), margin_fraction=margin_fraction, full_view=good,
                clipped=bool(b[0]<=0 or b[1]<=0 or b[2]>=width-1 or b[3]>=height-1))


def select_full_view(current, camera_pose, world_corners, K, image_shape,
                     lower, upper, yaw_bounds, scan_pitch, max_yaw, max_pitch,
                     max_translation, margin_fraction, check_cancel=lambda: None):
    """Bounded joint1/joint4 FK search. Result requires path checking and fresh RGB-D.

    Corners at observed target depth predict framing, not visibility of unseen parts.
    No extra actuator, thread, unconstrained IK or accumulating pitch corrections.
    """
    q = np.asarray(current, dtype=float)
    lo, hi = np.asarray(lower,float), np.asarray(upper,float)
    K = np.asarray(K,float); corners=np.asarray(world_corners,float)
    if (q.shape!=(6,) or lo.shape!=(6,) or hi.shape!=(6,) or K.shape!=(3,3)
            or corners.shape!=(4,3) or not all(np.isfinite(a).all() for a in (q,lo,hi,K,corners))
            or np.any(lo>hi) or min(max_yaw,max_pitch,max_translation)<=0
            or not 0<margin_fraction<.25):
        raise ValueError('INVALID_FULL_VIEW_LIMITS')
    height,width=image_shape[:2];origin=np.asarray(camera_pose(q))[:3,3]
    def evaluate(candidate):
        pose=np.asarray(camera_pose(candidate));local=(corners-pose[:3,3])@pose[:3,:3]
        if (np.any(local[:,2]<=0) or not np.isfinite(local).all()
                or np.linalg.norm(pose[:3,3]-origin)>max_translation+1e-9):return None
        pixels=local@K.T;pixels=pixels[:,:2]/pixels[:,2,None]
        uv=pixels/np.array([width-1,height-1]);low=uv.min(0);high=uv.max(0)
        overflow=np.maximum(margin_fraction-low,0)+np.maximum(high-(1-margin_fraction),0)
        center=(low+high)/2
        cost=float(100*np.sum(overflow**2)+np.sum((center-.5)**2))
        return cost,dict(predicted_bounds_px=np.r_[pixels.min(0),pixels.max(0)].tolist(),
            predicted_full_view=bool(np.all(overflow<=1e-9)),center_error_fraction=float(np.linalg.norm(center-.5)),
            camera_translation_m=float(np.linalg.norm(pose[:3,3]-origin)))
    before=evaluate(q)
    if before is None:raise ValueError('REFINE_TARGET_BEHIND_CAMERA')
    best=q.copy();score,detail=before
    ylo=max(lo[0],yaw_bounds[0],q[0]-max_yaw);yhi=min(hi[0],yaw_bounds[1],q[0]+max_yaw)
    plo=max(lo[3],scan_pitch-max_pitch);phi=min(hi[3],scan_pitch+max_pitch)
    if ylo>yhi or plo>phi:raise ValueError('REFINE_START_OUTSIDE_LIMITS')
    for yaw in np.unique(np.r_[np.linspace(ylo,yhi,21),q[0]]):
        check_cancel()
        for pitch in np.unique(np.r_[np.linspace(plo,phi,21),q[3]]):
            candidate=q.copy();candidate[0]=yaw;candidate[3]=pitch
            if np.any(candidate<lo) or np.any(candidate>hi):continue
            value=evaluate(candidate)
            if value is not None and value[0]<score-1e-12:
                best=candidate;score,detail=value
    return best,dict(mode='bounded_full_view',before=before[1],after=detail,
                     delta_rad=(best-q).tolist(),requires_fresh_verification=True)
