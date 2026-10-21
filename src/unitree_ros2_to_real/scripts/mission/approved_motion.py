"""Reverse a previously checked segment; never solve IK or renew approval time."""
import copy
import numpy as np
from .scene import distance


def reverse_segment(segment, current, cfg, *, known_empty):
    if not known_empty:raise ValueError('REVERSE_OBJECT_HOLD_UNKNOWN')
    if segment.get('collision_checked') is not True or segment.get('payload_checked') is not False:
        raise ValueError('REVERSE_APPROVAL_MISSING')
    points=np.asarray(segment.get('points'),dtype=float)
    if points.ndim!=2 or points.shape[1]!=7 or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('REVERSE_PATH_INVALID')
    old=segment.get('approval_scene_snapshot') or segment.get('scene_snapshot')
    fresh=current['scene_snapshot']
    if not old:raise ValueError('REVERSE_SCENE_PROVENANCE_MISSING')
    if (old['owner_session_id']!=fresh['owner_session_id']
            or old['body']['navigation_session_id']!=fresh['body']['navigation_session_id']
            or old['body']['tf_revision']!=fresh['body']['tf_revision']):
        raise ValueError('REVERSE_EPOCH_CHANGED')
    # Ordinary RL_ZERO leg motion does not invalidate the attempt's frozen
    # geometry. Epoch, body continuity and measured arm start remain live gates.
    pe,re=distance(old['body']['map_body'],fresh['body']['map_body'])
    if pe>cfg['frames']['continuity_translation_m'] or re>cfg['frames']['continuity_rotation_rad']:
        raise ValueError('REVERSE_BODY_DISCONTINUITY')
    if np.max(np.abs(np.asarray(current['q7'])-points[-1]))>cfg['execution']['start_tolerance_rad']:
        raise ValueError('REVERSE_START_CHANGED')
    result=copy.deepcopy(segment)
    result.setdefault('approval_scene_snapshot',copy.deepcopy(old))
    result.update(kind='move',points=points[::-1].tolist(),reused_approval=True,
                  reverse_of_request=segment.get('id'),scene_snapshot=fresh)
    # collision_context continues to identify the original approval. A fresh
    # scene proves admission facts; it does not manufacture a new collision pass.
    return result


def release_for_retry(segment, current, cfg, *, open_angle, rate, velocity, assessment):
    """Open at the checked approach endpoint after operator-defined NOT_GRASPED.

    Retain the original open-jaw approximation; no IK/collision recomputation.
    NOT_GRASPED authorizes retry, but does not physically prove an empty gripper.
    """
    if assessment!='NOT_GRASPED':raise ValueError('RETRY_RELEASE_ASSESSMENT_UNKNOWN')
    if segment.get('collision_gripper_reference',{}).get('model')!='open_gripper_approximation':raise ValueError('RETRY_OPEN_APPROXIMATION_MISSING')
    original=np.asarray(segment['points'],dtype=float)
    if original.ndim!=2 or original.shape[1]!=7:raise ValueError('REVERSE_PATH_INVALID')
    if not np.isfinite(open_angle) or abs(open_angle-original[-1,6])>1e-9:raise ValueError('RETRY_OPEN_ORIGINAL_TARGET_MISMATCH')
    # Explicitly validate six-axis endpoint plus original approval/epoch/body.
    # q7 is the one commanded opening change; it is not claimed measured open.
    check=copy.deepcopy(current);check['q7']=list(current['q7']);check['q7'][6]=float(original[-1,6])
    validated=reverse_segment(segment,check,cfg,known_empty=True)
    if not np.isfinite(rate) or not np.isfinite(velocity) or rate<=0 or velocity<=0:raise ValueError('RETRY_RELEASE_RATE_INVALID')
    start=np.asarray(current['q7'],dtype=float)
    # Same quintic blend as Planning.samples; peak derivative is 1.875.
    count=max(1,int(np.ceil(1.875*abs(open_angle-start[6])*rate/velocity)))
    if count>rate*cfg['execution']['max_plan_duration_s']:raise ValueError('RETRY_RELEASE_DURATION_EXCEEDED')
    phase=np.linspace(0.,1.,count+1);blend=10*phase**3-15*phase**4+6*phase**5
    points=np.repeat(start[None,:],count+1,axis=0);points[:,6]=start[6]+blend*(open_angle-start[6])
    validated.update(kind='release',operation='retry_open',points=points.tolist(),
        scene_snapshot=copy.deepcopy(current['scene_snapshot']),physical_geometry_qualified=False,
        collision_points=copy.deepcopy(segment.get('collision_points',segment['points'])),
        retry_release_assessment='NOT_GRASPED')
    return validated
