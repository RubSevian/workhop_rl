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
