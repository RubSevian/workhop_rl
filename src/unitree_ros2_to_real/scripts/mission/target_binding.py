"""RGB-D target membership before NMS/IK; not contact or collision approval."""
import time
import numpy as np

from .config import validate_target_binding as validate_config


def filter_target_binding(grasps,depth_mm,K,mask,options,*,mask_source,min_depth,max_depth,cancel_check=None):
    started=time.monotonic();cfg=validate_config(options);grasps=np.asarray(grasps)
    if grasps.ndim!=2 or grasps.shape[1]!=17 or not np.isfinite(grasps).all():
        raise ValueError('TARGET_BINDING_INVALID_GRASPS')
    if not cfg['enabled']:return grasps,dict(enabled=False,candidates=[],input_count=len(grasps),kept_count=len(grasps))
    if mask_source!='segmentation':raise ValueError('TARGET_BINDING_SEGMENTATION_REQUIRED')
    dep=np.asarray(depth_mm,dtype=float)/1000.;K=np.asarray(K,dtype=float)
    if (mask is None or dep.ndim!=2 or np.asarray(mask).shape!=dep.shape
            or not np.isfinite(mask).all()):raise ValueError('TARGET_BINDING_INVALID_MASK')
    mask=np.asarray(mask,dtype=bool)
    if not mask.any():raise ValueError('TARGET_BINDING_EMPTY_MASK')
    if K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0:
        raise ValueError('TARGET_BINDING_INVALID_INTRINSICS')
    v,u=np.indices(dep.shape);valid=np.isfinite(dep)&(dep>=min_depth)&(dep<=max_depth)&(dep>0)
    points=np.stack(((u-K[0,2])*dep/K[0,0],(v-K[1,2])*dep/K[1,1],dep),axis=-1)[valid]
    target=mask[valid];rows=[];keep=[]
    for index,g in enumerate(grasps):
        if cancel_check is not None:cancel_check()
        local=(points-g[13:16])@g[4:13].reshape(3,3)
        inside=((local[:,0]>=g[3]-cfg['finger_length_m'])&(local[:,0]<=g[3])
                &(np.abs(local[:,1])<=g[1]/2)&(np.abs(local[:,2])<=cfg['finger_height_m']/2))
        all_cells=np.unique(np.floor(points[inside]/cfg['voxel_size_m']).astype(np.int64),axis=0)
        target_cells=np.unique(np.floor(points[inside&target]/cfg['voxel_size_m']).astype(np.int64),axis=0)
        fraction=len(target_cells)/len(all_cells) if len(all_cells) else 0.
        reason=('INSUFFICIENT_TARGET' if len(target_cells)<cfg['min_target_voxels'] else
                'LOW_TARGET_PURITY' if fraction<cfg['min_target_fraction'] else 'SUPPORTED')
        keep.append(reason=='SUPPORTED')
        rows.append(dict(input_index=index,target_voxels=len(target_cells),all_voxels=len(all_cells),
                         target_fraction=fraction,reason=reason))
    filtered=grasps[np.asarray(keep,dtype=bool)]
    return filtered,dict(enabled=True,input_count=len(grasps),kept_count=len(filtered),
                         mask_source=mask_source,mask_pixels=int(mask.sum()),target_depth_points=int(target.sum()),
                         elapsed_s=time.monotonic()-started,parameters=cfg,candidates=rows)
