#!/usr/bin/env python3
"""Read-only GraspNet stage viewer. Reads one snapshot, never commands hardware."""
import argparse
import json
from pathlib import Path
import numpy as np

STAGES=('decoded','cloud','bbox','width','depth')

def load_snapshot(path):
    with np.load(path,allow_pickle=False) as data:
        snapshot={key:data[key].copy() for key in data.files}
    if 'rgbd_depth_mm' not in snapshot:
        raise ValueError('Old snapshot has no RGB-D depth image; run a new GraspNet calculation')
    if snapshot['rgbd_depth_mm'].ndim!=2 or snapshot['color'].ndim!=3:
        raise ValueError('Invalid RGB-D snapshot dimensions')
    snapshot['metadata']=json.loads(str(snapshot['metadata']))
    return snapshot

def project(points,K):
    points=np.asarray(points)
    valid=np.isfinite(points).all(axis=1)&(points[:,2]>1e-6)
    uv=np.full((len(points),2),np.nan)
    uv[valid]=points[valid,:2]/points[valid,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]
    return uv,valid

def matched_arm(snapshot,arm):
    return arm if arm and arm.get('request_id')==snapshot['metadata'].get('request_id') else None


def display_transform(snapshot,arm=None):
    arm=matched_arm(snapshot,arm)
    value=arm.get('camera_to_arm') if arm else snapshot['metadata'].get('camera_to_arm')
    if value is None:return None
    t=np.asarray(value,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]):raise ValueError('Invalid display transform')
    if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5):raise ValueError('Invalid display rotation')
    return t


def snapshot_cloud(snapshot,step=4):
    depth=snapshot['rgbd_depth_mm'];K=snapshot['K'];meta=snapshot['metadata']
    v,u=np.mgrid[0:depth.shape[0]:step,0:depth.shape[1]:step];z=depth[::step,::step].astype(float)/1000
    keep=np.isfinite(z)&(z>meta['min_depth'])&(z<meta['max_depth'])
    xyz=np.column_stack(((u[keep]-K[0,2])*z[keep]/K[0,0],(v[keep]-K[1,2])*z[keep]/K[1,1],z[keep]))
    # Snapshot color comes from the BGR camera pipeline.
    rgb=snapshot['color'][::step,::step,::-1][keep]/255.
    target=snapshot.get('target_mask',np.zeros_like(depth,dtype=bool))[::step,::step][keep].astype(bool)
    return xyz,rgb,target


def draw_snapshot(ax2,ax3,snapshot,stage,max_grasps=80,*,arm=None,view='base_top'):

    from matplotlib.patches import Rectangle
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    ax2.clear();ax3.clear()
    if stage not in snapshot:
        ax2.set_title('Stage not recorded in this snapshot');return
    color=snapshot['color'];K=snapshot['K'];meta=snapshot['metadata'];grasps=snapshot[stage]
    ax2.imshow(color[:,:,::-1])
    if stage=='target_binding' and 'target_mask' in snapshot:
        overlay=np.zeros((*snapshot['target_mask'].shape,4));overlay[snapshot['target_mask'].astype(bool)]=[0,1,0,.25];ax2.imshow(overlay)
    uv,valid=project(grasps[:,13:16],K)
    ax2.scatter(uv[valid,0],uv[valid,1],s=10,c='cyan')
    bbox=meta['parameters'].get('bbox_xyxy')
    if bbox is not None:
        x1,y1,x2,y2=bbox
        ax2.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,fill=False,edgecolor='yellow'))
        ratio=max(1.,meta['parameters']['target_expand_ratio']);margin=meta['parameters']['target_margin_px']
        cx=(x1+x2)/2;cy=(y1+y2)/2
        left=max(0,round(cx-(x2-x1)*ratio/2)-margin);top=max(0,round(cy-(y2-y1)*ratio/2)-margin)
        right=min(color.shape[1]-1,round(cx+(x2-x1)*ratio/2)+margin)
        bottom=min(color.shape[0]-1,round(cy+(y2-y1)*ratio/2)+margin)
        ax2.add_patch(Rectangle((left,top),right-left,bottom-top,fill=False,edgecolor='lime',linestyle='--'))
    if bbox is not None:
        ax2.scatter((bbox[0]+bbox[2])/2,(bbox[1]+bbox[3])/2,c='yellow',marker='+',s=100,label='YOLO box centre (reference)')
    paired=matched_arm(snapshot,arm)
    center_data=paired.get('target_center') if paired else None
    if center_data and center_data.get('active'):
        camera=np.asarray(paired['camera_to_arm'])
        center_cam=(np.asarray(center_data['center_base_m'])-camera[:3,3])@camera[:3,:3]
        pos,ok=project(center_cam[None],K)
        if ok[0]:ax2.scatter(*pos[0],c='magenta',marker='x',s=100,label='3D centre projected (ranking)')
        chosen=next((c for c in paired['candidates'] if c['candidate']==paired.get('selected')),None)
        feature=(chosen or {}).get('ranking_features',{}).get('center',{})
        if feature.get('active'):
            point_cam=(np.asarray(feature['grasp_center_base_m'])-camera[:3,3])@camera[:3,:3]
            point,valid_point=project(point_cam[None],K)
            if valid_point[0]:
                ax2.scatter(*point[0],c='lime',marker='o',s=60,label='Selected working jaw centre')
                if ok[0]:ax2.plot([pos[0,0],point[0,0]],[pos[0,1],point[0,1]],c='magenta',ls='--')
    if bbox is not None or (center_data and center_data.get('active')):ax2.legend(fontsize=7,loc='upper left')
    ax2.set_title(f'{stage}: {len(grasps)} candidates; projected origins\nYellow: YOLO; green: expanded filter');ax2.axis('off')
    xyz,rgb,target_mask=snapshot_cloud(snapshot)
    transform=display_transform(snapshot,arm) if view!='camera' else None
    def convert(points):
        return points if transform is None else np.asarray(points)@transform[:3,:3].T+transform[:3,3]
    xyz=convert(xyz)
    crop=None
    if view=='base_top' and transform is not None and target_mask.any():
        focus=xyz[target_mask];crop=(focus.min(0)-.04,focus.max(0)+.04)
        keep=np.all((xyz>=crop[0])&(xyz<=crop[1]),axis=1)
        xyz,rgb,target_mask=xyz[keep],rgb[keep],target_mask[keep]
    ax3.scatter(*xyz.T,c=rgb,s=1,alpha=.25)
    if target_mask.any():ax3.scatter(*xyz[target_mask].T,c='cyan',s=2,alpha=.5)
    selected=grasps[np.argsort(-grasps[:,0],kind='stable')[:max_grasps]]
    if crop is not None:
        origins=convert(selected[:,13:16]);selected=selected[np.all((origins>=crop[0])&(origins<=crop[1]),axis=1)]
    segments=[]
    for grasp in selected:
        width=grasp[1];depth_g=grasp[3];R=grasp[4:13].reshape(3,3);t=grasp[13:16]
        # Schematic fingers: x=approach axis, y=jaw opening axis (GraspNet convention).
        local=np.array([[depth_g,width/2,0],[-.04,width/2,0],[-.04,-width/2,0],[depth_g,-width/2,0]])
        segments.append(convert(local@R.T+t))
    if segments:ax3.add_collection3d(Line3DCollection(segments,colors='red',linewidths=.7))
    if len(xyz):
        focus=xyz[target_mask] if view=='base_top' and transform is not None and target_mask.any() else xyz
        lo=focus.min(axis=0);hi=focus.max(axis=0);center=(lo+hi)/2;radius=max((hi-lo).max()*.65,.15)
        ax3.set_xlim(center[0]-radius,center[0]+radius);ax3.set_ylim(center[1]-radius,center[1]+radius)
        ax3.set_zlim(center[2]-radius,center[2]+radius)
    frame='base_link' if transform is not None else 'camera'
    paired=matched_arm(snapshot,arm)
    center_data=paired.get('target_center') if paired else None
    if transform is not None and center_data and center_data.get('active'):
        center=np.asarray(center_data['center_base_m'])
        ax3.scatter(*center,c='magenta',marker='x',s=80,label='Visible target centre')
        chosen=next((c for c in paired['candidates'] if c['candidate']==paired.get('selected')),None)
        feature=(chosen or {}).get('ranking_features',{}).get('center',{})
        if feature.get('active'):
            point=np.asarray(feature['grasp_center_base_m'])
            ax3.plot(*np.array([center,point]).T,c='magenta',label=f"Centre distance {feature['distance_m']*1000:.1f} mm")
        ax3.legend(fontsize=7)
    ax3.set_box_aspect((1,1,1));ax3.set_xlabel(f'{frame} X (m)');ax3.set_ylabel('Y (m)');ax3.set_zlabel('Z (m)')
    ax3.set_proj_type('ortho' if view=='base_top' else 'persp')
    if view=='base_top' and transform is not None:
        ax3.set_zticks([]);ax3.set_zlabel('')
    ax3.view_init(elev=90 if view=='base_top' and transform is not None else 25,azim=-90 if view=='base_top' and transform is not None else -65)
    warning=' | No matching transform: camera frame' if view!='camera' and transform is None else ''
    ax3.set_title(f'{frame}: top {len(selected)}/{len(grasps)} schematic grippers\nSaved cloud; base Z is not gravity when body tilts'+warning)

def main():
    import matplotlib.pyplot as plt
    from matplotlib.widgets import RadioButtons
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file',type=Path,default=Path.home()/'go2_diploma/sim2real/runtime/grasp_debug/latest.npz')
    parser.add_argument('--max-grasps',type=int,default=80)
    args=parser.parse_args()
    if args.max_grasps<1:parser.error('--max-grasps must be positive')
    fig=plt.figure(figsize=(14,7));ax2=fig.add_subplot(121);ax3=fig.add_subplot(122,projection='3d')
    fig.subplots_adjust(bottom=.23);selector=RadioButtons(fig.add_axes([.02,.02,.14,.18]),STAGES+('target_binding',))
    data=None;stamp=None
    def render(_=None):
        if data is not None:
            draw_snapshot(ax2,ax3,data,selector.value_selected,args.max_grasps)
            meta=data['metadata'];counts=' -> '.join(f'{name}: {len(data[name])}' for name in STAGES)
            fig.suptitle(f'Request {meta["request_id"]} | {counts}\nSaved observation; not live camera or robot execution')
            fig.canvas.draw_idle()
    def refresh():
        nonlocal data,stamp
        try:
            current=args.file.stat().st_mtime_ns
            if current!=stamp:data=load_snapshot(args.file);stamp=current;render()
        except FileNotFoundError:
            fig.suptitle('Waiting for a GraspNet calculation in debug mode');fig.canvas.draw_idle()
        except Exception as error:
            fig.suptitle(f'Snapshot error: {error}');fig.canvas.draw_idle()
    selector.on_clicked(render)
    timer=fig.canvas.new_timer(interval=1000);timer.add_callback(refresh);timer.start()
    refresh();plt.show()

if __name__=='__main__':main()
