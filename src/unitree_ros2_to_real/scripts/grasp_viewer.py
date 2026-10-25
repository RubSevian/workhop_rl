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

def draw_snapshot(ax2,ax3,snapshot,stage,max_grasps=80):
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
    ax2.set_title(f'{stage}: {len(grasps)} candidates; all valid centers\nYellow: YOLO; green: expanded filter');ax2.axis('off')
    depth=snapshot['rgbd_depth_mm']
    if depth.shape!=color.shape[:2]:
        from PIL import Image
        depth=np.array(Image.fromarray(depth).resize((color.shape[1],color.shape[0]),Image.Resampling.NEAREST))
    v,u=np.mgrid[0:depth.shape[0]:4,0:depth.shape[1]:4];z=depth[::4,::4].astype(float)/1000
    keep=(z>meta['min_depth'])&(z<meta['max_depth'])
    xyz=np.column_stack(((u[keep]-K[0,2])*z[keep]/K[0,0],(v[keep]-K[1,2])*z[keep]/K[1,1],z[keep]))
    rgb=color[::4,::4,::-1][keep]/255.
    ax3.scatter(*xyz.T,c=rgb,s=1,alpha=.4)
    selected=grasps[np.argsort(-grasps[:,0],kind='stable')[:max_grasps]]
    segments=[]
    for grasp in selected:
        width=grasp[1];depth_g=grasp[3];R=grasp[4:13].reshape(3,3);t=grasp[13:16]
        # Schematic fingers: x=approach axis, y=jaw opening axis (GraspNet convention).
        local=np.array([[depth_g,width/2,0],[-.04,width/2,0],[-.04,-width/2,0],[depth_g,-width/2,0]])
        segments.append(local@R.T+t)
    if segments:ax3.add_collection3d(Line3DCollection(segments,colors='red',linewidths=.7))
    if len(xyz):
        lo=xyz.min(axis=0);hi=xyz.max(axis=0);center=(lo+hi)/2;radius=max((hi-lo).max()/2,.1)
        ax3.set_xlim(center[0]-radius,center[0]+radius);ax3.set_ylim(center[1]+radius,center[1]-radius)
        ax3.set_zlim(center[2]-radius,center[2]+radius)
    ax3.set_box_aspect((1,1,1));ax3.set_xlabel('Camera X (m)');ax3.set_ylabel('Camera Y (m)');ax3.set_zlabel('Camera Z (m)')
    ax3.set_title(f'RGB-D cloud (display subsampled); schematic grippers\nTop {len(selected)}/{len(grasps)} by score; drag to rotate')

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
