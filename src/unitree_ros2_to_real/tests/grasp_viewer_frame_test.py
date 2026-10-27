"""Display frame pairing must never mix two different robot observations."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from grasp_viewer import display_transform,matched_arm,snapshot_cloud,draw_snapshot


def fixture():
    return dict(metadata=dict(request_id=2,min_depth=.05,max_depth=1.,parameters={}),color=np.zeros((8,8,3),dtype=np.uint8),rgbd_depth_mm=np.full((8,8),500),K=np.array([[100.,0,4],[0,100,4],[0,0,1]]),target_mask=np.ones((8,8),bool),decoded=np.empty((0,17)))


def test_mismatched_arm_never_used_and_capture_fallback():
    snap=fixture();arm=dict(request_id=1,camera_to_arm=np.eye(4).tolist())
    assert matched_arm(snap,arm) is None
    assert display_transform(snap,arm) is None
    t=np.eye(4);t[2,3]=.5;snap['metadata']['camera_to_arm']=t.tolist()
    np.testing.assert_equal(display_transform(snap,arm),t)
    arm['request_id']=2;np.testing.assert_equal(display_transform(snap,arm),np.eye(4))


def test_empty_candidates_and_top_camera_render():
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    snap=fixture();arm=dict(request_id=2,camera_to_arm=np.eye(4).tolist(),target_center=None)
    points,colors,mask=snapshot_cloud(snap);assert points.shape==(4,3) and mask.all()
    fig=Figure();FigureCanvasAgg(fig);ax2=fig.add_subplot(121);ax3=fig.add_subplot(122,projection='3d')
    for mode in ['base_top','base_3d','camera']:
        draw_snapshot(ax2,ax3,snap,'decoded',arm=arm,view=mode);fig.canvas.draw()
        assert ('camera' if mode=='camera' else 'base_link') in ax3.get_xlabel()
        if mode=='base_top':assert ax3.elev==90


def test_depth_detail_marks_depth_and_ignores_mismatched_cloud():
    from arm_plan_viewer import draw_depth_snapshot
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    snap=fixture();data=dict(request_id=3,camera_to_arm=np.eye(4).tolist(),candidates=[dict(candidate='test',depth_m=.04,width_m=.1,translation_camera_m=[0,0,.5],rotation_camera=np.eye(3).tolist())])
    fig=Figure();FigureCanvasAgg(fig);ax=fig.add_subplot(projection='3d')
    draw_depth_snapshot(ax,data,'test',snapshot=snap);fig.canvas.draw()
    assert 'No matching cloud' in ax.get_title()
    line=next(line for line in ax.lines if line.get_label()=='GraspNet depth: 40 mm')
    np.testing.assert_allclose(line.get_data_3d()[0],[0,.04])
    assert 'base_link' in ax.get_xlabel()
    assert ax.elev==23
    draw_depth_snapshot(ax,data,'test',snapshot=snap,top=True)
    assert ax.elev==90
    data['request_id']=2;draw_depth_snapshot(ax,data,'test',snapshot=snap);fig.canvas.draw()
    assert 'No matching cloud' not in ax.get_title()
