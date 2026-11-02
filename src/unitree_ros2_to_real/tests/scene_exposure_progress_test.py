"""Exercise camera admission against monotonic body/arm/leg histories."""
import sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.scene import SceneBuffer, BodyPose
from mission.scene_wait import camera_context, select_camera_frame
from mission.core import retryable_observation

CAP=1_000_000_000
CFG={'limits':{'sensor_age_s':.5,'arm_age_s':.5,'legs_age_s':.5},
     'frames':{'max_state_skew_s':.05},'execution':{'camera_max_age_s':2.}}
def body(stamp):
    return BodyPose(stamp,stamp,'nav','tf',np.eye(4).tolist(),np.eye(4).tolist())
def history(crossed=True):
    scene=SceneBuffer(CFG)
    scene.bodies.append(body(CAP-60_000_000))
    scene.arm([0.]*7,CAP-61_000_000,'owner');scene.leg([0.]*12,CAP-61_000_000)
    scene.arm([0.]*7,CAP-1_000_000,'owner');scene.leg([0.]*12,CAP-1_000_000)
    if crossed:scene.bodies.append(body(CAP+10_000_000))
    scene.arm([0.]*7,CAP+10_000_000,'owner');scene.leg([0.]*12,CAP+10_000_000)
    return scene

def test_closed_exposure_cannot_be_repaired_by_waiting():
    scene=history();now=CAP+100_000_000
    with patch('mission.scene_wait.time.monotonic_ns',return_value=now),patch('mission.scene_wait.time.sleep') as sleep:
        with pytest.raises(ValueError,match='RGBD_SCENE_UNMATCHABLE'):
            camera_context(lambda:scene.snapshot(now,CAP),CAP,2.,.02)
        sleep.assert_not_called()
    assert scene.selection['unmatchable_reason']=='SCENE_SNAPSHOT_SKEW'
    assert scene.selection['skew_s']>.05
    assert retryable_observation('RGBD_SCENE_UNMATCHABLE')

def test_delayed_body_can_still_repair_exposure():
    scene=history(False);now=CAP+100_000_000
    with pytest.raises(ValueError,match='SCENE_SNAPSHOT_SKEW'):scene.snapshot(now,CAP)
    scene.bodies.append(body(CAP-2_000_000))
    result=scene.snapshot(now,CAP)
    assert result.body.mono_ns==CAP-2_000_000
    assert max(result.body.mono_ns,result.feedback_ns,result.legs_ns)<=CAP

def test_unmatchable_candidate_does_not_hide_valid_other_frame():
    scene=history();now=CAP+100_000_000
    with patch('mission.scene_wait.time.monotonic_ns',return_value=now),patch('mission.scene_wait.time.sleep') as sleep:
        result=select_camera_frame([(CAP,CAP),(CAP+10_000_000,CAP+10_000_000)],
            lambda cap:scene.snapshot(now,cap),lambda:None,2.,.02)
        assert result.body.mono_ns==CAP+10_000_000
        sleep.assert_not_called()

def test_live_stale_and_epoch_faults_are_not_downgraded_to_frame_retry():
    scene=history()
    with pytest.raises(ValueError,match='live_arm'):scene.snapshot(CAP+600_000_000,CAP)
    scene=history();scene.bodies[-1]=BodyPose(CAP+10_000_000,CAP+10_000_000,'other','tf',np.eye(4).tolist(),np.eye(4).tolist())
    with pytest.raises(ValueError,match='EPOCH_CHANGED'):scene.snapshot(CAP+100_000_000,CAP)

def test_real_perception_wait_checks_cancellation_and_records_tuple():
    import ast, threading
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    import time
    from mission.scene_wait import transient_scene_error
    from mission.collision import CollisionCancelled
    source=Path(__file__).parents[1]/'scripts/grasp_perception.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Perception')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='sample_context')
    scope=dict(time=time,camera_context=camera_context,transient_scene_error=transient_scene_error)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
    collision=NS(check_deadline=Mock(side_effect=[None,CollisionCancelled('cancelled')]))
    scene=history(False)
    node=NS(lock=threading.Lock(),backend=NS(compute_metrics={},collision=collision),
        body_source=NS(provider=NS(fault='')),scene_error='',scene_buffer=scene,cfg=dict(CFG,execution=dict(CFG['execution'],tick_s=.02)))
    with patch('time.monotonic_ns',return_value=CAP+100_000_000),patch('time.sleep'):
        with pytest.raises(CollisionCancelled):scope['sample_context'](node,CAP,CAP)
    assert collision.check_deadline.call_count==2
    metrics=node.backend.compute_metrics
    assert metrics['frame_scene_rejections']=={'SCENE_SNAPSHOT_SKEW':1}
    assert metrics['last_scene_rejection']['selection']['body_ns']==CAP-60_000_000
