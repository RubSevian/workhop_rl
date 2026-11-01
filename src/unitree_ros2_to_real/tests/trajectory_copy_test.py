import copy
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from mission.trajectory_copy import copy_trajectory


def test_numeric_trajectory_is_equal_and_independent():
    original=dict(points=[[1.,2.,3.,4.,5.,6.,7.] for _ in range(6000)],scene_snapshot={'x':[1,2]},kind='move')
    clone=copy_trajectory(original)
    assert clone==copy.deepcopy(original)
    original['points'][0][0]=99
    original['scene_snapshot']['x'][0]=99
    assert clone['points'][0][0]==1 and clone['scene_snapshot']['x'][0]==1
    clone['points'][1][1]=88
    assert original['points'][1][1]==2


def test_unexpected_nested_value_preserves_deepcopy_semantics():
    original={'points':[[{'x':[1]}]],'meta':{'z':[]}}
    clone=copy_trajectory(original)
    clone['points'][0][0]['x'].append(2)
    assert original['points'][0][0]['x']==[1]
