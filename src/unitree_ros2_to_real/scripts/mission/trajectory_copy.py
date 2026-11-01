"""Isolated copies of JSON trajectories without recursive dispatch for each scalar."""
import copy


def copy_trajectory(value):
    points = value.get('points')
    # Only the validated JSON numeric matrix has the fast path. Unexpected
    # mutable values keep deepcopy semantics; validation remains at admission.
    if (type(points) is list and all(type(row) is list and
            all(type(x) in (int, float) for x in row) for row in points)):
        result = copy.deepcopy({key: item for key, item in value.items() if key != 'points'})
        result['points'] = [row.copy() for row in points]
        return result
    return copy.deepcopy(value)
