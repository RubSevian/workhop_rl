"""Bounded bidirectional RRT-Connect; geometry and ownership stay with Mission.

No ROS, IK or actuator access. This is a local implementation of the algorithm,
not the OMPL binary used in the offline comparison. Every edge uses the caller's
validator; approximate paths are never returned.
"""
from __future__ import annotations
import time
import numpy as np


class SearchBudgetExceeded(Exception):
    """Internal search exhaustion, distinct from cancellation/scene failures."""


class SearchBudget:
    def __init__(self, timeout_s, max_extensions):
        self.started = time.monotonic()
        self.deadline = None if timeout_s is None else self.started + timeout_s
        self.max_extensions = max_extensions
        self.extensions = 0

    def check(self):
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise SearchBudgetExceeded('TIME_BUDGET')

    def extend(self):
        self.check()
        if self.extensions >= self.max_extensions:
            raise SearchBudgetExceeded('EXTENSION_BUDGET')
        self.extensions += 1


from .config import validate_rrt_config as validate_config


def _connect_steps(start, goal, lower, upper, edge_valid, check_cancel, budget, *,
            range_rad=.35, seed=41, shortcut_checks=8):
    """Return exact start→goal waypoints or None; propagate all external errors.

    Euclidean metric in bounded joint space, no wrapping through joint limits.
    Budget is shared by all candidates in a fallback pass. Cancellation is checked
    before each extension/edge; the caller also checks it inside collision work.
    """
    start, goal, lower, upper = [np.asarray(x, dtype=float).copy() for x in (start, goal, lower, upper)]
    if (start.ndim != 1 or any(x.shape != start.shape for x in (goal, lower, upper))
            or not np.isfinite([start, goal, lower, upper]).all()
            or np.any(lower >= upper) or not np.isfinite(range_rad) or range_rad <= 0):
        raise ValueError('INVALID_RRT_INPUT')
    if any(np.any(q < lower) or np.any(q > upper) for q in (start, goal)):
        return None

    def valid(a, b):
        check_cancel(); budget.check()
        result = edge_valid(a, b)
        check_cancel(); budget.check()
        return result

    if not valid(start, start) or not valid(goal, goal):
        return None
    if np.array_equal(start, goal):
        return np.array([start, goal])
    rng = np.random.default_rng(seed)
    trees = [([start], [-1]), ([goal], [-1])]

    def extend(tree, target):
        check_cancel(); budget.extend()
        vertices, parents = tree
        index = int(np.argmin(np.linalg.norm(np.asarray(vertices) - target, axis=1)))
        delta = target - vertices[index]
        distance = float(np.linalg.norm(delta))
        if distance == 0:
            return index, True
        reached = distance <= range_rad
        q = target.copy() if reached else vertices[index] + range_rad * delta / distance
        if not valid(vertices[index], q):
            return None, False
        vertices.append(q); parents.append(index)
        return len(vertices) - 1, reached

    def branch(tree, index):
        out = []
        while index >= 0:
            out.append(tree[0][index]); index = tree[1][index]
        return out[::-1]

    side = 0
    while True:
        check_cancel(); budget.check()
        a, b = trees[side], trees[1 - side]
        index, _ = extend(a, rng.uniform(lower, upper))
        yield
        if index is not None:
            while True:
                other, reached = extend(b, a[0][index])
                if other is None:
                    yield
                    break
                if reached:
                    left, right = branch(a, index), branch(b, other)
                    path = left + right[-2::-1]
                    if side == 1:
                        path.reverse()
                    # Deterministic checked shortcuts; retain a valid raw path if
                    # optional shortening runs out of budget.
                    path = np.asarray(path)
                    keep = np.ones(len(path), dtype=bool)
                    stack = [(0, len(path) - 1)]
                    try:
                        for _ in range(shortcut_checks):
                            if not stack: break
                            i, j = stack.pop()
                            if j <= i + 1: continue
                            if valid(path[i], path[j]): keep[i+1:j] = False
                            else:
                                mid = (i+j)//2
                                stack.extend([(mid,j), (i,mid)])
                    except SearchBudgetExceeded:
                        pass
                    check_cancel()
                    return path[keep]
                yield
        side = 1 - side


class ConnectSearch:
    """Resumable search: RNG, trees and a pending CONNECT survive each slice."""
    def __init__(self, *args, **kwargs):
        self._steps = _connect_steps(*args, **kwargs)
        self.done = False
        self.path = None

    def advance(self, extensions):
        for _ in range(extensions):
            if self.done:
                break
            try:
                next(self._steps)
            except StopIteration as result:
                self.done, self.path = True, result.value
        return self.path


def connect(*args, **kwargs):
    search = ConnectSearch(*args, **kwargs)
    while not search.done:
        search.advance(1)
    return search.path


def fallback_pregrasp(planning, start, current, reference_ns, candidates,
                      validate_complete, config, records):
    """Mission adapter: one shared search budget; full existing admission remains.

    Called only after the direct pass returns no candidate. The candidates are
    the cost-ordered pregrasp geometry rejections, not IK/staleness failures.
    """
    from .planning import CandidatePathRejected
    validate_config(config)
    budget = SearchBudget(config.get('timeout_s'), config['max_extensions'])
    bounds = planning._command_joint_bounds
    scene = planning.collision_scene(current)
    velocity = np.asarray(scene['velocity_limits'], dtype=float)[:6]
    rate = float(scene['command_rate_hz'])
    step = float(planning.cfg['collision']['sample_max_joint_step_rad'])

    def edge_valid(a, b):
        # Full checker, including enabled environment rules. The edge samples
        # satisfy velocity admission; execution is retimed by Planning.samples.
        n = max(1, int(np.ceil(max(np.max(np.abs(b-a))/step,
                                  np.max(np.abs(b-a)*rate/velocity)))))
        q = a + np.linspace(0., 1., n+1)[:, None]*(b-a)
        points = np.column_stack((q, np.full(n+1, start[6])))
        try:
            planning.checked(points, current, False, reference_ns=reference_ns)
            return True
        except CandidatePathRejected:
            return False

    prior_cancel = getattr(planning.collision, 'cancel_check', None)

    def search_cancel():
        if prior_cancel is not None:
            prior_cancel()
        budget.check()

    for index, candidate in enumerate(candidates):
        planning.collision.check_deadline()
        record = dict(candidate_cost=float(candidate.cost), seed=config['seed']+index,
                      status='SEARCHING', scope='pregrasp', implementation='local_rrt_connect')
        records.append(record)
        began = time.monotonic()
        try:
            planning.collision.cancel_check = search_cancel
            path = connect(start[:6], candidate.pregrasp_joints, bounds[:, 0], bounds[:, 1],
                           edge_valid, lambda: prior_cancel() if prior_cancel is not None else None,
                           budget, range_rad=config['range_rad'], seed=config['seed']+index,
                           shortcut_checks=config['shortcut_checks'])
            record['status'] = 'PATH_FOUND' if path is not None else 'INVALID_ENDPOINT'
        except SearchBudgetExceeded as error:
            record['status'] = str(error)
            return None
        except BaseException:
            record['status'] = 'INTERRUPTED_OR_ERROR'
            raise
        finally:
            planning.collision.cancel_check = prior_cancel
            record.update(search_s=time.monotonic()-began, total_extensions=budget.extensions)
        if path is None:
            continue
        if not np.array_equal(path[0], start[:6]) or not np.array_equal(path[-1], candidate.pregrasp_joints):
            raise ValueError('RRT_ENDPOINT_MISMATCH')
        record.update(waypoints=len(path), path_rad=path.tolist())
        # Search termination is distinct from existing final validation deadlines.
        # Check every phase, including closure and return with payload.
        began = time.monotonic()
        try:
            if validate_complete(candidate, path[1:]):
                record['status'] = 'SELECTED_COLLISION_CHECKED'
                return candidate
            record['status'] = 'FINAL_PATH_REJECTED'
        except BaseException:
            record['status'] = 'FINAL_VALIDATION_ERROR'
            raise
        finally:
            record['final_validation_s'] = time.monotonic()-began
    return None


def fallback_complete(planning, start, current, reference_ns, candidates,
                      validate, config, records, gripper_reference):
    """Try checked reverse paths before return RRT; share one generation budget.

    candidates contains (candidate, rejected phase). validate performs local
    preflight or complete command admission. Only geometry failures are retryable.
    The cache lives solely in this frozen scene and never serves final admission.
    """
    import copy
    from .planning import CandidatePathRejected
    validate_config(config)
    budget = SearchBudget(config.get('timeout_s'), config['max_extensions'])
    frozen = copy.deepcopy(current)
    gripper_reference = copy.deepcopy(gripper_reference)
    scene = planning.collision_scene(frozen)
    velocity = np.asarray(scene['velocity_limits'], dtype=float)[:6]
    rate = float(scene['command_rate_hz'])
    step = float(planning.cfg['collision']['sample_max_joint_step_rad'])
    bounds = planning._command_joint_bounds
    prior_cancel = getattr(planning.collision, 'cancel_check', None)
    cache = {}
    stats = dict(scope='generation', max_extensions=budget.max_extensions,
                 total_extensions=0, geometry_checks=0, cache_hits=0,
                 status='SEARCHING', implementation='local_rrt_connect')
    records.append(stats)
    outgoing, returning = [], []

    def cancel():
        if prior_cancel is not None:
            prior_cancel()
        budget.check()

    def edge_valid(a, b, payload):
        cancel()
        # Direction is retained: the checker may impose directional conditions.
        key = (bool(payload), np.asarray(a).tobytes(), np.asarray(b).tobytes())
        if key in cache:
            stats['cache_hits'] += 1
            return cache[key]
        n = max(1, int(np.ceil(max(np.max(np.abs(b-a))/step,
                                  np.max(np.abs(b-a)*rate/velocity)))))
        q = a + np.linspace(0., 1., n+1)[:, None]*(b-a)
        points = np.column_stack((q, np.full(n+1, start[6])))
        stats['geometry_checks'] += 1
        try:
            planning.checked(points, frozen, payload, reference_ns=reference_ns,
                             gripper_reference=gripper_reference)
            valid = True
        except CandidatePathRejected:
            valid = False
        cancel()
        cache[key] = valid
        return valid

    def job(candidate, index, direction, outbound=None):
        payload = direction == 'return'
        a = candidate.retreat_joints if payload else start[:6]
        b = start[:6] if payload else candidate.pregrasp_joints
        seed = config['seed'] + 2*index + int(payload)
        record = dict(scope=direction, candidate_cost=float(candidate.cost), seed=seed,
                      extensions=0, search_s=0., status='SEARCHING')
        records.append(record)
        search = ConnectSearch(a, b, bounds[:, 0], bounds[:, 1],
            lambda a,b:edge_valid(a,b,payload), cancel, budget,
            range_rad=config['range_rad'], seed=seed, shortcut_checks=config['shortcut_checks'])
        return dict(candidate=candidate, index=index, search=search, record=record, outbound=outbound)

    def admit(candidate, outbound, return_path=None, reverse=False):
        # Never use the search cache or search deadline for final command checks.
        planning.collision.cancel_check = prior_cancel
        try:
            return validate(candidate, outbound[1:], return_path, reverse=reverse)
        finally:
            planning.collision.cancel_check = cancel

    def reverse_or_queue(candidate, index, outbound):
        # validate constructs the connector retreat -> pregrasp when necessary,
        # retimes the whole reverse route and checks with the post-grasp model.
        if admit(candidate, outbound, reverse=True):
            return True
        returning.append(job(candidate, index, 'return', outbound))
        return False

    def advance_one(queue):
        task = queue.pop(0)
        search, record = task['search'], task['record']
        before, tick = budget.extensions, time.monotonic()
        try:
            remaining = budget.max_extensions - budget.extensions
            if remaining <= 0:
                raise SearchBudgetExceeded('EXTENSION_BUDGET')
            search.advance(min(4, remaining))
        finally:
            record['extensions'] += budget.extensions - before
            record['search_s'] += time.monotonic() - tick
        if not search.done:
            queue.append(task)
            return None
        if search.path is None:
            record['status'] = 'INVALID_ENDPOINT'
            return None
        path, candidate = search.path, task['candidate']
        record.update(status='PATH_FOUND', path_rad=path.tolist())
        if record['scope'] == 'pregrasp':
            accepted = reverse_or_queue(candidate, task['index'], path)
        else:
            accepted = admit(candidate, task['outbound'], path[1:])
        record['status'] = 'SELECTED_COLLISION_CHECKED' if accepted else 'FINAL_PATH_REJECTED'
        if accepted:
            stats['status'] = 'SELECTED_COLLISION_CHECKED'
            return candidate
        return None

    began = time.monotonic()
    try:
        # Preflight fixed approach/close/withdrawal before spending tree work.
        for index, (candidate, phase) in enumerate(candidates):
            planning.collision.check_deadline()
            if not validate(candidate, local_only=True):
                continue
            if phase == 'return_scan':
                outbound = np.array([start[:6], candidate.pregrasp_joints])
                if reverse_or_queue(candidate, index, outbound):
                    stats['status'] = 'SELECTED_COLLISION_CHECKED'
                    return candidate
            else:
                outgoing.append(job(candidate, index, 'pregrasp'))
        planning.collision.cancel_check = cancel
        # Reserve half for return when needed. If no return is ready, outbound
        # may use the whole budget. No candidate can consume a round alone.
        while outgoing and (not returning or budget.extensions < budget.max_extensions//2):
            selected = advance_one(outgoing)
            if selected is not None:
                return selected
        # Last resort: alternate directions, round-robin within each direction.
        # A long outbound queue must not starve the first return candidate.
        while outgoing or returning:
            for queue in (returning, outgoing):
                if queue:
                    selected = advance_one(queue)
                    if selected is not None:
                        return selected
        stats['status'] = 'NO_COMPLETE_PATH'
        return None
    except SearchBudgetExceeded as error:
        stats['status'] = str(error)
        return None
    except BaseException:
        stats['status'] = 'INTERRUPTED_OR_ERROR'
        raise
    finally:
        planning.collision.cancel_check = prior_cancel
        stats.update(total_extensions=budget.extensions, elapsed_s=time.monotonic()-began)
