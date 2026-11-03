"""Bounded retries on one immutable RGB-D observation; no actuator or worker IO."""
import time
import numpy as np


class GenerationBudgetExceeded(ValueError):
    pass


def retryable_candidate_failure(reason):
    return reason in ('IK_FAIL', 'NO_CANDIDATES', 'NO_TARGET_BOUND_GRASPS',
                      'CANDIDATE_GEOMETRY_REJECTED') or reason.startswith('NO_COLLISION_FREE_CANDIDATE:')


def run_generations(attempt, validate_scene, check_cancel, count, budget_s, records):
    deadline = None if budget_s is None else time.monotonic() + budget_s
    last_failure = None
    def remaining():
        check_cancel()
        left = float("inf") if deadline is None else deadline - time.monotonic()
        if left <= 0:
            raise GenerationBudgetExceeded('GRASP_GENERATION_BUDGET_EXHAUSTED')
        return left
    for index in range(count):
        started = time.monotonic()
        record = dict(generation=index + 1)
        records.append(record)
        try:
            remaining()
            validate_scene()
            result = attempt(remaining)
            record['status'] = 'SELECTED'
            return result
        except GenerationBudgetExceeded:
            record['status'] = 'BUDGET_EXHAUSTED'
            if last_failure is not None:
                raise ValueError(last_failure) from None
            raise
        except ValueError as error:
            reason = str(error)
            record.update(status='REJECTED', reason=reason)
            if not retryable_candidate_failure(reason):
                raise
            last_failure = reason
            if index + 1 == count:
                raise
        finally:
            record['elapsed_s'] = time.monotonic() - started


def unseen_grasps(grasps, tested, translation_m=1e-6, rotation_rad=1e-6):
    """Skip numerically identical geometry already exhausted on this observation.

    Input must already be ranked/NMS-limited. Both flips are tested downstream.
    Nearby orientations may have different IK outcomes, so they remain eligible.
    Previous failures stay associated only with this observation and stationary pose.
    """
    rows = np.asarray(grasps, dtype=float).reshape(-1, 17)
    old = np.asarray(tested, dtype=float).reshape(-1, 17)
    keep = []
    for row in rows:
        duplicate = False
        for prior in old:
            if not np.allclose(row[1:4],prior[1:4],rtol=0,atol=1e-6):continue
            if np.linalg.norm(row[13:16] - prior[13:16]) >= translation_m:
                continue
            rotation_delta=np.linalg.norm(row[4:13]-prior[4:13])
            if rotation_delta < np.sqrt(2)*rotation_rad:
                duplicate = True
                break
        if not duplicate:
            keep.append(row)
    return np.asarray(keep, dtype=float).reshape(-1, 17)
