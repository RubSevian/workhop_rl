"""Inclusive stage timings, including failures/cancellation; no control decisions."""
from contextlib import contextmanager
import copy
import time


@contextmanager
def planning_stage(metrics, name, trace=None, request=None, progress=None, **details):
    previous = metrics.get('active_stage')
    row = dict(stage=name, status='running', started_ns=time.monotonic_ns(), **details)
    cpu = time.thread_time_ns()
    metrics.setdefault('planning_stages', []).append(row)
    metrics['active_stage'] = dict(row)
    if progress is not None and request is not None:
        try:progress.stage(request,dict(row))
        except Exception as error:progress.error=str(error)
    try:
        yield row
        row['status'] = 'completed'
    except BaseException as error:
        row.update(status='failed', reason=str(error) or type(error).__name__,
                   exception_type=type(error).__name__)
        if 'CANCELLED' in str(error) or isinstance(error, KeyboardInterrupt):
            row['status'] = 'cancelled'
        raise
    finally:
        row.update(elapsed_s=(time.monotonic_ns()-row['started_ns'])/1e9,
                   thread_cpu_s=(time.thread_time_ns()-cpu)/1e9)
        metrics['active_stage'] = previous
        if trace is not None and request is not None:
            trace.stage(request, copy.deepcopy(row))
        if progress is not None and request is not None:
            try:progress.stage(request,dict(row))
            except Exception as error:progress.error=str(error)


def ik_summary(calls):
    return dict(calls=len(calls), cache_hits=sum(bool(c.get('cache_hit')) for c in calls),
                elapsed_s=sum(c.get('elapsed_s', 0.) for c in calls),
                thread_cpu_s=sum(c.get('thread_cpu_s', 0.) for c in calls),
                attempts=sum(c.get('attempt_count', 0) for c in calls))
