"""Wait for delayed measurements, never move a camera exposure timestamp."""
import time


def transient_scene_error(reason):
    return (reason in ('BODY_SNAPSHOT_MISSING','COHERENT_JOINT_SNAPSHOT_MISSING',
                       'SCENE_SNAPSHOT_SKEW','BODY_TF_STALE_OR_UNBOUND')
            or reason.startswith('SCENE_SNAPSHOT_STALE:'))


def camera_context(read, capture_ns, max_age_s, poll_s):
    deadline=capture_ns+round(max_age_s*1e9)
    while True:
        if time.monotonic_ns()>deadline:
            raise ValueError('RGBD_STALE_OR_REPEATED')
        try:
            result=read()
        except ValueError as error:
            reason=str(error)
            if not transient_scene_error(reason):raise
            remaining=(deadline-time.monotonic_ns())/1e9
            if remaining<=0:raise ValueError('RGBD_STALE_OR_REPEATED') from error
            time.sleep(min(poll_s,remaining))
            continue
        if time.monotonic_ns()>deadline:
            raise ValueError('RGBD_STALE_OR_REPEATED')
        return result


def select_camera_frame(frames, admit, check_deadline, max_age_s, poll_s):
    """Try a bounded burst, newest first; every exposure keeps its own TTL.

    frames are (capture_mono_ns, opaque_frame). admit must not wait: this loop
    waits for real history while allowing another measured exposure to match.
    """
    remaining=list(frames)
    ttl=round(max_age_s*1e9)
    while remaining:
        check_deadline()
        retry=[]
        for stamp,frame in reversed(remaining):
            now=time.monotonic_ns()
            if not 0<=now-stamp<=ttl:continue
            try:
                check_deadline()
                result=admit(frame)
            except ValueError as error:
                if str(error)=='RGBD_STALE_OR_REPEATED':continue
                if not (transient_scene_error(str(error)) or str(error)=='RGBD_TF_FEEDBACK_STALE'):raise
                retry.append((stamp,frame));continue
            check_deadline()
            # Descheduling inside admission cannot renew an exposure.
            if 0<=time.monotonic_ns()-stamp<=ttl:return result
        remaining=list(reversed(retry))
        if remaining:
            delay=min(poll_s,max(0.,(remaining[-1][0]+ttl-time.monotonic_ns())/1e9))
            if delay>0:time.sleep(delay)
    raise ValueError('RGBD_STALE_OR_REPEATED')
