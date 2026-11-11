"""Frozen pre-optimization scene selection for behavioral parity tests."""
from mission.scene import SceneBuffer, SceneSnapshot
class ReferenceSceneBuffer(SceneBuffer):
    def snapshot(self,now_ns,capture_ns=None):
        historical=capture_ns is not None and capture_ns!=now_ns
        live=None
        if historical:
            if not 0<=(now_ns-capture_ns)/1e9<=self.cfg['execution']['camera_max_age_s']:
                raise ValueError('RGBD_STALE_OR_REPEATED')
            # A historical pose is not a substitute for a healthy live stream.
            live=self.snapshot(now_ns)
        capture_ns=now_ns if capture_ns is None else capture_ns
        reference_ns=capture_ns if historical else now_ns
        # Current stream health is independent of the age of causal samples
        # selected to match delayed odometry or a camera exposure.
        for name,stream in (('arm',self.arms),('legs',self.legs)):
            ttl=self.cfg['limits'].get('legs_age_s',self.cfg['limits']['arm_age_s']) if name=='legs' else self.cfg['limits']['arm_age_s']
            if not stream or not 0<=(now_ns-stream[-1][0])/1e9<=ttl:
                raise ValueError('SCENE_SNAPSHOT_STALE:live_'+name)
        candidates=(b for b in reversed(self.bodies) if b.mono_ns<=capture_ns)
        newest=None;last_error='BODY_SNAPSHOT_MISSING';skipped=0
        for body in candidates:
            if newest is None:newest=body
            if (body.navigation_session_id!=newest.navigation_session_id
                    or body.tf_revision!=newest.tf_revision):
                raise ValueError('SCENE_SNAPSHOT_EPOCH_CHANGED')
            if not 0<=(reference_ns-body.mono_ns)/1e9<=self.cfg['limits']['sensor_age_s']:
                # Older candidates cannot regain freshness.
                last_error='SCENE_SNAPSHOT_STALE:body';break
            # Live requests choose the newest complete measured tuple, rather
            # than rejecting every tuple when the newest body lands in a joint
            # receipt gap. Camera requests keep their fixed exposure target.
            target=capture_ns if historical else body.mono_ns
            arm=next((a for a in reversed(self.arms) if a[0]<=target),None)
            leg=next((q for q in reversed(self.legs) if q[0]<=target),None)
            if arm is None or leg is None:
                last_error='COHERENT_JOINT_SNAPSHOT_MISSING'
            else:
                if arm[2]!=self.arms[-1][2]:raise ValueError('SCENE_SNAPSHOT_EPOCH_CHANGED')
                if live is not None and (body.navigation_session_id!=live.body.navigation_session_id
                        or body.tf_revision!=live.body.tf_revision or arm[2]!=live.owner_session_id):
                    raise ValueError('SCENE_SNAPSHOT_EPOCH_CHANGED')
                stamps=[body.mono_ns,arm[0],leg[0]]
                skew=(max(stamps)-min(stamps))/1e9
                self.selection=dict(reference_ns=reference_ns,body_ns=stamps[0],
                    arm_ns=stamps[1],legs_ns=stamps[2],skew_s=skew,
                    skipped_newer_incomplete=skipped,selected_age_s=(now_ns-body.mono_ns)/1e9)
                if any(stamp>reference_ns for stamp in stamps):raise ValueError('SCENE_SNAPSHOT_STALE:future')
                if skew<=self.cfg['frames']['max_state_skew_s']:
                    return SceneSnapshot(body,arm[1],leg[1],arm[0],leg[0],arm[2],self.arms[-1][0],self.legs[-1][0])
                last_error='SCENE_SNAPSHOT_SKEW'
            if historical:break # Do not move camera exposure to another body time.
            skipped+=1
        # Monotonic histories which have all crossed the exposure cannot
        # acquire another causal sample for it. Waiting out camera TTL cannot
        # repair this tuple; discard this frame and request a new exposure.
        if (historical and self.bodies and self.bodies[-1].mono_ns>capture_ns
                and self.arms[-1][0]>capture_ns and self.legs[-1][0]>capture_ns):
            self.selection.update(reference_ns=capture_ns,unmatchable_reason=last_error)
            raise ValueError('RGBD_SCENE_UNMATCHABLE')
        raise ValueError(last_error)
