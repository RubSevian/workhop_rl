"""Consume only measured native observer samples; no clock fitting/interpolation."""
import math

class NativeLegInput:
    def __init__(self,scene,cfg):
        self.scene=scene;self.cfg=cfg;self.session='';self.last_batch=0
        self.received=0;self.dropped=0;self.batches=0
    def apply(self,batch,now_ns):
        if (not batch.source_session_id or type(batch.observed_ns) is not int
                or not 0<batch.observed_ns<=now_ns
                or len(batch.samples)>self.cfg['frames'].get('leg_collector_capacity',512)):
            raise ValueError('INVALID_NATIVE_LEG_BATCH')
        if batch.observed_ns<=self.last_batch:return None
        samples=[];previous=0
        for sample in batch.samples:
            if (type(sample.observed_ns) is not int or not previous<sample.observed_ns<=batch.observed_ns
                    or len(sample.q)!=12 or not all(math.isfinite(q) for q in sample.q)):
                raise ValueError('INVALID_NATIVE_LEG_SAMPLE')
            previous=sample.observed_ns
            if now_ns-sample.observed_ns<=round(self.cfg['frames']['odometry_history_s']*1e9):
                samples.append((sample.observed_ns,tuple(sample.q)))
        if batch.source_session_id!=self.session:self.scene.legs.clear()
        self.session=batch.source_session_id;self.last_batch=batch.observed_ns
        self.received=batch.samples_received;self.dropped=batch.samples_dropped;self.batches+=1
        for stamp,q in samples:self.scene.leg(q,stamp)
        return samples[-1] if samples else None
    def status(self):
        return dict(mode='native_callback_batch',source_session_id=self.session,
                    samples_received=self.received,samples_dropped=self.dropped,batches=self.batches,
                    last_batch_ns=self.last_batch)
