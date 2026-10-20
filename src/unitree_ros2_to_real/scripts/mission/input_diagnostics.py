"""Bounded callback telemetry. Times are local MONOTONIC, not DDS receipt time."""
import copy

class InputDiagnostics:
    def __init__(self):
        self.streams={};self.last_tick=None;self.tick_gap_s=None;self.max_tick_gap_s=0.

    def tick(self,now):
        self.tick_gap_s=None if self.last_tick is None else now-self.last_tick
        self.max_tick_gap_s=max(self.max_tick_gap_s,self.tick_gap_s or 0.)
        self.last_tick=now

    def callback(self,name,start,end,outcome,value=None,publishers=None):
        if name not in ('r3','nav','owner','perception'):return
        row=self.streams.setdefault(name,dict(callbacks=0,accepted=0,rejected=0,max_gap_s=0.,max_duration_s=0.))
        gap=None if 'last_callback_s' not in row else start-row['last_callback_s']
        duration=end-start
        row.update(callbacks=row['callbacks']+1,last_callback_s=start,last_gap_s=gap,
            last_duration_s=duration,max_duration_s=max(row['max_duration_s'],duration),
            max_gap_s=max(row['max_gap_s'],gap or 0.),last_outcome=outcome,publishers=publishers)
        stamp=value.get('observed_ns') if isinstance(value,dict) else None
        row['last_producer_stamp_ns']=stamp if type(stamp) is int else None
        row['producer_age_at_decode_s']=end-stamp/1e9 if type(stamp) is int else None
        key='accepted' if outcome=='accepted' else 'rejected';row[key]+=1
        if outcome=='accepted':row['last_accepted_callback_s']=end

    def snapshot(self,now):
        rows=copy.deepcopy(self.streams)
        for row in rows.values():row['callback_silence_s']=now-row['last_callback_s']
        return dict(streams=rows,tick_gap_s=self.tick_gap_s,max_tick_gap_s=self.max_tick_gap_s)
