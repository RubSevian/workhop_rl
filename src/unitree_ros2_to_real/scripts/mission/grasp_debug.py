"""One replaceable RGB-D/candidate snapshot; asynchronous and diagnostic only."""
import json
from pathlib import Path
import queue
import threading
import numpy as np
from .leg_plan_trace import diagnostic_json

STAGES={'decoded':'decoded_grasps','cloud':'pre_bbox_grasps','bbox':'bbox_grasps',
        'width':'width_grasps','depth':'grasps'}

class GraspDebug:
    def __init__(self,directory):
        self.path=Path(directory)/'latest.npz'
        self.error='';self.dropped=0;self.closed=False
        self.queue=queue.Queue(maxsize=1)
        self.thread=threading.Thread(target=self._write,daemon=True,name='grasp-snapshot')
        self.thread.start()

    def submit(self,color,depth,K,result,metadata):
        if self.closed:return
        try:
            meta=dict(metadata,counts=result.get('filter_counts',{}),parameters=result.get('filter_parameters',{}))
            data=dict(color=color.copy(),rgbd_depth_mm=depth.copy(),K=np.asarray(K).copy(),
                metadata=np.array(json.dumps(meta,default=diagnostic_json)))
            for stage,key in STAGES.items():
                # Missing stage data is an error, not a misleading empty stage.
                data[stage]=np.asarray(result[key]).copy().reshape(-1,17)
            self.queue.put_nowait(data)
        except queue.Full:self.dropped+=1
        except Exception as error:self.error=str(error)

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            while True:
                data=self.queue.get()
                if data is None:return
                temp=self.path.with_suffix('.tmp')
                with temp.open('wb') as stream:np.savez_compressed(stream,**data)
                temp.replace(self.path)
        except Exception as error:self.error=str(error)

    def close(self):
        self.closed=True
        try:self.queue.put_nowait(None)
        except queue.Full:
            # Drain the pending last snapshot on shutdown before the sentinel.
            def finish():
                while self.thread.is_alive():
                    try:self.queue.put(None,timeout=.1);return
                    except queue.Full:pass
            threading.Thread(target=finish,daemon=True).start()
        self.thread.join(timeout=2.)
