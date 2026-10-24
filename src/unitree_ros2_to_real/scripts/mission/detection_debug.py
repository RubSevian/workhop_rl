"""Bounded best-effort image diagnostics; never grants motion authority."""
import json
from pathlib import Path
import queue
import threading
import time
from .leg_plan_trace import diagnostic_json


class DetectionDebug:
    def __init__(self, directory, archive=True):
        self.root=Path(directory);self.archive=archive
        self.directory=self.root/(str(time.time_ns()) if archive else "live")
        self.queue=queue.Queue(maxsize=4);self.dropped=0;self.error='';self.closed=False
        self.thread=threading.Thread(target=self._write,daemon=True,name='detection-images')
        self.thread.start()

    def submit(self, image, metadata):
        if self.closed or image is None:return
        try:self.queue.put_nowait((image.copy(),dict(metadata)))
        except queue.Full:self.dropped+=1
        except Exception as error:self.error=str(error)

    def _write(self):
        try:
            import cv2
            self.directory.mkdir(parents=True,exist_ok=True)
            sequence=0
            while True:
                item=self.queue.get()
                if item is None:break
                image,meta=item;started=time.monotonic()
                h,w=image.shape[:2];scale=min(1.,960/w)
                if scale<1:image=cv2.resize(image,None,fx=scale,fy=scale)
                for detection in meta.get('detections',[]):
                    box=detection.get('bbox_xyxy')
                    if box is None:continue
                    x1,y1,x2,y2=[round(x*scale) for x in box]
                    cv2.rectangle(image,(x1,y1),(x2,y2),(0,220,255),2)
                    label=f"{detection['class_name']} {detection['confidence']:.2f}"
                    cv2.putText(image,label,(max(0,x1),max(18,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,220,255),2)
                lines=[f"{meta.get('operation')} | scan {meta.get('scan_index')} | frame {meta['frame']} | {meta['reason']}",
                       f"YOLO {meta['yolo_s']:.3f}s | frame wait {meta['frame_wait_s']:.3f}s | capture age {(meta.get('capture_age_at_result_s') or 0):.3f}s",
                       f"confirmed={meta.get('confirmed',False)} | required={meta.get('required')} / budget={meta.get('budget')}"]
                image=cv2.copyMakeBorder(image,78,0,0,0,cv2.BORDER_CONSTANT,value=(20,20,20))
                for i,line in enumerate(lines):cv2.putText(image,line,(8,21+24*i),cv2.FONT_HERSHEY_SIMPLEX,.47,(245,245,245),1)
                name=f"{meta.get('operation_id',0)}_{meta['frame']}_{time.monotonic_ns()}"
                sequence+=1
                if not self.archive:name=f"frame_{sequence%2}"
                path=self.directory/(name+'.jpg')
                image_temp=self.directory/(name+'.tmp.jpg')
                if not cv2.imwrite(str(image_temp),image,[cv2.IMWRITE_JPEG_QUALITY,85]):raise OSError('JPEG write failed')
                image_temp.replace(path)
                meta.update(image_path=str(path),saved_wall_ns=time.time_ns(),writer_s=time.monotonic()-started,dropped=self.dropped)
                if self.archive:(self.directory/(name+'.json')).write_text(json.dumps(meta,ensure_ascii=False,default=diagnostic_json))
                temp=self.root/'latest.tmp';temp.write_text(json.dumps(meta,ensure_ascii=False,default=diagnostic_json));temp.replace(self.root/'latest.json')
        except Exception as error:self.error=str(error)

    def close(self):
        self.closed=True
        try:self.queue.put_nowait(None)
        except queue.Full:
            self.queue.get_nowait();self.dropped+=1;self.queue.put_nowait(None)
        self.thread.join(timeout=2.)
