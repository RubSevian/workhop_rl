"""Image diagnostics without camera, ROS or actuators."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.detection_debug import DetectionDebug

class DebugTests(unittest.TestCase):
    def test_writes_annotated_image_and_metadata(self):
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            writer=DetectionDebug(directory)
            writer.submit(np.zeros((80,100,3),dtype=np.uint8),dict(frame=1,operation='detect',operation_id=7,scan_index=0,
                reason='VALID_TARGET',yolo_s=.2,frame_wait_s=.1,capture_age_at_result_s=.4,
                detections=[dict(class_name='cup',confidence=.9,bbox_xyxy=[10,10,30,40])]))
            writer.close();self.assertEqual(writer.error,'');self.assertFalse(writer.thread.is_alive())
            metadata=json.loads((Path(directory)/'latest.json').read_text())
            image=cv2.imread(metadata['image_path'])
            self.assertEqual(image.shape,(158,100,3));self.assertTrue(np.any(image))
            self.assertEqual(metadata['operation_id'],7)
    def test_io_failure_is_diagnostic_only(self):
        with tempfile.TemporaryDirectory() as directory:
            blocked=Path(directory)/'file';blocked.write_text('not a directory')
            writer=DetectionDebug(blocked);writer.close()
            self.assertTrue(writer.error)

if __name__=='__main__':unittest.main()
