"""Operator output preserves failure meaning and never implies physical motion."""
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission.terminal_status import terminal_lines,reason_text
from mission.short_journal import ShortJournal
from mission.planning_metrics import planning_stage

class Tests(unittest.TestCase):
    def test_timeout_and_grasp_planning_explained(self):
        text=' '.join(terminal_lines(dict(event='mission_state',state='BLOCKED',reason='PHASE_TIMEOUT',previous_phase_duration_s=180.)))
        self.assertIn('ОСТАНОВКА',text);self.assertIn('180.0 с',text);self.assertIn('PHASE_TIMEOUT',text)
        text=' '.join(terminal_lines(dict(event='mission_state',state='PLAN_GRASP',pending={})))
        self.assertIn('движение к объекту ещё не началось',text)
        self.assertIn('Head_upper ↔ link_3_m3',reason_text("{'reason':'COLLISION:Head_upper:link_3_m3','sample_index':772}"))

    def test_stage_progress_reaches_terminal_and_small_journal(self):
        with tempfile.TemporaryDirectory() as directory:
            terminal=io.StringIO();journal=ShortJournal(directory,'perception','test',terminal=terminal)
            request=dict(id=1,session_id='mission',operation='grasp_plan');metrics={}
            with self.assertRaisesRegex(ValueError,'COLLISION'):
                with planning_stage(metrics,'collision_home',request=request,progress=journal):
                    raise ValueError('COLLISION:Head_upper:link_3_m3')
            journal.close()
            rows=[json.loads(line) for line in journal.path.read_text().splitlines()]
            self.assertEqual([r['status'] for r in rows],['running','failed'])
            self.assertIn('возврат HOME',terminal.getvalue())
            self.assertIn('Обнаружена коллизия',terminal.getvalue())
            self.assertNotIn('sample_index',terminal.getvalue())
            self.assertFalse(journal.error)

    def test_terminal_error_does_not_stop_file_writer_or_calculation(self):
        class Broken:
            def write(self,value):raise BrokenPipeError('synthetic terminal closed')
        with tempfile.TemporaryDirectory() as directory:
            journal=ShortJournal(directory,'test','session',terminal=Broken())
            for _ in range(2):journal.put('planning_stage',stage='graspnet',status='running')
            journal.close()
            self.assertEqual(journal.written,2);self.assertTrue(journal.terminal_error);self.assertFalse(journal.error)
        class FailedProgress:
            error=''
            def stage(self,*args):raise OSError('synthetic logging failure')
        metrics={};progress=FailedProgress()
        with planning_stage(metrics,'graspnet',request=dict(id=1),progress=progress):pass
        self.assertEqual(metrics['planning_stages'][0]['status'],'completed')
        self.assertIn('synthetic',progress.error)

    def test_compute_reports_detection_and_counts_without_raw_payload(self):
        lines=terminal_lines(dict(event='compute_result',operation='detect',ok=True,found=True,summary=dict(elapsed_s=3.)))
        self.assertIn('объект найден',lines[0]);self.assertIn('3.0 с',lines[0])
        lines=terminal_lines(dict(event='compute_result',operation='grasp_plan',ok=False,reason='IK_FAIL',summary=dict(elapsed_s=2.,filter_counts=dict(decoded=31,after_depth=6,after_nms=6,ik_feasible=3))))
        self.assertIn('IK не нашёл',lines[0]);self.assertIn('вариантов после IK: 3',lines[1])

if __name__=='__main__':unittest.main()
