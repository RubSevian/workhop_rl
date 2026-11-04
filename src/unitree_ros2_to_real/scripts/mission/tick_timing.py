"""Local callback timing only; never used for authority or freshness decisions."""
import time


class TickTiming:
    def __init__(self):
        self.started_ns = self.wall = time.monotonic_ns()
        self.started_cpu_ns = self.cpu = time.thread_time_ns()
        self.stages = {}
        self.health_observed_ns = None

    def mark(self, name):
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        self.stages[name] = dict(wall_s=(wall-self.wall)/1e9,
                                 thread_cpu_s=(cpu-self.cpu)/1e9)
        self.wall, self.cpu = wall, cpu

    def finish(self, node):
        self.mark('status_publish')
        previous = getattr(node, '_previous_tick_timing', None)
        row = dict(started_ns=self.started_ns, finished_ns=self.wall,
                   finished_cpu_ns=self.cpu, stages=self.stages,
                   health_observed_ns=self.health_observed_ns,
                   wall_s=(self.wall-self.started_ns)/1e9,
                   thread_cpu_s=(self.cpu-self.started_cpu_ns)/1e9)
        gap = 0 if previous is None else (self.started_ns-previous['started_ns'])/1e9
        row['between_ticks_wall_s'] = None if previous is None else (self.started_ns-previous['finished_ns'])/1e9
        row['between_ticks_thread_cpu_s'] = None if previous is None else (self.started_cpu_ns-previous['finished_cpu_ns'])/1e9
        journal = getattr(node, 'short_journal', None)
        if journal is not None and max(gap, row['wall_s']) >= .1:
            journal.put('mission_tick_timing', tick=row, previous_tick=previous,
                        callbacks=node.input_diagnostics.snapshot(self.wall/1e9))
        node._previous_tick_timing = row
