"""Linux per-thread CFS niceness; never change FIFO policy or other threads."""
import os
import threading

def inference_thread_nice(level):
    if type(level) is not int or not 0<=level<=19:raise ValueError('Invalid inference niceness')
    tid=threading.get_native_id()
    current=os.getpriority(os.PRIO_PROCESS,tid)
    # Only lower our own scheduling weight; never require a privileged raise.
    if level>current:os.setpriority(os.PRIO_PROCESS,tid,level)
    return os.getpriority(os.PRIO_PROCESS,tid)


def inference_thread_status():
    """Describe the actual planning thread; never change ROS/FIFO scheduling."""
    tid=threading.get_native_id()
    return dict(pid=os.getpid(),tid=tid,name=threading.current_thread().name,
                nice=os.getpriority(os.PRIO_PROCESS,tid),
                cpu_affinity=sorted(os.sched_getaffinity(tid)),
                scheduler=os.sched_getscheduler(tid),
                numeric_thread_limits={key:os.environ.get(key) for key in
                    ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')})
