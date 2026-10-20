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
