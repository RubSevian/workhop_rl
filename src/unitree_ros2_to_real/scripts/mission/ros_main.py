"""Bounded shutdown of non-actuator Python nodes under ROS launch signals."""
import signal
import threading
import os
import sys
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

def spin(node_type):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping=False
    def interrupt(*unused):
        nonlocal stopping
        stopping=True
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    node=None
    try:
        node=node_type();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        # Pybind can translate an interrupt inside take_message to RuntimeError.
        # Suppress only after an explicit shutdown signal, never a runtime fault.
        if not stopping:raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        outcome=[]
        def cleanup():
            try:
                if node is not None:node.destroy_node()
                rclpy.try_shutdown();outcome.append(True)
            except Exception:outcome.append(False)
        worker=threading.Thread(target=cleanup,daemon=True,name='passive-dds-cleanup')
        worker.start();worker.join(getattr(node,'shutdown_budget_s',5.))
        if not outcome or not outcome[0] or worker.is_alive():
            print('Passive ROS cleanup exceeded budget or failed; exiting this process only',file=sys.stderr,flush=True)
            os._exit(70)
        if stopping:
            # Avoid middleware/global-object destructors running a second time.
            sys.stdout.flush();sys.stderr.flush();os._exit(0)
