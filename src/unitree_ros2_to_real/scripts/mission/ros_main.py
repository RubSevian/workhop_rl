"""Bounded shutdown of non-actuator Python nodes under ROS launch signals."""
import signal
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
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
