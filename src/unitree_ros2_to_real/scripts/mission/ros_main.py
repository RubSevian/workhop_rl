"""Bounded shutdown of non-actuator Python nodes under ROS launch signals."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

def spin(node_type):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    node=None
    try:
        node=node_type();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
