#!/usr/bin/env python3
"""Explicit one-axis operator request. NEVER executed by a launch file."""
import argparse
import json
import time
import yaml
from r3_manual_lib import validate, eligible

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vx', type=float, default=0)
    parser.add_argument('--vy', type=float, default=0)
    parser.add_argument('--wz', type=float, default=0)
    parser.add_argument('--duration', type=float, required=True)
    args = parser.parse_args()
    command = validate((args.vx, args.vy, args.wz), args.duration)
    import rclpy
    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('r3_explicit_manual_command')
    status, received = {}, -1.0
    def observe(msg):
        nonlocal status, received
        try:
            status = yaml.safe_load(msg.data)
            received = time.monotonic()
        except Exception:
            status = {}
    sub = node.create_subscription(String, '/go2/locomotion_status', observe, 10)
    request = node.create_publisher(String, '/go2/commissioning/manual_request', 1)
    deadman = node.create_publisher(String, '/go2/commissioning/manual_deadman', 1)
    nav = node.create_publisher(Bool, '/navigation_active', 1)
    zero = node.create_publisher(TwistStamped, '/cmd_vel', 1)
    client = node.create_client(Trigger, '/go2/commissioning/manual_step')
    def apply(values, duration):
        msg = String(data=json.dumps({'command': values, 'duration_s': duration, 'sent_ns': time.monotonic_ns()}))
        request.publish(msg)
        rclpy.spin_once(node, timeout_sec=.01)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
        if not future.done() or future.result() is None:
            raise RuntimeError('manual service timeout: server deadman will hold')
        if not future.result().success:
            raise RuntimeError(future.result().message)
    active = False
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('commissioning service absent')
        until = time.monotonic()+3
        while time.monotonic()<until and not eligible(status, received, time.monotonic()):
            rclpy.spin_once(node, timeout_sec=.05)
        if not eligible(status, received, time.monotonic()):
            raise RuntimeError('requires fresh eligible R3 RL_ZERO/RL_ACTIVE; read-only/blocked states refused')
        print(f'OPERATOR REQUEST: command={command}, duration={args.duration:.3f}s; bounds=(0.20,0.10,0.10), one axis')
        nav.publish(Bool(data=False))
        apply(command, args.duration)
        active = True
        until = time.monotonic()+args.duration
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.02)
            if not eligible(status, received, time.monotonic()):
                break
            deadman.publish(String(data=json.dumps({'sent_ns': time.monotonic_ns()})))
    finally:
        # Loss/kill also expires the independent server-side deadman/duration.
        if active:
            try:
                apply((0.,0.,0.), .1)
            except Exception:
                pass
        nav.publish(Bool(data=False))
        stop = TwistStamped(); stop.header.stamp = node.get_clock().now().to_msg(); zero.publish(stop)
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
