#!/usr/bin/env python3
"""Readiness-gated manual target probe; observes but never commands the legs."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--manual-result', required=True)
    parser.add_argument('--duration', type=float, default=180)
    parser.add_argument('--target-arm', nargs=3, type=float, default=[0.3, 0.0, 0.3])
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('payload_runtime_probe')
    state = {'ready': False, 'pose': None, 'sent': False, 'events': [], 'status': []}
    def ready(msg): state['ready'] = msg.data
    def pose(msg): state['pose'] = msg
    def status(msg):
        if not state['status'] or msg.data != state['status'][-1]['data']:
            state['status'].append({'wall_s': time.monotonic()-start, 'data': msg.data})
    def payload(msg):
        value = json.loads(msg.data)
        if not state['events'] or value['payload_attached'] != state['events'][-1]['payload_attached']:
            state['events'].append(value)
    subscriptions = [
        node.create_subscription(Bool, '/stage4d/navigation_ready', ready, 10),
        node.create_subscription(PoseStamped, '/sim/rars_base_pose', pose, 10),
        node.create_subscription(String, '/stage4d/manual_manip_status', status, 10),
        node.create_subscription(String, '/stage4d/payload_status', payload,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    publisher = node.create_publisher(PoseStamped, '/stage4d/manual_manip_target', 10)
    start = time.monotonic()
    wall_start = time.time()
    result_path = Path(args.manual_result)
    state['complete'] = False
    try:
        while time.monotonic()-start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not state['sent'] and state['ready'] and state['pose'] and time.monotonic()-start > 12:
                source = state['pose']
                q = source.pose.orientation
                # Arm-frame point, transformed using the authoritative current mount pose.
                x,y,z,w = q.x,q.y,q.z,q.w
                rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                p=source.pose.position
                point=np.array([p.x,p.y,p.z])+rotation@np.array(args.target_arm)
                msg=PoseStamped(); msg.header=source.header; msg.header.frame_id='map'
                msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=point.tolist()
                msg.pose.orientation.w=1.0
                publisher.publish(msg)
                state['sent']=True; state['target_world']=point.tolist()
            if state['sent'] and result_path.exists() and result_path.stat().st_mtime >= wall_start:
                result=json.loads(result_path.read_text())
                if result.get('target_world') and np.allclose(result['target_world'], state['target_world']):
                    state['complete']=bool(result.get('complete'))
                    state['manual_result']=str(result_path)
                    break
    finally:
        state.pop('pose')
        Path(args.output).write_text(json.dumps(state, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    return 0 if state['complete'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
