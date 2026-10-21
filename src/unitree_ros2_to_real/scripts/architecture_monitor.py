#!/usr/bin/env python3
"""Read-only terminal events for R3, mission, NAV, perception and arm; no commands."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mission.operator_events import OperatorEvents,render_event
from mission.status_codec import decode_status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jsonl',type=Path)
    parser.add_argument('--format',choices=('text','json'),default='text')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args);node=Node('go2_rars01_architecture_monitor')
    events=OperatorEvents();output=args.jsonl.open('a',encoding='utf-8') if args.jsonl else None
    topics={'r3':'/go2/locomotion_status','mission':'/go2/mission/status','nav':'/go2/navigation/status',
            'perception':'/rars01/mission/perception_health','arm':'/rars01/commissioning/state'}
    def receive(name,msg):
        try:
            data=decode_status(msg.data) if name in ('r3','arm') else json.loads(msg.data)
            event=events.observe(name,data,time.monotonic_ns(),node.get_clock().now().nanoseconds)
            if event is None:return
            line=json.dumps(event,ensure_ascii=False,allow_nan=False)
            print(line if args.format=='json' else render_event(event),flush=True)
            if output:output.write(line+'\n');output.flush()
        except (ValueError,KeyError,TypeError) as error:
            # Decoder errors are deduplicated by the same state-change policy.
            event=events.observe(name,{'decode_error':str(error)},time.monotonic_ns(),node.get_clock().now().nanoseconds)
            if event:
                line=json.dumps(event,ensure_ascii=False);print(line if args.format=='json' else render_event(event),flush=True)
                if output:output.write(line+'\n');output.flush()
    for name,topic in topics.items():node.create_subscription(String,topic,lambda msg,n=name:receive(n,msg),1)
    try:rclpy.spin(node)
    finally:
        if output:output.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
