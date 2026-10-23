#!/usr/bin/env python3
"""Read-only terminal transitions; every received status is retained in JSONL."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import yaml
from mission.operator_events import OperatorEvents,render_event
from mission.status_codec import decode_status

TOPICS={'r3':'/go2/locomotion_status','mission':'/go2/mission/status','nav':'/go2/navigation/status',
        'perception':'/rars01/mission/perception_health','arm':'/rars01/commissioning/state'}


def json_safe(value):
    if isinstance(value,dict):return {key:json_safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [json_safe(item) for item in value]
    return str(value) if isinstance(value,float) and not math.isfinite(value) else value


class MonitorOutput:
    """Testable output boundary: journal all snapshots, print only transitions."""
    def __init__(self,journal,terminal=None,format='text'):
        self.events=OperatorEvents();self.journal=journal
        self.terminal=terminal if terminal is not None else sys.stdout;self.format=format

    def receive(self,name,raw,monotonic_ns,ros_ns):
        record=dict(record_type='snapshot',subsystem=name,monotonic_ns=monotonic_ns,ros_ns=ros_ns,raw=raw)
        try:
            data=decode_status(raw) if name in ('r3','arm') else json.loads(raw)
            event=self.events.observe(name,data,monotonic_ns,ros_ns)
        except (ValueError,KeyError,TypeError,yaml.YAMLError) as error:
            # Full parser diagnostics stay in the journal. Offsets, raw text and
            # line numbers of another malformed frame are not new system states.
            record['decode_error']=str(error)
            detail=getattr(error,'msg',None) or getattr(error,'problem',None) or str(error)
            event=self.events.observe(name,{'decode_error':'STATUS_DECODE_ERROR:'+type(error).__name__+':'+detail},monotonic_ns,ros_ns)
        event=json_safe(event)
        record['event']=event
        # Raw wire text preserves nonfinite YAML measurements and every numeric
        # detail without silently replacing them or putting NaN in JSON output.
        self.journal.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
        if event is not None:
            print(json.dumps(event,ensure_ascii=False,allow_nan=False) if self.format=='json' else render_event(event),file=self.terminal,flush=True)
        return event


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jsonl',type=Path,help='All received snapshots and optional transition events')
    parser.add_argument('--format',choices=('text','json'),default='text')
    args,ros_args=parser.parse_known_args()
    path=args.jsonl or Path(os.environ.get('ROS_LOG_DIR',str(Path.cwd()/'runtime/architecture_monitor')))/('snapshots_'+str(time.time_ns())+'_'+str(os.getpid())+'.jsonl')
    path.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=ros_args);node=Node('go2_rars01_architecture_monitor')
    try:
        with path.open('a',encoding='utf-8',buffering=1024*1024) as journal:
            output=MonitorOutput(journal,format=args.format)
            # Best-effort diagnostic subscriptions cannot impose reliable-reader
            # backpressure on control status publishers when disk IO is slow.
            qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.BEST_EFFORT)
            subscriptions=[node.create_subscription(String,topic,lambda msg,n=name:output.receive(
                n,msg.data,time.monotonic_ns(),node.get_clock().now().nanoseconds),qos) for name,topic in TOPICS.items()]
            flush_timer=node.create_timer(1.,journal.flush)
            try:rclpy.spin(node)
            except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
