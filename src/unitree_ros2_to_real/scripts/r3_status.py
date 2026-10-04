#!/usr/bin/env python3
"""Read-only bounded structured status display."""
import argparse
import time
import yaml

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--timeout', type=float, default=10.)
    args=parser.parse_args()
    import rclpy
    from std_msgs.msg import String
    rclpy.init(); node=rclpy.create_node('r3_read_only_status')
    got=False; last=-1.
    def show(msg):
        nonlocal got, last
        now=time.monotonic()
        if got and now-last<1:return
        print(yaml.safe_dump(yaml.safe_load(msg.data), sort_keys=False), flush=True)
        got=True;last=now
    node.create_subscription(String,'/go2/locomotion_status',show,10)
    deadline=time.monotonic()+args.timeout
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if got and args.once:break
        if not got:raise RuntimeError('no commissioning status observed; no gate passed')
    finally:
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
