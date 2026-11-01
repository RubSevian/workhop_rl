#!/usr/bin/env python3
"""Actual Perception guard-condition delivery, physical=false, isolated DDS only."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import yaml

if os.environ.get('ROS_DOMAIN_ID')!='228' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('Requires localhost ROS domain 228')
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from grasp_perception import Perception


def main():
    raw=yaml.safe_load((ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())
    raw['go2_mission']['execution'].update(short_journal_enabled=False,diagnostic_recording_enabled=False)
    with tempfile.TemporaryDirectory() as directory:
        cfg=Path(directory)/'config.yaml';cfg.write_text(yaml.safe_dump(raw))
        rclpy.init(args=['--ros-args','-p','mission_config_path:='+str(cfg),
                        '-p','workspace:='+str(ROOT),'-p','physical:=false'])
        node=Perception();observer=rclpy.create_node('delivery_observer')
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(observer)
        received=[]
        observer.create_subscription(String,'/rars01/mission/computed',
            lambda m:received.append((time.monotonic_ns(),json.loads(m.data))),10)
        def spin_until(predicate,seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end and not predicate():executor.spin_once(timeout_sec=.01)
            assert predicate()
        try:
            spin_until(lambda:node.results.get_subscription_count()>0,3.)
            for index in range(3):
                stamp=time.monotonic_ns()
                result=dict(id=index+1,session_id='smoke',observed_ns=stamp,ok=True,found=False)
                node.compute_cancel.begin(dict(result,operation='detect'))
                encoded=json.dumps(result);result['queued_ns']=time.monotonic_ns()
                node.busy=True;node.completed.put((result,encoded));node.result_guard.trigger()
                spin_until(lambda:len(received)>index,1.)
                at,out=received[-1]
                assert out['observed_ns']==stamp and 0<=(at-stamp)/1e9<.35
                assert out['publish_started_ns']>=out['queued_ns']>=stamp
            node.busy=True;node.completed.put((result,encoded))
            node.compute_cancel.observe(dict(session_id='smoke',observed_ns=time.monotonic_ns(),state='CANCELLED'))
            node.result_guard.trigger();spin_until(lambda:not node.busy,1.)
            end=time.monotonic()+.1
            while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
            assert len(received)==3
            assert node.backend is None and node.worker_thread is None
            print('PASS actual Perception guard delivery and cancellation; no camera/models/actuators')
        finally:
            executor.shutdown();observer.destroy_node();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
