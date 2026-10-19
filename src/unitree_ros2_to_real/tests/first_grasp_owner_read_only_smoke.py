#!/usr/bin/env python3
"""Actual owner services/lease in READ_ONLY. Never connects serial or enables."""
import os
from pathlib import Path
import subprocess
import tempfile
import time
import sys
import yaml
assert os.environ.get('ROS_DOMAIN_ID')=='228' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
ROOT=Path(__file__).resolve().parents[5]
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

def main():
    binary=Path(sys.argv[1])
    with tempfile.TemporaryDirectory(prefix='grasp-owner-read-only-') as temporary:
        argv=[str(binary),'--ros-args','-p','read_only:=true','-p','connect_serial:=false',
              '-p','sdk_config_path:='+str(ROOT/'repos/rars01_graspnet/config/default.yaml'),
              '-p','config_path:='+str(ROOT/'repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml'),
              '-p','mission_config_path:='+str(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml'),
              '-p','workspace:='+str(ROOT),'-p','lock_directory:='+temporary+'/locks','-p','journal_path:='+temporary+'/journal']
        with open(Path(temporary)/'owner.log','w+') as log:
            owner=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT)
            rclpy.init();node=rclpy.create_node('read_only_owner_probe');states=[]
            node.create_subscription(String,'/rars01/commissioning/state',lambda msg:states.append(yaml.safe_load(msg.data)),1)
            client=node.create_client(Trigger,'/rars01/mission/cancel_manipulation')
            try:
                deadline=time.monotonic()+5
                while time.monotonic()<deadline and (not states or not client.service_is_ready()):rclpy.spin_once(node,timeout_sec=.05)
                assert states and states[-1]['owner_state']=='READ_ONLY'
                assert not states[-1]['connected'] and states[-1]['successful_sends']==0 and not states[-1]['enable_attempted']
                assert not states[-1]['mission_capable']
                future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                assert future.done() and future.result().success
                duplicate=subprocess.run(argv,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=3)
                assert duplicate.returncode!=0 and b'Serial owner lease busy' in duplicate.stdout,duplicate.stdout
                assert node.count_publishers('/lowcmd')==0
                print('PASS actual READ_ONLY owner: cancel service responds; duplicate lease rejected; serial=false enable=false sends=0 LowCmd=0')
            finally:
                node.destroy_node();rclpy.try_shutdown()
                # This is solely our read_only/connect=false test process.
                owner.terminate();owner.wait(timeout=3)
if __name__=='__main__':main()
