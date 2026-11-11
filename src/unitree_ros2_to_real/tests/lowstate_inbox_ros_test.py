"""Opt-in read-only integration: synthetic LowState on loopback, no actuators."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import json
import yaml
import rclpy
from std_msgs.msg import String
from unitree_go.msg import LowState
from rclpy.qos import qos_profile_sensor_data


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '219' or 'name="lo"' not in os.environ.get('CYCLONEDDS_URI', ''):
        raise RuntimeError('Requires isolated domain 219 and explicit loopback interface')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('lowstate_inbox_readonly_test')
    values=[]
    sub=node.create_subscription(String,'/offline/r3_status',lambda msg:values.append(yaml.safe_load(msg.data)),10)
    pub=node.create_publisher(LowState,'/offline/lowstate',qos_profile_sensor_data)
    command=[str(args.binary.resolve()),'--ros-args','-p','operation_profile:=read_only',
             '-p','read_only:=true','-p','remote_auto_sequence:=false',
             '-p','motion_diagnostics_enabled:=false','-p','config_path:='+str(args.config.resolve()),
             '-p','model_path:='+str(args.model.resolve()),'-r','/lowstate:=/offline/lowstate',
             '-r','/go2/locomotion_status:=/offline/r3_status']
    def spin_until(predicate,timeout):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            assert child.poll() is None, 'Controller exited; see controller.log'
            rclpy.spin_once(node,timeout_sec=.02)
            assert node.count_publishers('/lowcmd')==0
            assert node.count_publishers('/api/sport/request')==0
        assert predicate(), 'Timed out waiting for read-only input/status'
    with (args.output/'controller.log').open('w') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        try:
            spin_until(lambda:bool(values) and pub.get_subscription_count()==1,45)
            msg=LowState();msg.imu_state.quaternion[0]=1.
            msg.wireless_remote[0]=0xFE;msg.wireless_remote[1]=0xEF
            for motor in msg.motor_state:motor.q=.1
            start=time.monotonic();sent=0
            while time.monotonic()-start<2:
                msg.tick+=2;pub.publish(msg);sent+=1
                rclpy.spin_once(node,timeout_sec=.001);time.sleep(.001)
            spin_until(lambda:any(0<=v.get('lowstate_age_s',-1)<.06 for v in values),2)
            receipt_status=len(values)
            spin_until(lambda:len(values)>receipt_status and values[-1].get('lowstate_age_s',-1)>.15,2)
            assert all(v['read_only'] and not v['output_enabled'] and v['sent_packets']==0 for v in values)
            assert all(not v.get('fault_latched',False) for v in values)
            result=dict(passed=True,synthetic_frames=sent,statuses=len(values),
                        stopped_source_age_s=values[-1]['lowstate_age_s'],actuator_publishers=0,
                        note='Functional integration only; no real-robot latency or throughput claim')
            (args.output/'result.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(result))
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:child.kill();child.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
