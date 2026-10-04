"""ROS loopback-only R3 smoke. All messages synthetic; read_only=true mandatory."""
import os
import subprocess
import signal
import time
from pathlib import Path
import rclpy
import yaml
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from unitree_go.msg import LowState

assert os.environ.get('ROS_DOMAIN_ID') == '223'
root = Path(__file__).resolve().parents[1]
binary = root/'install_r1/unitree_legged_real/lib/unitree_legged_real/go2_r3_commissioning'
command = [str(binary), '--ros-args', '-p', 'read_only:=true', '-p', 'enable_actuator_output:=false',
           '-p', 'config_path:='+str(root/'repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml'),
           '-p', 'model_path:='+str(root/'weights/policy_2.pt')]
rclpy.init()
node = rclpy.create_node('r3_loopback_synthetic_probe')
status = {}
def observe(msg):
    global status
    status = yaml.safe_load(msg.data)
node.create_subscription(String, '/go2/locomotion_status', observe, 10)
pub = node.create_publisher(LowState, '/lowstate', 10)
with (root/'r3_readonly_smoke_node.log').open('w') as log:
    child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    try:
        until = time.monotonic()+10
        while not status and time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert status.get('read_only') is True and status.get('output_enabled') is False, status
        assert status['model_loaded'] and not status['lowcmd_publisher_present'], status
        client = node.create_client(SetBool, '/go2/commissioning/enable_output')
        assert client.wait_for_service(timeout_sec=3)
        req = SetBool.Request();req.data=True
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.done() and not future.result().success
        assert 'read_only' in future.result().message
        assert node.count_publishers('/lowcmd') == 0
        for name in ('request_stand', 'request_rl'):
            c = node.create_client(Trigger, '/go2/commissioning/'+name)
            assert c.wait_for_service(timeout_sec=3)
            f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=3)
            assert f.done() and not f.result().success
        msg=LowState();msg.imu_state.quaternion=[1.,0.,0.,0.]
        for motor in msg.motor_state:
            motor.q=.2;motor.dq=0.
        raw=[0]*40;raw[2]=0x22;raw[3]=1;msg.wireless_remote=raw
        until=time.monotonic()+1.1
        while time.monotonic()<until:
            pub.publish(msg);rclpy.spin_once(node,timeout_sec=.02)
        assert status['remote_mask']==0x122 and status['remote_event']==1, status
        assert not status['output_enabled'] and node.count_publishers('/lowcmd')==0, status
        assert 'gate0_not_passed' in status['blockers']
        print('PASS isolated ROS domain223/loopback: status, read-only enable/stand/RL refusal, synthetic SDK remote event, zero LowCmd publishers')
    finally:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown()
