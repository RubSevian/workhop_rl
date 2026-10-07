"""ROS loopback-only R3 smoke. All messages synthetic; read_only=true mandatory."""
import os
import subprocess
import signal
import time
import struct
from pathlib import Path
import rclpy
import yaml
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from unitree_go.msg import LowState

assert os.environ.get('ROS_DOMAIN_ID') == '223'
root = Path(__file__).resolve().parents[1]
binary = Path(os.environ.get('R3_SMOKE_BINARY', str(root/'install_r1/unitree_legged_real/lib/unitree_legged_real/go2_r3_commissioning')))
config = Path(os.environ.get('R3_SMOKE_CONFIG_PATH', str(root/'repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml')))
profile_config = yaml.safe_load(config.read_text())
command = [str(binary), '--ros-args', '-p', 'read_only:=true', '-p', 'enable_actuator_output:=false',
           '-p', 'config_path:='+str(config),
           '-p', 'model_path:='+str(root/'weights/policy_2.pt')]
trial = os.environ.get('R3_LIEDOWN_TRIAL_SMOKE') == '1'
if trial: command += ['-p','controlled_stop_lie_down_trial:=true']
autonomy = os.environ.get("R3_AUTONOMY_SMOKE") == "1"
remote_test = os.environ.get('R3_REMOTE_TEST_SMOKE') == '1'
command += ['-p', 'remote_test_mode:='+str(remote_test).lower()]
if autonomy: command += ['-p','control_mode:=autonomy','-p','motion_commands_enabled:=true']
rclpy.init()
node = rclpy.create_node('r3_loopback_synthetic_probe')
status = {}
def observe(msg):
    global status
    status = yaml.safe_load(msg.data)
node.create_subscription(String, '/go2/locomotion_status', observe, 10)
pub = node.create_publisher(LowState, '/lowstate', 10)
with (root/'runtime/r3_readonly_smoke_node.log').open('w') as log:
    child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    try:
        until = time.monotonic()+10
        while not status and time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert status.get('read_only') is True and status.get('output_enabled') is False, status
        if 'operation_profile' in status:
            assert status['operation_profile']=='read_only' and status['capabilities']['leg_output'] is False
            assert status['state']=='STANDBY' and 'phase' in status and 'legacy_state' in status
        assert status['model_loaded'] and not status['lowcmd_publisher_present'], status
        if 'controlled_stop_lie_down_trial' in status:
            assert status['controlled_stop_lie_down_trial'] is trial and not status['lie_down_dynamics_validated']
            assert status['custom_leg_output']=='OFF' and not status['lowcmd_lease_present']
            assert status['output_stop_confirmed'] and not status['motor_power_off_confirmed']
        if 'passive_command_sent' in status:
            assert not status['passive_command_sent'] and status['passive_packets_sent']==0
            assert not status['passive_sequence_complete'] and status['last_commanded_leg_mode']==-1
            assert status['passive_packet_limit']==10 and status['passive_timeout_s']==.1
        assert status['remote_test_mode'] is (remote_test and not autonomy), status
        assert status['control_mode']==('autonomy' if autonomy else 'remote_test'),status
        from geometry_msgs.msg import TwistStamped
        nav_pub=node.create_publisher(TwistStamped,'/cmd_vel',1)
        nav=TwistStamped();nav.twist.linear.x=.8;nav.twist.linear.y=-.5;nav.twist.angular.z=.7
        until=time.monotonic()+.5
        while time.monotonic()<until:
            nav.header.stamp=node.get_clock().now().to_msg();nav_pub.publish(nav);rclpy.spin_once(node,timeout_sec=.02)
        if autonomy:
            assert status['navigation_command_fresh'],status
            assert status['navigation_requested_command']==[.2,-.1,.1],status
            until=time.monotonic()+.35
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            assert not status['navigation_command_fresh'],status
            nav.header.stamp.sec-=2
            until=time.monotonic()+.15
            while time.monotonic()<until:
                nav_pub.publish(nav);rclpy.spin_once(node,timeout_sec=.02)
            assert not status['navigation_command_fresh'] and status['navigation_requested_command']==[0,0,0],status
            # Invalid timestamp/value must be rejected without killing the node.
            for invalid in ('negative','nanosecond_overflow','zero','future','nonfinite'):
                nav.header.stamp=node.get_clock().now().to_msg()
                nav.twist.linear.x=.1
                if invalid=='negative':nav.header.stamp.sec=-1
                if invalid=='nanosecond_overflow':nav.header.stamp.nanosec=1000000000
                if invalid=='zero':nav.header.stamp.sec=0;nav.header.stamp.nanosec=0
                if invalid=='future':nav.header.stamp.sec+=2
                if invalid=='nonfinite':nav.twist.linear.x=float('nan')
                until=time.monotonic()+.15
                while time.monotonic()<until:
                    nav_pub.publish(nav);rclpy.spin_once(node,timeout_sec=.02)
                assert child.poll() is None,invalid
                assert not status['navigation_command_fresh'] and status['navigation_requested_command']==[0,0,0],(invalid,status)
            print('PASS malformed nav timestamp/NaN rejected; controller alive')
            print('PASS autonomy source: TwistStamped clamp, freshness expiry, stale header rejection, read-only zero output')
        else:
            assert not status['navigation_command_fresh'] and status['navigation_requested_command']==[0,0,0],status
            print('PASS remote source ignores navigation commands')
        client = node.create_client(SetBool, '/go2/commissioning/enable_output')
        assert client.wait_for_service(timeout_sec=3)
        req = SetBool.Request();req.data=True
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.done() and not future.result().success
        assert 'read_only' in future.result().message
        assert node.count_publishers('/lowcmd') == 0
        if 'operation_profile' in status:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            changer=node.create_client(SetParameters, '/go2_r3_commissioning/set_parameters')
            assert changer.wait_for_service(timeout_sec=3)
            for name, value in [('operation_profile','remote_test'), ('read_only',False), ('motion_commands_enabled',True), ('controlled_stop_lie_down_trial',not trial)]:
                request=SetParameters.Request()
                val=ParameterValue()
                if isinstance(value,bool):val.type=ParameterType.PARAMETER_BOOL;val.bool_value=value
                else:val.type=ParameterType.PARAMETER_STRING;val.string_value=value
                request.parameters=[Parameter(name=name,value=val)]
                changed=changer.call_async(request);rclpy.spin_until_future_complete(node,changed,timeout_sec=3)
                assert changed.done() and not changed.result().results[0].successful
            print('PASS immutable profile/legacy parameters refuse runtime upgrade')
        for name in ('request_stand', 'request_rl'):
            c = node.create_client(Trigger, '/go2/commissioning/'+name)
            assert c.wait_for_service(timeout_sec=3)
            f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=3)
            assert f.done() and not f.result().success
        msg=LowState();msg.imu_state.quaternion=[1.,0.,0.,0.]
        for motor in msg.motor_state:
            motor.q=.2;motor.dq=0.
        raw=bytearray(40);raw[2]=0x22;raw[3]=1
        if remote_test:
            struct.pack_into('<f',raw,4,.5)
            struct.pack_into('<f',raw,8,-1.)
            struct.pack_into('<f',raw,20,1.)
        msg.wireless_remote=list(raw)
        until=time.monotonic()+1.1
        while time.monotonic()<until:
            pub.publish(msg);rclpy.spin_once(node,timeout_sec=.02)
        assert status['remote_mask']==0x122 and status['remote_event']==1, status
        mapping=[3,4,5,0,1,2,9,10,11,6,7,8]
        actor_stand=profile_config['go2_rars01']['default_dof_pos']
        expected_stand=[actor_stand[i] for i in mapping]
        assert len(status['measured_motor_q'])==12 and len(status['stand_error_rad'])==12,status
        assert all(abs(q-.2)<1e-6 for q in status['measured_motor_q']),status
        assert all(abs(a-b)<1e-6 for a,b in zip(status['stand_target_motor_q'],expected_stand)),status
        errors=[abs(.2-q) for q in expected_stand]
        assert all(abs(a-b)<1e-6 for a,b in zip(status['stand_error_rad'],errors)),status
        assert abs(status['stand_error_max_rad']-max(errors))<1e-6,status
        assert status['stand_error_motor_index']==errors.index(max(errors)),status
        assert not status['output_enabled'] and node.count_publishers('/lowcmd')==0, status
        if not profile_config['real_deployment']['r3_commissioning']['gate0_verified']:
            assert 'gate0_not_passed' in status['blockers']
        assert 'transport_ready' in status['blockers']
        if remote_test:
            assert all(abs(a-b)<1e-6 for a,b in zip(status['remote_test_requested_command'],[.2,.1,-.05])), status
            assert status['motion_command']==[0,0,0] and status['sent_packets']==0, status
            print('PASS remote test preview: workshop ly/-rx/-lx stick mapping, bounded combined commands, read-only motion stays zero')
        # Exercise the deployed L1+L2+X exit and the separate emergency chord.
        for mask,event in ((0x422,2),(0x222,3)):
            neutral=bytearray(40);msg.wireless_remote=list(neutral)
            until=time.monotonic()+.2
            while time.monotonic()<until:
                pub.publish(msg);rclpy.spin_once(node,timeout_sec=.02)
            neutral[2]=mask&255;neutral[3]=mask>>8;msg.wireless_remote=list(neutral)
            until=time.monotonic()+1.1
            while time.monotonic()<until:
                pub.publish(msg);rclpy.spin_once(node,timeout_sec=.02)
            assert status['remote_mask']==mask and status['remote_event']==event,status
            assert status['sent_packets']==0 and not status['output_enabled'],status
            assert status['motion_command']==[0,0,0] and node.count_publishers('/lowcmd')==0,status
        log.flush()
        trace=(root/'runtime/r3_readonly_smoke_node.log').read_text()
        assert 'R3 transition' in trace and 'fixed_kp=' in trace
        assert 'hold_capture_tolerance_rad=' not in trace
        print('PASS mapped stand-error diagnostics and persisted transition/gain logging')
        print('PASS L1+L2+X event=2; L1+L2+B event=3; no leg output')
        print('PASS isolated ROS domain223/loopback: status, read-only enable/stand/RL refusal, synthetic SDK remote event, zero LowCmd publishers')
    finally:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown()
