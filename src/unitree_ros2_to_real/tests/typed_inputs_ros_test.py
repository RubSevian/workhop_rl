"""Opt-in native R3 typed-input admission test on loopback, without actuators."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy,yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rars01_state_interfaces.msg import ArmFastState
from go2_navigation_interfaces.msg import NavigationReadiness

def main():
 assert os.environ.get('ROS_DOMAIN_ID')=='219' and 'name="lo"' in os.environ.get('CYCLONEDDS_URI','')
 p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--config',required=True);p.add_argument('--model',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 rclpy.init();n=rclpy.create_node('typed_r3_input_test');states=[]
 n.create_subscription(String,'/go2/locomotion_status',lambda m:states.append(yaml.safe_load(m.data)),10)
 arm=n.create_publisher(ArmFastState,'/rars01/commissioning/fast_state',10);nav=n.create_publisher(NavigationReadiness,'/go2/mission/readiness_typed',10)
 def cancel(req,res):res.success=True;return res
 services=[n.create_service(Trigger,t,cancel) for t in ('/go2/mission/cancel_navigation','/rars01/mission/cancel_manipulation')]
 msg=ArmFastState();msg.owner_session_id='test-arm';msg.owner_state='HOLD_HOME';msg.home_joint_count=7
 msg.connected=msg.enabled_local=msg.feedback_ready=msg.target_valid=msg.accepted_target_valid=msg.arm_home_ready=True
 msg.motor_ids=list(range(1,7));msg.valid=[True]*6;msg.motor_id=list(range(1,8));msg.motor_status=[1]*7;msg.valid7=[True]*7
 envelope=NavigationReadiness(session_id='test-nav',navigation_ready=True,perception_ready=True,command_enabled=True,command=[.1,0.,0.])
 log=(a.output/'controller.log').open('w');child=subprocess.Popen([a.binary,'--ros-args','-p','operation_profile:=read_only','-p','read_only:=true','-p','remote_auto_sequence:=false','-p','motion_diagnostics_enabled:=false','-p','arm_state_transport:=typed','-p','navigation_state_transport:=typed','-p','config_path:='+a.config,'-p','model_path:='+a.model],stdout=log,stderr=subprocess.STDOUT)
 def wait(predicate,seconds=4,publish_nav=True):
  first=len(states);end=time.monotonic()+seconds
  while time.monotonic()<end:
   assert child.poll() is None,'controller exited'
   stamp=time.monotonic_ns();msg.observed_ns=msg.frame_received_ns=msg.accepted_ns=stamp;arm.publish(msg)
   if publish_nav:envelope.observed_ns=stamp;nav.publish(envelope)
   rclpy.spin_once(n,timeout_sec=.01)
   assert n.count_publishers('/lowcmd')==0 and n.count_publishers('/api/sport/request')==0
   if len(states)>first and predicate(states[-1]):return
  raise AssertionError('status condition timeout')
 try:
  wait(lambda s:s['arm_control_ready'] and s['navigation_ready'],45)
  msg.valid7=[True]*6+[False];wait(lambda s:not s['arm_control_ready'] and bool(s['arm_error']))
  msg.valid7=[True]*7;wait(lambda s:s['arm_control_ready'])
  msg.schema_version=99;wait(lambda s:not s['arm_control_ready'] and 'SCHEMA' in s['arm_error'])
  msg.schema_version=2;wait(lambda s:s['arm_control_ready'])
  wait(lambda s:not s['navigation_ready'],publish_nav=False)
  wait(lambda s:s['navigation_ready'])
  envelope.schema_version=99;wait(lambda s:not s['navigation_ready'])
  assert all(s['read_only'] and not s['output_enabled'] and s['sent_packets']==0 for s in states)
 finally:
  if child.poll() is None:child.send_signal(signal.SIGINT)
  try:child.wait(timeout=12)
  except subprocess.TimeoutExpired:child.kill();child.wait()
  log.close();n.destroy_node();rclpy.try_shutdown()
 assert child.returncode==0,child.returncode
 (a.output/'result.json').write_text(json.dumps(dict(passed=True,statuses=len(states),shutdown_returncode=child.returncode,actuator_publishers=0,checks=['healthy typed inputs','invalid seventh motor','invalid arm schema','NAV expiry','NAV recovery','invalid NAV schema','clean SIGINT']),indent=2))
if __name__=='__main__':main()
