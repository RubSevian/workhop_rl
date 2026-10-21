#!/usr/bin/env python3
"""Installed Mission/Perception launch, physical=false; no models/camera/SDK IO."""
import json,os,signal,subprocess,tempfile,time
from pathlib import Path
CANONICAL='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
if (os.environ.get('ROS_DOMAIN_ID')!='228' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
        or os.environ.get('CYCLONEDDS_URI')!=CANONICAL or os.environ.get('GO2_NETWORK_INTERFACE') or os.environ.get('ROS_STATIC_PEERS')):
    raise RuntimeError('Read-only smoke requires isolated domain228/canonical loopback/no SDK or peers')
import rclpy
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[5]
rclpy.init();node=rclpy.create_node('mission_read_only_launch_probe');statuses=[];health=[];requests=[]
node.create_subscription(String,'/go2/mission/status',lambda m:statuses.append(json.loads(m.data)),1)
node.create_subscription(String,'/rars01/mission/perception_health',lambda m:health.append(json.loads(m.data)),1)
for topic in ('/rars01/mission/compute','/rars01/mission/trajectory'):
    node.create_subscription(String,topic,lambda m:requests.append(m.data),1)
process=None
with tempfile.TemporaryDirectory(prefix='go2-mission-readonly-') as folder:
    with open(Path(folder)/'launch.log','w+') as log:
        try:
            process=subprocess.Popen(['ros2','launch','unitree_legged_real','go2_rars01_first_grasp.launch.py',
                'physical:=false','test_mode:=full','workspace:='+str(ROOT),
                'mission_config_path:='+str(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml')],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                assert process.poll() is None,'read-only launch exited'
                rclpy.spin_once(node,timeout_sec=.05)
                if statuses and health:break
            assert statuses and health,'read-only component status timeout'
            assert statuses[-1]['state']=='BLOCKED' and 'read_only_launch' in statuses[-1]['blockers']
            assert not health[-1]['model_ready'] and not health[-1]['ready']
            assert not requests,'read-only launch sent a planning/motion request'
            assert node.count_publishers('/lowcmd')==0
            assert node.count_publishers('/api/sport/request')==0
            print('PASS installed full-mode Mission/Perception inspection: physical=false, no model/camera startup, no task/trajectory request, no LowCmd/Sport publisher',flush=True)
        except Exception:
            log.flush();log.seek(0);print(log.read());raise
        finally:
            if process and process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            if process:
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
            node.destroy_node();rclpy.try_shutdown()
