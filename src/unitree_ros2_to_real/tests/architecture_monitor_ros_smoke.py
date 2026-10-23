#!/usr/bin/env python3
"""Installed monitor, synthetic status publishers on isolated loopback; no SDK."""
import copy,json,os,signal,subprocess,tempfile,time
from pathlib import Path
CANONICAL='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
if (os.environ.get('ROS_DOMAIN_ID')!='228' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
        or os.environ.get('CYCLONEDDS_URI')!=CANONICAL or os.environ.get('GO2_NETWORK_INTERFACE') or os.environ.get('ROS_STATIC_PEERS')):
    raise RuntimeError('Synthetic monitor smoke requires isolated domain228/canonical loopback/no SDK or peers')
import rclpy
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('architecture_monitor_synthetic_status_probe')
TOPICS={'r3':'/go2/locomotion_status','mission':'/go2/mission/status','nav':'/go2/navigation/status',
        'perception':'/rars01/mission/perception_health','arm':'/rars01/commissioning/state'}
pubs={name:node.create_publisher(String,topic,1) for name,topic in TOPICS.items()}
base={'r3':dict(state='ACTIVE',phase='RL_ZERO',operation_profile='full_mission',fault_latched=False,fault='',phase_warnings=[],blockers=[]),
      'mission':dict(state='WAIT_GOAL',test_mode='full',pending=None,availability=dict(navigation=True,manipulation=True)),
      'nav':dict(navigation_ready=True,input_mode='stack',block_reason='',tf_pair=dict(reason='OK')),
      'perception':dict(ready=True,camera_mode='standby',model_ready=True,busy=False),
      'arm':dict(owner_state='HOLD_HOME',feedback_ready=True,motors_enabled=True,mission_state='IDLE',mission_id=0,last_error='')}
process=None;counter=0
with tempfile.TemporaryDirectory(prefix='architecture-monitor-smoke-') as folder:
    journal=Path(folder)/'snapshots.jsonl';console=Path(folder)/'terminal.log'
    def events():
        return [json.loads(line) for line in console.read_text().splitlines() if line.startswith('{')]
    def publish(status):
        global counter
        counter+=1
        for name,data in status.items():
            d=copy.deepcopy(data);d.update(observed_ns=time.monotonic_ns(),frame_count=counter,feedback_age_s=counter/1e6)
            if name=='nav':d['tf_pair'].update(age_s=counter/1e6,frames=counter)
            pubs[name].publish(String(data=json.dumps(d)))
        rclpy.spin_once(node,timeout_sec=.02)
    def settle(status,expected):
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            publish(status)
            if len(events())>=expected:return
            time.sleep(.03)
        raise AssertionError(('event timeout',expected,events()))
    with console.open('w') as log:
        try:
            exe=Path(get_package_prefix('unitree_legged_real'))/'lib/unitree_legged_real/architecture_monitor.py'
            process=subprocess.Popen([str(exe),'--format','json','--jsonl',str(journal)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+8
            while not all(node.count_subscribers(topic) for topic in TOPICS.values()):
                assert time.monotonic()<deadline and process.poll() is None,'monitor discovery/startup failed'
                rclpy.spin_once(node,timeout_sec=.05)
            settle(base,5);assert len(events())==5,events()
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                publish(base);time.sleep(.08);assert process.poll() is None
            assert len(events())==5,('steady RL_ZERO emitted repeated events',events())
            fault=copy.deepcopy(base);fault['r3'].update(state='EMERGENCY_FAULT',fault_latched=True,fault='policy_result_stale')
            settle(fault,6)
            for _ in range(20):publish(fault)
            assert len(events())==6
            settle(base,7);assert events()[-1]['event_type']=='RECOVERED'
            active=copy.deepcopy(base);active['r3']['phase']='RL_ACTIVE';settle(active,8)
            assert events()[-1]['changes']=={'phase':{'from':'RL_ZERO','to':'RL_ACTIVE'}}
            assert node.count_publishers('/lowcmd')==0 and node.count_publishers('/api/sport/request')==0
            owned=node.get_publisher_names_and_types_by_node('go2_rars01_architecture_monitor','/')
            assert all(topic in ('/rosout','/parameter_events') for topic,_ in owned),owned
            assert not node.get_client_names_and_types_by_node('go2_rars01_architecture_monitor','/')
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=8);assert process.returncode==0,console.read_text()
            records=[json.loads(line) for line in journal.read_text().splitlines()]
            assert len(records)>100 and sum(r['event'] is not None for r in records)==8
            assert len(events())==8
            print(json.dumps(dict(result='PASS',steady_seconds=30,terminal_events=8,initial_events=5,
                journal_snapshots=len(records),actuator_publishers=0,monitor_control_clients=0)),flush=True)
        except Exception:
            log.flush();print(console.read_text());raise
        finally:
            if process and process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            if process:
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
            node.destroy_node();rclpy.try_shutdown()
