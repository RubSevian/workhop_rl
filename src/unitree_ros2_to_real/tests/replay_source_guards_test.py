"""Fail before opening any DDS socket/device with unsafe replay environment."""
import os
import subprocess
import sys
from pathlib import Path
source,production=map(Path,sys.argv[1:3])
dds='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
base=dict(os.environ,ROS_DOMAIN_ID='228',ROS_LOCALHOST_ONLY='1',CYCLONEDDS_URI=dds,RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
base.pop('GO2_NETWORK_INTERFACE',None);base.pop('ROS_STATIC_PEERS',None)
for update in ({'ROS_DOMAIN_ID':'0'},{'ROS_LOCALHOST_ONLY':'0'},{'GO2_NETWORK_INTERFACE':'enP8p1s0'},{'ROS_STATIC_PEERS':'192.168.123.161'},{'CYCLONEDDS_URI':dds.replace('name="lo"','name="enP8p1s0"')}):
    env=base|update
    run=subprocess.run([str(source),'/not-opened.bin','5','/not-opened.ns','/not-opened'],env=env,capture_output=True,text=True,timeout=10)
    assert run.returncode==1 and 'Replay requires' in run.stderr,(update,run.stdout,run.stderr)
for update in ({'ROS_DOMAIN_ID':'229'},{}):
    args=[str(production),'--ros-args','-p','policy_worker_self_test_frames:=1','-p','motion_diagnostics_enabled:=false','-p','model_path:=/not-loaded.pt','-p','config_path:=/not-loaded.yaml']
    # Valid domain with a physical profile also has to fail before model load.
    if not update:args+=['-p','operation_profile:=remote_test']
    run=subprocess.run(args,env=base|update,capture_output=True,text=True,timeout=10)
    assert run.returncode==1 and 'Startup failed closed' in run.stderr+run.stdout
    assert 'Explicit existing config_path' not in run.stderr+run.stdout
print('PASS isolated source environment and production self-test fail-closed guards')
