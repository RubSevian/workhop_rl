"""Reject every physical configuration before model/SDK/output construction."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

executable=Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix='r3-probe-guards-') as logs:
    for domain,parameters in [('229',[]),('228',['network_interface:=enP8p1s0']),
                             ('228',['operation_profile:=remote_test']),('228',['read_only:=false'])]:
        env=dict(os.environ,ROS_DOMAIN_ID=domain,ROS_LOCALHOST_ONLY='1',
            RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_LOG_DIR=logs,
            CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>')
        env.pop('ROS_AUTOMATIC_DISCOVERY_RANGE',None);env.pop('GO2_NETWORK_INTERFACE',None)
        args=[str(executable),'--ros-args','-p','motion_diagnostics_enabled:=false',
              '-p','config_path:=/not-loaded.yaml','-p','model_path:=/not-loaded.pt']
        for value in parameters:args+=['-p',value]
        result=subprocess.run(args,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=10)
        assert result.returncode==1 and 'Startup failed closed' in result.stdout,(domain,parameters,result.stdout)
        assert 'Explicit existing config_path' not in result.stdout,'guard was checked after model load'
print('PASS compute probe rejects wrong domain, SDK interface, physical profile and output flag')
