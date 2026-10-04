"""Inspect loader resolution with Jazzy first in LD_LIBRARY_PATH. No SDK RPC."""
import os
from pathlib import Path
import re
import subprocess
import sys

binary=Path(sys.argv[1]).resolve()
sdk=Path(sys.argv[2]).resolve()
env=dict(os.environ)
env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:'+env.get('LD_LIBRARY_PATH','')
dynamic=subprocess.check_output(['readelf','-d',str(binary)],text=True,env=env)
assert '(RPATH)' in dynamic and '(RUNPATH)' not in dynamic, dynamic
loaded=subprocess.check_output(['ldd',str(binary)],text=True,env=env)
for soname in ('libddsc.so.0','libddscxx.so.0'):
    line=next((line for line in loaded.splitlines() if re.match(r'\s*'+re.escape(soname)+r'\s+=>',line)),None)
    assert line, loaded
    path=Path(line.split('=>',1)[1].strip().split()[0]).resolve()
    assert path.parent==sdk, f'{soname} mixed DDS library: {path}; expected {sdk}'
print('PASS SDK DDS isolation with Jazzy LD_LIBRARY_PATH: both libraries from',sdk)
