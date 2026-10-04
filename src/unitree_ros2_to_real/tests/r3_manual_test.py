import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from r3_manual_lib import validate, eligible
assert validate((.1,0,0),1)==(.1,0,0)
for cmd,duration in [((.1,.1,0),1),((.21,0,0),1),((0,.11,0),1),((0,0,.11),1),((float('nan'),0,0),1),((.1,0,0),0),((.1,0,0),2)]:
    try:validate(cmd,duration)
    except ValueError:pass
    else:raise AssertionError((cmd,duration))
s={'state':'RL_ZERO','read_only':False,'output_enabled':True,'fault_latched':False,'blockers':[]}
assert eligible(s,0,.1)
assert not eligible(s,0,.251)
assert not eligible(s,1,0)
for key,value in [('read_only',True),('output_enabled',False),('fault_latched',True),('state','HOLDING'),('blockers',['stale'])]:
    bad=s.copy();bad[key]=value;assert not eligible(bad,0,.1)
assert not eligible({},0,.1)
print('PASS manual bounds/state/freshness; pure Python, no ROS or publisher')
