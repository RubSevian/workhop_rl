import subprocess,sys,hashlib
old,new,config=sys.argv[1:]
a=subprocess.run([old,config],capture_output=True,check=True).stdout
b=subprocess.run([new,config],capture_output=True,check=True).stdout
if a!=b:
 for i,(v,w) in enumerate(zip(a.splitlines(),b.splitlines())):
  if v!=w:raise AssertionError(f"baseline mismatch line {i}: {v[:100]!r} != {w[:100]!r}")
 raise AssertionError("trace length differs")
print('PASS byte-identical A capture/stand/hold/reset/first-policy trace:',hashlib.sha256(a).hexdigest())
