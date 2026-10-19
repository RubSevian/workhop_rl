#!/usr/bin/env python3
"""Reproduce diagnostic evidence Markdown only. Never loads SDK/ROS/camera."""
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from collision_alignment_test import ROOT,CFG,GEOMETRY,LEGS,HOME,READY,Collision

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
 c=Collision(CFG,GEOMETRY);world=np.eye(4);world[2,3]=.42
 output=['# Collision alignment evidence — offline synthetic geometry',
  '', '17.10.2026. All joints below are diagnostic, NOT measured-safe. Actuator/serial/LowCmd commands: 0.',
  '', 'Coal triangle BVH. SDK leg order FR/FL/RR/RL, hip/thigh/calf. T_map_body: translation [0,0,0.42]m, quaternion [0,0,0,1].',
  '', '`legs = '+json.dumps(LEGS)+'`','', '## Git snapshot (dirty sources retained)','']
 for repo in ('.','repos/workhop_rl','repos/autonomy_nav_go2','repos/rars01_description','repos/rars01_graspnet','repos/rars_arm_sdk'):
  head=subprocess.check_output(['git','-C',str(ROOT/repo),'rev-parse','HEAD'],text=True).strip()
  status=subprocess.check_output(['git','-C',str(ROOT/repo),'status','--short'],text=True).strip()
  output+=['`'+repo+'` HEAD `'+head+'`','', '```text',status or '(clean)','```','']
 output+=['## Sources / scale / conservative proxy proposal','',
  'No proxy activated. Local AABB boxes below enclose each loaded STL but may over-block cavities/hinges. CAD/contact qualification still required.',
  '', '| Link / geometry | Loaded bounds min→max (m) | Mesh SHA256 / source |','|---|---|---|']
 for link,g in zip(c.links,c.geom.geometryObjects):
  path=getattr(g,'meshPath','')
  if not path:continue
  if path.startswith('package://rars01_description/'):
   path=str(Path(CFG['paths']['description_root'])/path.split('package://rars01_description/')[1])
  if not Path(path).is_file():continue
  g.geometry.computeLocalAABB();bounds=[g.geometry.aabb_local.min_.tolist(),g.geometry.aabb_local.max_.tolist()]
  output+=[f'| {link} / {g.name} | `{np.round(bounds,6).tolist()}` | `{sha(path)}` `{path}` |']
 for key in ('arm_urdf','arm_description','full_urdf'):
  output+=['',f'{key}: `{CFG["paths"][key]}`, SHA256 `{sha(CFG["paths"][key])}`.']
 datasets={}
 for name,q in [('HOME80',HOME),('HOME_CLOSED',np.zeros(7)),('READY_DIAGNOSTIC',READY),('JOINT1_MIN',np.r_[-2.8,HOME[1:]]),('JOINT1_MAX',np.r_[2.8,HOME[1:]])]:
  start=time.perf_counter();rows=c.diagnose(q,LEGS,map_body=world);elapsed=time.perf_counter()-start
  datasets[name]=dict(q=q.tolist(),rows=rows,elapsed_s=elapsed)
  output+=['',f'## {name}', '', '`q7 = '+json.dumps(q.tolist())+'`', '', f'Full near-pair scan: {elapsed:.3f}s. No admission/physical PASS implied.', '',
   '| Links | Geometries | Distance mm | Coal collide | Class / clearance rule | Moving joints in chain |',
   '|---|---|---:|---|---|---|']
  for r in rows:
   moving=','.join(j['name'] for j in r['chain'] if j['type']!='fixed') or 'none (fixed)'
   output+=[f'| {" ↔ ".join(r["pair"])} | {" / ".join(r["geometry"])} | {r["distance_m"]*1000:.6f} | {r["surface_collision"]} | {r["category"]}; {r["clearance_policy"]} | {moving} |']
 # Historical sources are explicit; MuJoCo module is unavailable in current venv.
 output+=['','## Historical M0.3 pairs reproduced on current pinned triangles','',
  'MuJoCo and old Coal values are historical numbers from Stage4D M0.3 report/task, not a fresh MuJoCo run. Self distances are invariant to leg pose/world rigid transform. Old probe qpos0 legs=0 violates calf limits and is never an execution default.', '',
  '| Pair | Historical MJ mm | Historical Coal mm | Current Coal mm | Current collide |','|---|---:|---:|---:|---|']
 poses=c.placements(HOME,LEGS);pairs=[('link1_m0','link2_m2',2.575,5.414),('link1_m0','link_4_m4',.064,12.022),('link2_m1','link_4_m4',.066,.070),('link2_m1','link_3_m3',.078,.078),('link2_m2','link_4_m4',11.559,14.195)]
 for a,b,mj,old in pairs:
  i,j=c.links.index(a),c.links.index(b);d,hit=c.query(c.geom.geometryObjects[i].geometry,poses[i],c.geom.geometryObjects[j].geometry,poses[j])
  output+=[f'| {a} ↔ {b} | {mj} | {old} | {d*1000:.6f} | {hit} |']
 destination=ROOT/'docs/test_statistics/2026_10_17/COLLISION_ALIGNMENT_EVIDENCE.md';destination.parent.mkdir(parents=True,exist_ok=True)
 marker='\n## Initial profile snapshot (before this task)'
 historical=''
 if destination.exists() and marker in destination.read_text():historical=marker+destination.read_text().split(marker,1)[1]
 destination.write_text('\n'.join(output)+'\n'+historical)
 print(destination);print(json.dumps({name:dict(near_pairs=len(d['rows']),seconds=d['elapsed_s']) for name,d in datasets.items()}))
if __name__=='__main__':main()
