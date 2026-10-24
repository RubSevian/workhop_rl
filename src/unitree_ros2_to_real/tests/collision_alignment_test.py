#!/usr/bin/env python3
"""Offline geometry + path groups; no ROS, SDK, serial, camera or enable."""
import copy
import json
from pathlib import Path
import sys
import time
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[5]
sys.path[:0]=[str(ROOT/'repos/workhop_rl/src/unitree_ros2_to_real/scripts'),str(ROOT/'repos/rars01_graspnet')]
from mission.config import load,ignored_self_collision_pairs,blockers,environment_checks_enabled
from mission.collision import Collision,CollisionComputeTimeout
from mission.transforms import rigid
from rars01_graspnet.config import load_config
from rars01_graspnet.gripper_geometry import RarsGripperGeometry
from rars01_graspnet.kinematics import RarsKinematics
from rars01_graspnet.real_grasp_trajectory import joint_motion_samples
CFG=load(ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',ROOT)
# This is a synthetic fixture, independent of the operator's deployment flag.
# Never require editing the real profile merely to run offline safety tests.
CFG=copy.deepcopy(CFG);CFG['physical_enabled']=False
GRASP=load_config(CFG['paths']['grasp_config']);G=GRASP['robot']['gripper']['rars01']
FIELDS=('linkage_radius_m','connecting_rod_length_m','carriage_width_m','jaw_center_End_link_m','jaw_depth_m','jaw_height_m','R_grasp_End_link')
GEOMETRY=RarsGripperGeometry(**{k:G[k] for k in FIELDS},maximum_width_m=GRASP['robot']['rars01']['max_grasp_width_m'],motor_angle_limit_rad=G['angle_open'])
LEGS=[-.1,.8,-1.5,.1,.8,-1.5,-.1,.8,-1.5,.1,.8,-1.5] # synthetic SDK-order standing; NOT measured
HOME=np.r_[np.zeros(6),GEOMETRY.motor_angle_for_width(.08)]
READY=np.r_[[-.0011,.972,1.3737,-1.1017,-.0009,-.0007],HOME[6]]

class Alignment(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=Collision(CFG,GEOMETRY)
 def test_broadphase_parity_and_clearance_boundary(self):
  # Same real narrow phase and pair order, with/without conservative screening.
  c=copy.copy(self.c);c.last_stats={}
  for q in (HOME,READY,(HOME+READY)/2):
   poses=c.placements(q,LEGS);c.broadphase_enabled=False
   reference=c.self_failure(poses)
   c.broadphase_enabled=True;c.last_stats={}
   actual=c.self_failure(poses)
   self.assertEqual(actual,reference)
   self.assertGreater(c.last_stats.get('broadphase_skipped',0),0)
  cache_size=len(c._solid_cache)
  c.self_failure(c.placements(HOME,LEGS))
  self.assertEqual(len(c._solid_cache),cache_size) # stable native IDs across Pin wrappers
  # Actual primitive geometry tests near the clearance boundary; no mesh-rule
  # exemptions. Far geometry must skip query, near geometry must remain exact.
  from types import SimpleNamespace
  f=c.fcl;objects=[SimpleNamespace(geometry=f.Sphere(1.)) for _ in range(2)]
  c.geom=SimpleNamespace(geometryObjects=objects);c.links=['a','b'];c.arm_links={'a'}
  c.pairs=[(0,1)];c.zero_contact_pairs=set();c._pair_relations={}
  c.topology=SimpleNamespace(relation=lambda *args:'moving_chain')
  c._local_corners=np.stack([c.local_corners(o.geometry) for o in objects])
  for gap,blocked in ((-1e-5,True),(.5*c.margin,True),(10*c.margin,False)):
   poses=[np.eye(4),np.eye(4)];poses[1][0,3]=2+gap
   c.last_stats={};failure=c.self_failure(poses)
   self.assertEqual(failure is not None,blocked)
   self.assertEqual(c.last_stats.get('narrowphase_queries',0),int(blocked))
  # True analytic containment remains a collision with positive surface gap.
  c.geom.geometryObjects[0].geometry=f.Box(4.,4.,4.)
  c._local_corners=np.stack([c.local_corners(o.geometry) for o in objects])
  # Box/sphere are analytic, so containment is already a signed collision.
  self.assertIsNotNone(c.self_failure([np.eye(4),np.eye(4)]))
 def test_actual_mesh_containment_and_empty_space(self):
  from types import SimpleNamespace
  c=copy.copy(self.c);f=c.fcl
  def mesh(boxes):
   vertices=f.StdVec_Vec3s();triangles=f.StdVec_Triangle()
   for centre,half in boxes:
    offset=len(vertices)
    for p in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]:
     vertices.append(np.asarray(centre)+np.asarray(p)*half)
    for a,b,d,e in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
     triangles.append(f.Triangle(offset+a,offset+b,offset+d));triangles.append(f.Triangle(offset+a,offset+d,offset+e))
   shape=f.BVHModelOBBRSS();shape.beginModel(len(triangles),len(vertices));shape.addSubModel(vertices,triangles);shape.endModel()
   return shape
  outer=mesh([([0,0,0],1.)]);inner=mesh([([0,0,0],.1)])
  c.geom=SimpleNamespace(geometryObjects=[SimpleNamespace(geometry=g) for g in (outer,inner)])
  c.links=['a','b'];c.arm_links={'a'};c.pairs=[(0,1)];c.zero_contact_pairs=set();c._pair_relations={}
  c.topology=SimpleNamespace(relation=lambda *args:'moving_chain')
  c._local_corners=np.stack([c.local_corners(o.geometry) for o in c.geom.geometryObjects])
  poses=[np.eye(4),np.eye(4)]
  failure=c.self_failure(poses)
  self.assertEqual(failure['reason'],'SOLID_CONTAINMENT_COLLISION');self.assertFalse(failure['surface_collision'])
  poses[1][0,3]=1.;self.assertTrue(c.self_failure(poses)['surface_collision'])
  poses[1][0,3]=2.;self.assertIsNone(c.self_failure(poses))
  # Empty space between disconnected solids lies inside their combined AABB.
  outer=mesh([([-2,0,0],.5),([2,0,0],.5)])
  c.geom.geometryObjects[0].geometry=outer
  c._local_corners=np.stack([c.local_corners(o.geometry) for o in c.geom.geometryObjects])
  poses=[np.eye(4),np.eye(4)]
  self.assertTrue(c.nested_bounds((c._local_corners[0,:,:3].min(axis=0),c._local_corners[0,:,:3].max(axis=0)),
      (c._local_corners[1,:,:3].min(axis=0),c._local_corners[1,:,:3].max(axis=0))))
  self.assertIsNone(c.self_failure(poses))
  # Nested closed shells bound a cavity, which must remain empty.
  shell=mesh([([0,0,0],2.),([0,0,0],1.)])
  self.assertFalse(c.containment_ambiguous(shell,np.eye(4),inner,np.eye(4)))
  shifted=np.eye(4);shifted[0,3]=1.5
  self.assertTrue(c.containment_ambiguous(shell,np.eye(4),inner,shifted))
 def test_identical_samples_and_compute_deadline(self):
  # New compute controls are actual typed YAML values, not string toggles.
  import tempfile,yaml
  raw=yaml.safe_load((ROOT/'repos/rars01_graspnet/config/jetson_orin_nano.yaml').read_text())['go2_mission']
  with tempfile.TemporaryDirectory(prefix='collision-config-') as directory:
   path=Path(directory)/'mission.yaml'
   for key,value in [('broadphase_enabled','false'),('broadphase_padding_m',0),('check_timeout_s',-1),('check_timeout_s',float('nan'))]:
    invalid=copy.deepcopy(raw);invalid['collision'][key]=value;path.write_text(yaml.safe_dump({'go2_mission':invalid}))
    with self.assertRaises(ValueError):load(path,ROOT)
  c=copy.copy(self.c);c.cfg=copy.deepcopy(CFG)
  now=time.monotonic_ns();scene=dict(legs=LEGS,legs_ns=now,feedback_ns=now,base_ns=now,
      map_body=np.eye(4).tolist(),command_rate_hz=100,velocity_limits=[1.]*7)
  self.assertTrue(c.check([HOME,HOME,HOME],scene),c.last_failure)
  self.assertEqual(c.last_stats['samples'],1);self.assertEqual(c.last_stats['duplicate_samples'],3)
  # Reuse a pair only when every moving joint between its two links is
  # exactly unchanged. Shared rigid motion (joint1) preserves self geometry.
  path=[HOME.copy() for _ in range(4)]
  path[1][0]+=.001;path[2][0]+=.002;path[3][0]+=.002;path[3][1]+=.001
  for q in path:self.assertIsNone(c.self_failure(c.placements(q,LEGS)))
  now=time.monotonic_ns();scene.update(legs_ns=now,feedback_ns=now,base_ns=now)
  self.assertTrue(c.check(path,scene),c.last_failure)
  self.assertGreater(c.last_stats.get('rigid_pair_reused',0),0)
  self.assertIsNone(c._clear_cache)
  self.assertIsNone(c.check_timeout_s)
  c.cancel_reason='OPERATOR_CANCEL'
  with self.assertRaisesRegex(RuntimeError,'COLLISION_CANCELLED'):
   c.check([HOME,HOME],scene)
  c.cancel_reason=''
  c.check_timeout_s=1e-12
  with self.assertRaisesRegex(CollisionComputeTimeout,'COLLISION_COMPUTE_TIMEOUT'):
   c.check([HOME,HOME],scene)
  self.assertEqual(c.last_reason,'COLLISION_COMPUTE_TIMEOUT');self.assertIsNone(c._check_deadline_ns)
  c.check_timeout_s=20.;c.work_deadline_ns=time.monotonic_ns()-1
  with self.assertRaises(CollisionComputeTimeout):c.check([HOME,HOME],scene)
 def test_configured_clearance_preserves_intersection_gate(self):
  c=self.c;poses=c.placements(HOME,LEGS)
  self.assertAlmostEqual(c.margin*1000,.001,places=12)
  pairs,margin,zero_pairs=c.pairs,c.margin,c.zero_contact_pairs
  def pair(a,b):return (c.links.index(a),c.links.index(b))
  try:
   # Real pinned mesh gap passes the requested margin and fails the former one.
   c.pairs=[pair('arm_mount_link','link1_m0')]
   self.assertIsNone(c.self_failure(poses))
   c.margin=.01
   self.assertIsNotNone(c.self_failure(poses))
   c.margin=margin
   # Without an explicit contact rule a reported intersection still blocks.
   c.zero_contact_pairs=set()
   c.pairs=[pair('link1_m0','link2_m1')]
   failure=c.self_failure(poses)
   self.assertIsNotNone(failure);self.assertTrue(failure['surface_collision'])
  finally:c.pairs,c.margin,c.zero_contact_pairs=pairs,margin,zero_pairs
 def test_only_joint1_pair_is_excluded(self):
  c=self.c;excluded=frozenset(('base_link','link1_m0'))
  self.assertEqual(c.ignored_pairs,{excluded})
  actual={frozenset((c.links[i],c.links[j])) for i,j in c.pairs}
  expected={frozenset((c.links[i],c.links[j])) for i,j in c.all_pairs
            if c.topology.relation(c.links[i],c.links[j]) not in ('same_body','rigid_assembly')}
  self.assertEqual(actual,expected-{excluded})
  poses=c.placements(HOME,LEGS);i,j=(c.links.index(n) for n in ('base_link','link1_m0'))
  self.assertTrue(c.query(c.geom.geometryObjects[i].geometry,poses[i],c.geom.geometryObjects[j].geometry,poses[j])[1])
  strict=copy.copy(c);strict.zero_contact_pairs=set()
  failure=strict.self_failure(poses)
  self.assertEqual(failure['pair'],['link1_m0','link2_m1'])
  self.assertTrue(failure['surface_collision'])
  # Reversing the names identifies the same mutual pair; no wildcard matching.
  self.assertEqual(ignored_self_collision_pairs(dict(ignored_self_collision_pairs=[['link1_m0','base_link']])),{excluded})
  for pairs in (None, [['base_link']], [['base_link','base_link']], [['base_link',1]], [['base_link','link1_m0'],['link1_m0','base_link']]):
   with self.assertRaises(ValueError):ignored_self_collision_pairs(dict(ignored_self_collision_pairs=pairs))
 def test_explicit_zero_contacts(self):
  c=copy.copy(self.c)
  expected={frozenset(pair) for pair in [('link1_m0','link2_m1'),('link2_m1','link2_m2'),('link2_m2','link_3_m3'),('link_3_m3','link_4_m4'),('link_4_m4','link_5_m5'),('End_link','gripper_left'),('End_link','gripper_right'),('gripper_left','gripper_right')]}
  self.assertEqual(c.zero_contact_pairs,expected)
  # Use actual pinned STL at open/closed HOME, restricted to authorized pairs.
  c.pairs=[(i,j) for i,j in c.pairs if frozenset((c.links[i],c.links[j])) in expected]
  for q in (HOME,np.zeros(7)):
   self.assertIsNone(c.self_failure(c.placements(q,LEGS)))
  poses=c.placements(HOME,LEGS)
  # A signed negative result is rejected even for a listed pair.
  c.query=lambda *args:(-1e-5,True)
  failure=c.self_failure(poses)
  self.assertIsNotNone(failure);self.assertEqual(failure['distance_m'],-1e-5)
  # Other moving pairs keep both contact and clearance gates.
  c.pairs=[(c.links.index('link2_m1'),c.links.index('link_4_m4'))]
  c.containment_ambiguous=lambda *args,**kwargs:False
  for result in ((0.,True),(.5e-6,False)):
   c.query=lambda *args,result=result:result
   self.assertIsNotNone(c.self_failure(poses))
 def test_geometry_regression(self):
  c=self.c;rows=c.diagnose(HOME,LEGS)
  bypair={frozenset(r['pair']):r for r in rows}
  self.assertEqual(bypair[frozenset(('base_link','link1_m0'))]['clearance_policy'],'excluded_by_config')
  self.assertEqual(bypair[frozenset(('link1_m0','link2_m1'))]['clearance_policy'],'allowed_zero_contact')
  for pair,value in [(('link1_m0','link2_m2'),.005413577),(('link2_m1','link_4_m4'),.000069599),(('link2_m1','link_3_m3'),.000077782)]:
   self.assertAlmostEqual(bypair[frozenset(pair)]['distance_m'],value,places=7)
  self.assertEqual([j['type'] for j in c.topology.chain('arm_mount_link','link1_m0')],['fixed','revolute'])
  self.assertEqual(c.topology.relation('base','arm_mount_link'),'rigid_assembly')
  self.assertEqual(c.topology.relation('arm_mount_link','link1_m0'),'moving_chain')
  # Same canonical FK, no duplicate root/scale compensation.
  fk=RarsKinematics(CFG['paths']['arm_urdf'],'base_link','End_link')
  c.placements(READY,LEGS)
  np.testing.assert_allclose(c.data.oMf[c.model.getFrameId('base_link')].homogeneous@fk.forward(READY[:6]),c.data.oMf[c.model.getFrameId('End_link')].homogeneous,atol=1e-12)
  with self.assertRaises(ValueError):c.placements(np.r_[3.,HOME[1:]],LEGS)
  with self.assertRaises(ValueError):c.placements(HOME,[0]*12) # old MJCF qpos0 calves exceed limits
  # Positive triangle-surface distance misses an enclosed sphere; actual
  # closed-solid containment must catch it without a nested-box rejection.
  f=c.fcl;vertices=f.StdVec_Vec3s();triangles=f.StdVec_Triangle()
  for p in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]:vertices.append(np.array(p,dtype=float))
  for a,b,d,e in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
   triangles.append(f.Triangle(a,b,d));triangles.append(f.Triangle(a,d,e))
  mesh=f.BVHModelOBBRSS();mesh.beginModel(len(triangles),len(vertices));mesh.addSubModel(vertices,triangles);mesh.endModel()
  t=np.eye(4);sphere=f.Sphere(.1)
  distance,hit=c.query(mesh,t,sphere,t);self.assertAlmostEqual(distance,.9);self.assertFalse(hit)
  self.assertTrue(c.containment_ambiguous(mesh,t,sphere,t))
  outside=t.copy();outside[0,3]=2
  self.assertFalse(c.containment_ambiguous(mesh,t,sphere,outside));self.assertFalse(c.query(mesh,t,sphere,outside)[1])
  crossing=t.copy();crossing[0,3]=.99;self.assertTrue(c.query(mesh,t,sphere,crossing)[1])
  print('PASS geometry: pinned distances, moving mount, FK, bounds, surface penetration and actual solid containment')
 def test_explicit_robot_only_scope(self):
  self.assertFalse(CFG['collision']['environment_enabled'])
  missing=blockers(CFG)
  for name in ('environment','floor_z_in_map_m_missing','scene_revision_missing'):
   self.assertNotIn(name,missing)
  for name in ('collision_geometry','payload','payload_box_m_missing','payload_tcp_xyz_m_missing','emergency'):
   self.assertNotIn(name,missing)
  full=copy.deepcopy(CFG);full['collision']['environment_enabled']=True
  for name in ('floor_z_in_map_m_missing','scene_revision_missing'):
   self.assertIn(name,blockers(full))
  self.assertTrue(environment_checks_enabled({})) # old configs retain full checks
  for value in (None,'false',0,1):
   with self.assertRaises(ValueError):environment_checks_enabled(dict(environment_enabled=value))
  c=copy.copy(self.c);c.cfg=copy.deepcopy(CFG)
  # Synthetic scene only; no hardware state is injected.
  def scene():
   now=time.monotonic_ns()
   return dict(legs=LEGS,legs_ns=now,feedback_ns=now,base_ns=now,map_body=np.eye(4).tolist(),
               command_rate_hz=100,velocity_limits=[1.]*7)
  self.assertTrue(c.check([HOME,HOME],scene()),c.last_failure)
  self.assertTrue(c.check([HOME,HOME],scene(),True),c.last_failure)
  c.payload_enabled=True
  self.assertFalse(c.check([HOME,HOME],scene(),True))
  self.assertEqual(c.last_reason,'PAYLOAD_GEOMETRY_MISSING')
  c.payload_enabled=False
  stale=scene();stale['legs_ns']-=10**9
  self.assertFalse(c.check([HOME,HOME],stale));self.assertIn('STALE',c.last_reason)
  c.environment_enabled=True
  self.assertFalse(c.check([HOME,HOME],scene()))
  self.assertEqual(c.last_reason,'SURVEYED_MAP_ENVIRONMENT_MISSING')
  c.floor=.5;c.cfg['collision']['scene_revision']='test-only'
  s=scene();s['scene_revision']='other'
  self.assertFalse(c.check([HOME,HOME],s));self.assertEqual(c.last_reason,'COLLISION_SCENE_REVISION_MISMATCH')
  s=scene();s['scene_revision']='test-only'
  self.assertFalse(c.check([HOME,HOME],s));self.assertIn('ENVIRONMENT_',c.last_reason)
  # Same intersecting external floor is explicitly ignored in robot-only mode.
  c.environment_enabled=False
  self.assertTrue(c.check([HOME,HOME],scene()),c.last_failure)
  # Disabling the environment cannot suppress a robot collision query.
  c.query=lambda *args:(-.001,True)
  self.assertFalse(c.check([HOME,HOME],scene()));self.assertTrue(c.last_reason.startswith('COLLISION:'))
  self.assertEqual(c.last_failure['sample_index'],0)
 def test_full_path_payload_fail_closed(self):
  c=copy.copy(self.c);refine=READY.copy();refine[3]+=.01;pre=refine.copy();pre[0]+=.02;grasp=pre.copy();grasp[1]+=.02
  hold=grasp.copy();hold[6]=GEOMETRY.motor_angle_for_width(.06)
  home=HOME.copy();home[6]=hold[6]
  phases=[('SCAN',HOME,READY,False),('REFINE',READY,refine,False),('PREGRASP',refine,pre,False),('GRASP',pre,grasp,False),('CLOSE_SWEEP',grasp,np.r_[grasp[:6],0.],False),('RETREAT',hold,refine.copy(),True),('HOME6_HOLD',np.r_[refine[:6],hold[6]],home,True)]
  results=[]
  for name,start,end,payload in phases:
   if payload:end[6]=start[6]
   points,duration=joint_motion_samples(start,end,1.,.01,np.r_[np.full(6,.1),2.])
   t=time.perf_counter();failure=c.self_failure(c.placements(points[0],LEGS),0)
   if failure:
    self.assertNotIn(frozenset(failure['pair']),c.ignored_pairs)
    if frozenset(failure['pair']) in c.zero_contact_pairs:self.assertLess(failure['distance_m'],0)
   self.assertFalse(c.check(points,dict(legs=LEGS),payload))
   results.append(dict(phase=name,samples=len(points),duration_s=duration,geometry_first=failure,admission=c.last_reason,elapsed_s=time.perf_counter()-t))
  print(json.dumps(dict(scope='synthetic_trajectories_NOT_measured_safe',phases=results,physical_commands=0)))
  for invalid in ([],[HOME.tolist()],[[float('nan')]*7]*2):self.assertFalse(c.check(invalid,{}))
  self.assertEqual(c.last_reason,'INVALID_COLLISION_PATH')
  now=time.monotonic_ns();scene=dict(legs_ns=now-10**9,feedback_ns=now,base_ns=now)
  self.assertFalse(c.check([HOME,HOME],scene,True));self.assertIn('STALE',c.last_reason)
  scene.update(legs_ns=now)
  self.assertFalse(c.check([HOME,HOME],scene,True));self.assertIn('map_body',c.last_reason)
  self.assertFalse(CFG['physical_enabled']);self.assertNotIn('qualification',CFG)
  # Real planning result-validation method, without constructing any backend.
  from mission.planning import Planning
  from mission.scene import FrameChain,BodyPose,SceneSnapshot
  from types import SimpleNamespace
  planner=Planning.__new__(Planning);planner.cfg=copy.deepcopy(CFG);planner.grasp=GRASP
  planner.cfg['frames']['vehicle_to_mount']=planner.cfg['frames']['vehicle_to_mount_candidate']
  planner.frames=FrameChain(planner.cfg)
  planner.collision=c;planner.eye=SimpleNamespace(vehicle_mount=np.eye(4))
  sample=dict(legs=list(LEGS),legs_ns=now,feedback_ns=now,odom_mono_ns=now,
              odom_vehicle=np.eye(4),owner_session_id='synthetic',q=HOME[:6].tolist())
  sample['body']=BodyPose(now,now,'synthetic-nav','tf1',np.eye(4).tolist(),planner.frames.body(np.eye(4)).tolist())
  sample['scene_snapshot']=SceneSnapshot(sample['body'],tuple(HOME),tuple(LEGS),now,now,'synthetic').data()
  old=planner.collision_scene(sample)
  result=dict(trajectory=dict(points=[HOME.tolist(),HOME.tolist()],collision_context=old,payload_checked=False))
  changed=copy.deepcopy(sample);changed['legs'][0]+=.001
  planner.validate_result(result,lambda *args:changed)
  changed=copy.deepcopy(sample);changed['q'][1]+=.03
  with self.assertRaisesRegex(ValueError,'TRAJECTORY_START_CHANGED'):planner.validate_result(result,lambda *args:changed)
  # A changed state cannot bypass real geometry admission during rebuild.
  def recheck(request,context):
   return dict(trajectory=planner.checked([HOME,HOME],changed,False))
  planner.execute=recheck
  c.query=lambda *args:(-.001,True)
  with self.assertRaisesRegex(ValueError,'COLLISION'):
   planner.validate_result(result,lambda *args:changed,request={'operation':'home'})


if __name__=='__main__':unittest.main()
