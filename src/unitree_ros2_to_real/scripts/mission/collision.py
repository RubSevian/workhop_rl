"""Coal triangle-BVH checker; diagnostics and execution admission stay separate.

Explicit YAML pair rules affect mutual self-collision only. Allowed zero
contacts still block negative distance. A triangle surface's nonnegative
distance does NOT certify absence of solid penetration.
"""
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from .transforms import rigid, xyzrpy
from .config import ignored_self_collision_pairs, allowed_zero_contact_pairs, environment_checks_enabled, payload_checks_enabled

from .gripper_measurement import closed_zero

def measured_gripper_width(geometry,angle,tolerance):
    """Symmetric measured closed-zero band; never changes a motor command."""
    return geometry.opening_width(closed_zero(angle,tolerance))

def validate_geometry_joint(name,value,lower,upper,arm_tolerance):
    """Accept small arm measurement overshoot, preserving measured FK geometry.

    This does not change URDF/IK/owner command limits and never applies to legs.
    """
    if not np.isfinite(arm_tolerance) or not 0<=arm_tolerance<=np.deg2rad(2)+1e-12:
        raise ValueError('INVALID_ARM_MEASUREMENT_TOLERANCE')
    tolerance=arm_tolerance if name in ('joint1','joint2','joint3','joint4','joint5','joint6') else 0.
    if not np.isfinite(value) or not lower-tolerance-1e-9<=value<=upper+tolerance+1e-9:
        raise ValueError('COLLISION_JOINT_LIMIT:'+name)
    return value


def timed_collision_stage(method):
    """Inclusive stage timings; nested totals must not be added together."""
    from functools import wraps
    @wraps(method)
    def measured(self,*args,**kwargs):
        stats=getattr(self,'last_stats',None)
        if not getattr(self,'_profiling',False) or stats is None:return method(self,*args,**kwargs)
        stage=method.__name__;previous=stats.get('active_stage')
        started=time.monotonic_ns();cpu=time.thread_time_ns()
        stats['active_stage']=stage
        try:return method(self,*args,**kwargs)
        except BaseException:
            stats.setdefault('exception_stage',stage)
            raise
        finally:
            elapsed=(time.monotonic_ns()-started)/1e9
            old=stats.get(stage,{})
            stats[stage]=dict(calls=old.get('calls',0)+1,wall_s=old.get('wall_s',0.)+elapsed,
                cpu_s=old.get('cpu_s',0.)+(time.thread_time_ns()-cpu)/1e9,
                max_wall_s=max(old.get('max_wall_s',0.),elapsed),
                slowest_pair=stats.get('current_pair') if elapsed>=old.get('max_wall_s',0.) else old.get('slowest_pair'))
            stats['active_stage']=previous
    return measured


class CollisionCancelled(RuntimeError):
    """Explicit operation cancellation, distinct from exhausted compute budget."""


class CollisionComputeTimeout(RuntimeError):
    """Abort the whole planning request; not an infeasible grasp candidate."""

class Topology:
    def __init__(self, path):
        root=ET.parse(path).getroot()
        self.edges={}
        for j in root.findall('joint'):
            a,b=j.find('parent').get('link'),j.find('child').get('link')
            item=dict(name=j.get('name'),type=j.get('type'))
            self.edges.setdefault(a,[]).append((b,item));self.edges.setdefault(b,[]).append((a,item))
    def chain(self,a,b):
        queue=[(a,[])];seen=set()
        for node,path in queue:
            if node==b:return path
            seen.add(node)
            queue.extend((child,path+[joint]) for child,joint in self.edges.get(node,[]) if child not in seen)
        raise ValueError('DISCONNECTED_COLLISION_FRAMES')
    def relation(self,a,b):
        if a==b:return 'same_body'
        path=self.chain(a,b)
        if all(j['type']=='fixed' for j in path):return 'rigid_assembly'
        return 'moving_adjacent' if len(path)==1 else 'moving_chain'

class Collision:
    def __init__(self,cfg,geometry):
        import pinocchio as pin
        import coal
        self.pin,self.fcl,self.cfg,self.gripper=pin,coal,cfg,geometry
        self.model=pin.buildModelFromUrdf(cfg['paths']['full_urdf'])
        self.geom=pin.buildGeomFromUrdf(self.model,cfg['paths']['full_urdf'],pin.GeometryType.COLLISION,
            package_dirs=[str(Path(cfg['paths']['description_root']).parent)])
        self.topology=Topology(cfg['paths']['full_urdf'])
        self.links=[self.model.frames[g.parentFrame].name for g in self.geom.geometryObjects]
        if not self.links or any(g.geometry is None for g in self.geom.geometryObjects):raise ValueError('COLLISION_GEOMETRY_MISSING')
        self.arm_links={x.get('name') for x in ET.parse(cfg['paths']['arm_description']).getroot().findall('link')}
        self.all_pairs=[(i,j) for i in range(len(self.links)) for j in range(i+1,len(self.links))
                        if self.links[i] in self.arm_links or self.links[j] in self.arm_links]
        self.ignored_pairs=ignored_self_collision_pairs(cfg['collision'])
        self.zero_contact_pairs=allowed_zero_contact_pairs(cfg['collision'])
        if self.ignored_pairs & self.zero_contact_pairs:raise ValueError('CONFLICTING_SELF_COLLISION_PAIR_RULES')
        for pair in self.ignored_pairs | self.zero_contact_pairs:
            if not pair.issubset(set(self.links)) or not pair.intersection(self.arm_links):
                raise ValueError('UNKNOWN_SELF_COLLISION_PAIR:'+':'.join(sorted(pair)))
        self.pairs=[(i,j) for i,j in self.all_pairs
                    if self.topology.relation(self.links[i],self.links[j]) not in ('same_body','rigid_assembly')
                    and frozenset((self.links[i],self.links[j])) not in self.ignored_pairs]
        self.data,self.gdata=self.model.createData(),self.geom.createData()
        self.margin=float(cfg['collision']['margin_m'])
        if not np.isfinite(self.margin) or self.margin<=0:raise ValueError('COLLISION_MARGIN_INVALID')
        self.broadphase_enabled=cfg['collision'].get('broadphase_enabled',True)
        self.broadphase_padding=float(cfg['collision'].get('broadphase_padding_m',1e-9))
        self.check_timeout_s=cfg['collision'].get('check_timeout_s',20.)
        if (type(self.broadphase_enabled) is not bool or not np.isfinite(self.broadphase_padding)
                or self.broadphase_padding<=0 or (self.check_timeout_s is not None and (not np.isfinite(self.check_timeout_s) or self.check_timeout_s<=0))):
            raise ValueError('COLLISION_COMPUTE_SETTINGS_INVALID')
        self._corners_cache={};self._pair_relations={};self._solid_cache={}
        self._local_corners=np.stack([self.local_corners(g.geometry) for g in self.geom.geometryObjects])
        self._check_deadline_ns=None;self.work_deadline_ns=None
        self._distance_request=self.fcl.DistanceRequest()
        self._collision_request=self.fcl.CollisionRequest()
        self._bound_request=self.fcl.CollisionRequest()
        self._bound_request.break_distance=self.margin+self.broadphase_padding
        # Coal 3.0.2 MeshDistanceTraversalNode reduces sqrt(triangle distance²):
        # this particular mesh/mesh kernel is unsigned, even with signed=true.
        # Other versions/shapes/custom query implementations use the full query.
        self._unsigned_mesh_kernel=str(getattr(self.fcl,'__version__',''))=='3.0.2'
        # Immutable solid topology/ray acceleration belongs to initialization,
        # never to the bounded trajectory-check budget.
        self.cache_hits=0;self.cache_misses=0
        for obj in self.geom.geometryObjects:
            if type(obj.geometry).__name__.startswith('BVHModel'):
                key=int(obj.geometry.id())
                if key not in self._solid_cache:
                    from .solid import MeshSolid
                    solid,hit=MeshSolid.cached(obj.geometry,cfg['paths'].get('geometry_cache'),
                        obj.meshPath,obj.meshScale,str(getattr(coal,'__version__','')))
                    self._solid_cache[key]=(obj.geometry,solid)
                    self.cache_hits+=int(hit);self.cache_misses+=int(not hit)
        self._clear_cache=None;self._placements_ref=None;self._placements_q=None
        self._pair_q_indices={}
        for i,j in self.pairs:
            indices=[]
            for joint in self.topology.chain(self.links[i],self.links[j]):
                if joint['type']=='fixed':continue
                jid=self.model.getJointId(joint['name'])
                indices.extend(range(self.model.idx_qs[jid],self.model.idx_qs[jid]+self.model.nqs[jid]))
            self._pair_q_indices[(i,j)]=indices
        self.last_stats={}
        self.last_reason='';self.last_failure={}
        self.environment_enabled=environment_checks_enabled(cfg['collision'])
        self.floor=cfg['collision'].get('floor_z_in_map_m')
        self.boxes=cfg['collision']['environment_boxes']
        self.payload_enabled=payload_checks_enabled(cfg['collision'])
        self.payload_size=cfg['collision']['payload_box_m'];self.payload_xyz=cfg['collision']['payload_tcp_xyz_m']
        self.pin.forwardKinematics(self.model,self.data,self.pin.neutral(self.model));self.pin.updateFramePlacements(self.model,self.data)
        if any(j['type']!='fixed' for j in self.topology.chain('base','arm_mount_link')):raise ValueError('MOVING_BODY_MOUNT')
        self.body_mount=rigid(self.data.oMf[self.model.getFrameId('arm_mount_link')].homogeneous.copy())

    @timed_collision_stage
    def query(self,a,ta,b,tb):
        ta,tb=rigid(ta),rigid(tb)
        x,y=(self.fcl.Transform3s(t[:3,:3],t[:3,3]) for t in (ta,tb))
        out=self.fcl.DistanceResult()
        d=float(self.fcl.distance(a,x,b,y,self._distance_request,out))
        cr=self.fcl.CollisionResult();count=self.fcl.collide(a,x,b,y,self._collision_request,cr)
        if not np.isfinite(d):raise ValueError('NONFINITE_COLLISION_DISTANCE')
        return d,bool(count or cr.isCollision())

    @timed_collision_stage
    def native_bound_query(self,ga,x,gb,y,out):
        return self.fcl.collide(ga,x,gb,y,self._bound_request,out)

    def local_corners(self,shape):
        key=int(shape.id())
        if key not in self._corners_cache:
            shape.computeLocalAABB()
            lo,hi=np.asarray(shape.aabb_local.min_),np.asarray(shape.aabb_local.max_)
            if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(lo>hi):
                raise ValueError('COLLISION_AABB_INVALID')
            corners=np.array([[x,y,z,1.] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
            self._corners_cache[key]=(shape,corners)
        return self._corners_cache[key][1]

    @staticmethod
    def nested_bounds(a,b):
        (a0,a1),(b0,b1)=a,b
        return bool((np.all(a0<=b0) and np.all(b1<=a1)) or (np.all(b0<=a0) and np.all(a1<=b1)))

    def check_deadline(self):
        callback=getattr(self,'cancel_check',None)
        if callback is not None:callback()
        if getattr(self,'cancel_reason',''):
            raise CollisionCancelled('COLLISION_CANCELLED:'+self.cancel_reason)
        now=time.monotonic_ns()
        if ((self._check_deadline_ns is not None and now>=self._check_deadline_ns)
                or (self.work_deadline_ns is not None and now>=self.work_deadline_ns)):
            raise CollisionComputeTimeout('COLLISION_COMPUTE_TIMEOUT')

    def rigid_placements(self,poses):
        """Same SE(3) validation as rigid(), batched across the full robot."""
        t=np.asarray(poses,dtype=float)
        if (t.shape!=(len(self.links),4,4) or not np.isfinite(t).all()
                or not np.allclose(t[:,3,:],[0,0,0,1],atol=1e-8)):
            raise ValueError('INVALID_TRANSFORM')
        r=t[:,:3,:3]
        if (not np.allclose(r.transpose(0,2,1)@r,np.eye(3),atol=1e-6)
                or np.any(np.abs(np.linalg.det(r)-1)>1e-6)):
            raise ValueError('INVALID_ROTATION')
        return t

    @timed_collision_stage
    def mesh_solid(self,shape):
        key=int(shape.id())
        if key not in self._solid_cache:
            from .solid import MeshSolid
            self._solid_cache[key]=(shape,MeshSolid(shape))
        return self._solid_cache[key][1]

    @timed_collision_stage
    def containment_ambiguous(self,a,ta,b,tb,validated=False):
        """Compatibility name: now detects actual closed-mesh containment.

        Call only after excluding surface intersection. Bounds only screen work;
        their overlap or nesting never constitutes a collision.
        """
        if not validated:ta,tb=rigid(ta),rigid(tb)
        for outer,to,inner,ti in ((a,ta,b,tb),(b,tb,a,ta)):
            if not type(outer).__name__.startswith('BVHModel'):continue
            self.local_corners(outer)
            solid=self.mesh_solid(outer)
            if not solid.closed:continue
            rotation=to[:3,:3].T@ti[:3,:3]
            translation=to[:3,:3].T@(ti[:3,3]-to[:3,3])
            if type(inner).__name__.startswith('BVHModel'):
                points=self.mesh_solid(inner).representatives
            else:
                # Supported analytic primitives are connected and centred on origin.
                if type(inner).__name__ not in ('Box','Sphere','Capsule','Cylinder','Cone','Ellipsoid'):
                    raise ValueError('SOLID_PRIMITIVE_UNSUPPORTED')
                points=np.zeros((1,3))
            lo,hi=np.asarray(outer.aabb_local.min_),np.asarray(outer.aabb_local.max_)
            points=points@rotation.T+translation
            candidates=points[np.all(points>=lo,axis=1)&np.all(points<=hi,axis=1)]
            for point in candidates:
                self.check_deadline()
                if solid.contains(point):return True
        return False

    def distance(self,a,ta,b,tb):return self.query(a,ta,b,tb)[0]

    @timed_collision_stage
    def placements(self,state,legs):
        state,legs=np.asarray(state,dtype=float),np.asarray(legs,dtype=float)
        if state.shape!=(7,) or legs.shape!=(12,) or not np.isfinite(np.r_[state,legs]).all():raise ValueError('INVALID_JOINT_STATE')
        q=self.pin.neutral(self.model)
        names=[prefix+'_'+joint+'_joint' for prefix in ('FR','FL','RR','RL') for joint in ('hip','thigh','calf')]
        width=measured_gripper_width(self.gripper,float(state[6]),self.cfg['collision'].get('gripper_closed_measurement_tolerance_rad',0.))
        entries=list(zip(names,legs))+list(zip(('joint'+str(i) for i in range(1,7)),state[:6]))+[(n,width/2) for n in ('gripper_left_joint','gripper_right_joint')]
        for name,value in entries:
            j=self.model.getJointId(name)
            if j>=self.model.njoints or self.model.nqs[j]!=1:raise ValueError('COLLISION_JOINT_MAPPING:'+name)
            k=self.model.idx_qs[j]
            q[k]=validate_geometry_joint(name,value,self.model.lowerPositionLimit[k],self.model.upperPositionLimit[k],
                self.cfg['collision'].get('arm_joint_measurement_tolerance_rad',0.))
        self.pin.forwardKinematics(self.model,self.data,q);self.pin.updateFramePlacements(self.model,self.data)
        self.pin.updateGeometryPlacements(self.model,self.data,self.geom,self.gdata,q)
        poses=self.rigid_placements([t.homogeneous for t in self.gdata.oMg])
        self._placements_ref=poses;self._placements_q=q.copy()
        return poses

    def category(self,a,b):
        relation=self.topology.relation(a,b)
        if relation in ('same_body','rigid_assembly','moving_adjacent'):return relation
        if a in self.arm_links and b in self.arm_links:return 'nonadjacent_arm_self'
        return 'arm_leg' if any(x.startswith(('FR_','FL_','RR_','RL_')) for x in (a,b)) else 'arm_go2_body'

    def diagnose(self,state,legs,threshold=.01,map_body=None):
        """Geometry ONLY, never a motion certificate; includes excluded pairs."""
        poses=self.placements(state,legs);rows=[]
        if map_body is not None:poses=[rigid(map_body)@t for t in poses]
        for i,j in self.all_pairs:
            a,b=self.links[i],self.links[j]
            self.last_stats['current_pair']=[a,b];d,hit=self.query(self.geom.geometryObjects[i].geometry,poses[i],self.geom.geometryObjects[j].geometry,poses[j])
            if d<threshold or hit:
                relation=self.topology.relation(a,b)
                rows.append(dict(pair=[a,b],geometry=[self.geom.geometryObjects[k].name for k in (i,j)],distance_m=d,
                    surface_collision=hit,containment_ambiguous=not hit and self.containment_ambiguous(self.geom.geometryObjects[i].geometry,poses[i],self.geom.geometryObjects[j].geometry,poses[j]),category=self.category(a,b),chain=self.topology.chain(a,b),
                    clearance_policy='excluded_by_config' if frozenset((a,b)) in self.ignored_pairs else 'allowed_zero_contact' if frozenset((a,b)) in self.zero_contact_pairs else 'assembly' if relation in ('same_body','rigid_assembly') else 'penetration_only' if relation=='moving_adjacent' else 'configured_margin',margin_m=self.margin))
        return sorted(rows,key=lambda r:r['distance_m'])

    @timed_collision_stage
    def self_failure(self,poses,sample=0):
        self.check_deadline()
        # Keep the original narrow-phase order. Bounding-box distance is a
        # LOWER bound on mesh distance; only proven distant pairs are skipped.
        transforms=poses if poses is getattr(self,'_placements_ref',None) else self.rigid_placements(poses)
        world=(transforms@self._local_corners.transpose(0,2,1)).transpose(0,2,1)[:,:,:3]
        low,high=world.min(axis=1),world.max(axis=1)
        pairs=np.asarray(self.pairs,dtype=int).reshape(-1,2)
        if self.broadphase_enabled and len(pairs):
            i,j=pairs[:,0],pairs[:,1]
            gap=np.maximum(np.maximum(low[i]-high[j],low[j]-high[i]),0.)
            scale=max(1.,float(np.max(np.abs(world))))
            bound=self.margin+self.broadphase_padding+16*np.finfo(float).eps*scale
            selected=np.einsum('ij,ij->i',gap,gap)<=bound*bound
            skipped=int((~selected).sum())
            self.last_stats['broadphase_skipped']=self.last_stats.get('broadphase_skipped',0)+skipped
            pairs=pairs[selected]
        for i,j in pairs:
            i,j=int(i),int(j) # Pinocchio's geometry vector requires Python int.
            self.check_deadline()
            cache_key=None
            if (self._clear_cache is not None and poses is self._placements_ref
                    and getattr(self.query,'__func__',None) is Collision.query):
                cache_key=tuple(self._placements_q[self._pair_q_indices[(i,j)]])
                if self._clear_cache.get((i,j))==cache_key:
                    self.last_stats['rigid_pair_reused']=self.last_stats.get('rigid_pair_reused',0)+1
                    continue
            def remember_clear():
                if cache_key is not None:self._clear_cache[(i,j)]=cache_key
            a,b=self.links[i],self.links[j]
            self.last_stats['current_pair']=[a,b]
            ga,gb=self.geom.geometryObjects[i].geometry,self.geom.geometryObjects[j].geometry
            ambiguous=(any(type(g).__name__.startswith('BVHModel') for g in (ga,gb))
                       and np.all(low[i]<=high[j]) and np.all(low[j]<=high[i]))
            key=(int(i),int(j))
            if key not in self._pair_relations:self._pair_relations[key]=self.topology.relation(a,b)
            direct=self._pair_relations[key]=='moving_adjacent'
            zero_contact=frozenset((a,b)) in self.zero_contact_pairs
            native_mesh=(self.broadphase_enabled and self._unsigned_mesh_kernel
                and getattr(self.query,'__func__',None) is Collision.query
                and all(type(g).__name__=='BVHModelOBBRSS' for g in (ga,gb)))
            contained=None
            if native_mesh:
                x,y=(self.fcl.Transform3s(t[:3,:3],t[:3,3]) for t in (transforms[i],transforms[j]))
                out=self.fcl.CollisionResult()
                count=self.native_bound_query(ga,x,gb,y,out)
                hit=bool(count or out.isCollision())
                lower=float(out.distance_lower_bound)
                self.last_stats['native_bound_queries']=self.last_stats.get('native_bound_queries',0)+1
                self.check_deadline()
                if not hit and ambiguous:
                    contained=self.containment_ambiguous(ga,transforms[i],gb,transforms[j],validated=True)
                proven_clear=not hit and not contained
                # Allowed mesh contacts accept zero/positive unsigned distance;
                # containment without a surface hit still requires rejection.
                if zero_contact and np.isfinite(lower) and (hit or proven_clear):
                    remember_clear();continue
                if (proven_clear and np.isfinite(lower)
                        and (direct or lower>self.margin+self.broadphase_padding)):
                    remember_clear();continue
            # Ambiguous/unproven cases retain EXACT signed distance + collide,
            # including all primitives, custom queries and failure diagnostics.
            d,hit=self.query(ga,poses[i],gb,poses[j])
            self.last_stats['narrowphase_queries']=self.last_stats.get('narrowphase_queries',0)+1
            self.check_deadline()
            if contained is None:
                contained=not hit and self.containment_ambiguous(ga,transforms[i],gb,transforms[j],validated=True) if ambiguous else False
            reason=('COLLISION:'+a+':'+b if d<0 else
                    'SOLID_CONTAINMENT_COLLISION' if contained else
                    'COLLISION:'+a+':'+b if not zero_contact and (hit or (not direct and d<self.margin)) else '')
            if reason:
                self.failure(reason,sample,[a,b],d,self.category(a,b),hit)
                return dict(self.last_failure)
            remember_clear()
        return None

    def failure(self,reason,sample=-1,pair=None,distance=None,category=None,collision=False):
        self.last_reason=reason
        self.last_failure=dict(reason=reason,sample_index=sample,pair=pair,distance_m=distance,category=category,surface_collision=collision)
        return False

    @timed_collision_stage
    def validated_scene(self,scene,payload,*,reference_ns=None):
        if not isinstance(scene,dict):raise ValueError('COLLISION_STATE_SNAPSHOT_REQUIRED')
        # Candidate passes share the acquisition-time scene. Execution admission
        # always uses live time; source stamps and work deadlines are never renewed.
        now=time.monotonic_ns() if reference_ns is None else reference_ns
        if 'scene_snapshot' in scene:
            from .scene import validate_scene_data
            validate_scene_data(scene['scene_snapshot'],self.cfg,now,
                                scene['navigation_session_id'],scene['owner_session_id'])
        else:
            for key,ttl in [('legs_ns',self.cfg['limits'].get('legs_age_s',self.cfg['limits']['arm_age_s'])),('feedback_ns',self.cfg['limits']['arm_age_s']),('base_ns',self.cfg['limits']['sensor_age_s'])]:
                if type(scene.get(key)) is not int or not 0<=(now-scene[key])/1e9<=ttl:raise ValueError('COLLISION_STATE_STALE:'+key)
        if self.environment_enabled:
            if self.floor is None or self.cfg['collision'].get('environment_frame')!='map':raise ValueError('SURVEYED_MAP_ENVIRONMENT_MISSING')
            if not scene.get('scene_revision') or scene['scene_revision']!=self.cfg['collision'].get('scene_revision'):raise ValueError('COLLISION_SCENE_REVISION_MISMATCH')
        return rigid(scene['map_body'])

    def check(self,points,scene,payload=False,*,reference_ns=None):
        payload=bool(payload and self.payload_enabled)
        self.last_failure={}
        started=time.monotonic_ns()
        self._check_deadline_ns=None if self.check_timeout_s is None else started+int(self.check_timeout_s*1e9)
        self._profiling=True
        cpu_started=time.thread_time_ns()
        self.last_stats=dict(started_ns=started,timeout_s=self.check_timeout_s,samples=0,duplicate_samples=0,narrowphase_queries=0,broadphase_skipped=0)
        try:
            self.check_deadline()
            p=np.asarray(points,dtype=float)
            self.last_stats['input_points']=len(p) if p.ndim else 0
            if p.ndim!=2 or p.shape[1]!=7 or len(p)<2 or not np.isfinite(p).all():raise ValueError('INVALID_COLLISION_PATH')
            map_body=self.validated_scene(scene,payload,reference_ns=reference_ns)
            if payload and (self.payload_size is None or self.payload_xyz is None):raise ValueError('PAYLOAD_GEOMETRY_MISSING')
            rate=scene['command_rate_hz'];velocity=np.asarray(scene['velocity_limits'],dtype=float)
            if not np.isfinite(rate) or rate<=0 or velocity.shape!=(7,) or not np.isfinite(velocity).all() or np.any(velocity<=0):raise ValueError('COLLISION_VELOCITY_LIMITS_MISSING')
            if np.any(np.abs(np.diff(p,axis=0))*rate>velocity+1e-6):raise ValueError('COLLISION_PATH_VELOCITY_LIMIT')
            obstacles=[]
            if self.environment_enabled:
                floor=np.eye(4);floor[2,3]=float(self.floor)-.5;obstacles.append(('floor',self.fcl.Box(100,100,1),floor))
                for box in self.boxes:obstacles.append(('environment',self.fcl.Box(*box['size_m']),xyzrpy(box)))
            self._clear_cache={}
            sample=0;previous=None
            carried_shape=self.fcl.Box(*self.payload_size) if payload else None
            for start,end in zip(p[:-1],p[1:]):
                count=max(1,int(np.ceil(np.max(np.abs(end-start))/self.cfg['collision']['sample_max_joint_step_rad'])))
                # Most command intervals need no extra subdivision; avoid allocating
                # a linspace array without changing the checked grid or its indices.
                for alpha in ((0.,1.) if count==1 else np.linspace(0,1,count+1)):
                    self.check_deadline()
                    state=start+alpha*(end-start)
                    if previous is not None and np.array_equal(state,previous):
                        self.last_stats['duplicate_samples']+=1
                        sample+=1 # retain the original grid index in failure diagnostics
                        continue
                    previous=state.copy()
                    self.last_stats['samples']+=1
                    if self.last_stats['samples']%32==0:
                        time.sleep(0) # Yield the Python worker to ROS; no freshness/geometry bypass.
                        self.check_deadline()
                    self.last_stats['current_pair']=None
                    self.last_stats['sample_index']=sample
                    poses=self.placements(state,scene['legs'])
                    if self.self_failure(poses,sample):return False
                    world=[map_body@t for t in poses]
                    for i,link in enumerate(self.links):
                        if link not in self.arm_links:continue
                        for category,shape,pose in obstacles:
                            d,hit=self.query(self.geom.geometryObjects[i].geometry,world[i],shape,pose)
                            ambiguous=not hit and self.containment_ambiguous(self.geom.geometryObjects[i].geometry,world[i],shape,pose)
                            if hit or ambiguous or d<self.margin:return self.failure(('ENVIRONMENT_CONTAINMENT_UNRESOLVED:' if ambiguous and not hit and d>=self.margin else 'ENVIRONMENT_COLLISION:')+link,sample,[link,category],d,category,hit)
                    if payload:
                        tcp=map_body@self.data.oMf[self.model.getFrameId('End_link')].homogeneous
                        carried=tcp.copy();carried[:3,3]+=tcp[:3,:3]@np.asarray(self.payload_xyz)
                        shape=carried_shape
                        for i,link in enumerate(self.links):
                            d,hit=self.query(shape,carried,self.geom.geometryObjects[i].geometry,world[i])
                            ambiguous=not hit and self.containment_ambiguous(shape,carried,self.geom.geometryObjects[i].geometry,world[i])
                            if hit or ambiguous or d<self.margin:
                                reason='PAYLOAD_CONTACT_MODEL_MISSING' if link in ('gripper_left','gripper_right') else 'PAYLOAD_COLLISION:'+link
                                return self.failure(reason,sample,['payload',link],d,'attached_payload',hit)
                        for category,obstacle,pose in obstacles:
                            d,hit=self.query(shape,carried,obstacle,pose)
                            if hit or d<self.margin:return self.failure('PAYLOAD_ENVIRONMENT_COLLISION',sample,['payload',category],d,'attached_payload',hit)
                    sample+=1
            self.check_deadline()
            self.last_reason='';return True
        except CollisionCancelled as error:
            self.failure(str(error))
            raise
        except CollisionComputeTimeout:
            self.failure('COLLISION_COMPUTE_TIMEOUT')
            raise
        except (ValueError,RuntimeError,KeyError,TypeError,OverflowError) as e:return self.failure(str(e))
        finally:
            self._clear_cache=None
            self.last_stats['elapsed_s']=(time.monotonic_ns()-started)/1e9
            self.last_stats['thread_cpu_s']=(time.thread_time_ns()-cpu_started)/1e9
            self.last_stats['result_reason']=self.last_reason
            self._profiling=False
            self._check_deadline_ns=None
