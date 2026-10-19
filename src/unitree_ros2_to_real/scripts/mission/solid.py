"""Triangle-surface components and parity containment, without AABB admission.

Only watertight components define a solid. Open components retain surface
collision semantics. Used after Coal has excluded surface intersections.
"""
import numpy as np
import hashlib
import os
from pathlib import Path
import tempfile
import zipfile
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

class MeshSolid:
    @classmethod
    def cached(cls, shape, directory, mesh_path, scale, version):
        """Cache immutable topology only; never cache a collision decision.

        Content hashes invalidate replaced meshes and implementation changes.
        NPZ contains numeric arrays only; no pickle/code deserialization.
        """
        source=Path(mesh_path)
        if not directory or not source.is_file():return cls(shape),False
        digest=hashlib.sha256(Path(__file__).read_bytes())
        digest.update(source.read_bytes())
        digest.update(np.asarray(scale,dtype=np.float64).tobytes())
        digest.update((str(version)+np.__version__).encode())
        path=Path(directory)/(digest.hexdigest()+'.npz')
        try:
            with np.load(path,allow_pickle=False) as data:
                result=cls.__new__(cls)
                result.representatives=data['representatives']
                result.open_components=int(data['open_components'])
                count=int(data['closed_count'])
                if count<0 or count>len(result.representatives) or result.open_components<0:raise ValueError('cache counts')
                result.closed=[data['closed_'+str(i)] for i in range(count)]
                if result.representatives.ndim!=2 or result.representatives.shape[1]!=3:raise ValueError('cache representatives')
                if not np.isfinite(result.representatives).all():raise ValueError('cache finite')
                for triangles in result.closed:
                    if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not len(triangles) or not np.isfinite(triangles).all():raise ValueError('cache triangles')
            result._prepare_rays()
            return result,True
        except (OSError,ValueError,KeyError,EOFError,zipfile.BadZipFile):
            pass
        result=cls(shape)
        arrays=dict(representatives=result.representatives,open_components=result.open_components,closed_count=len(result.closed))
        arrays.update({'closed_'+str(i):t for i,t in enumerate(result.closed)})
        temporary=None
        try:
            path.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=path.parent,suffix='.npz',delete=False) as stream:
                temporary=stream.name
                np.savez(stream,**arrays)
            os.replace(temporary,path)
        except OSError:
            pass # Cache failure never prevents a freshly computed geometry.
        finally:
            if temporary and os.path.exists(temporary):os.unlink(temporary)
        return result,False

    def __init__(self,shape):
        vertices=np.asarray(shape.vertices(),dtype=float)
        if not shape.num_tris or vertices.ndim!=2 or vertices.shape[1]!=3 or not np.isfinite(vertices).all():
            raise ValueError('SOLID_MESH_INVALID')
        faces=np.array([[shape.tri_indices(i)[j] for j in range(3)]
                        for i in range(shape.num_tris)],dtype=int)
        # STL duplicates vertices per triangle: weld exact coordinates only.
        vertices,inverse=np.unique(vertices,axis=0,return_inverse=True)
        faces=inverse[faces]
        edges=np.sort(np.concatenate((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]])),axis=1)
        graph=coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(len(vertices),len(vertices)))
        count,labels=connected_components(graph,directed=False)
        self.representatives=[];self.closed=[];self.open_components=0
        for component in range(count):
            f=faces[labels[faces[:,0]]==component]
            if not len(f):continue
            self.representatives.append(vertices[f[0,0]])
            e=np.sort(np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]])),axis=1)
            _,counts=np.unique(e,axis=0,return_counts=True)
            if np.all(counts==2):self.closed.append(vertices[f])
            else:self.open_components+=1
        self.representatives=np.asarray(self.representatives)
        self._prepare_rays()

    def _prepare_rays(self):
        self.rays=[];bounds=[]
        for triangles in self.closed:
            bounds.append((triangles.min(axis=(0,1)),triangles.max(axis=(0,1))))
            # Morton-order triangle blocks form a compact ray broad phase.
            centres=triangles.mean(axis=1);lo=centres.min(axis=0)
            scale=np.maximum(centres.max(axis=0)-lo,1e-15)
            xyz=np.minimum(1023,((centres-lo)/scale*1023).astype(np.uint64))
            code=np.zeros(len(xyz),dtype=np.uint64)
            for bit in range(10):
                for axis in range(3):code|=((xyz[:,axis]>>bit)&1)<<(3*bit+axis)
            triangles=triangles[np.argsort(code)]
            starts=np.arange(0,len(triangles),64)
            block_lo=np.minimum.reduceat(triangles.min(axis=1),starts)
            block_hi=np.maximum.reduceat(triangles.max(axis=1),starts)
            a=triangles[:,0];e1=triangles[:,1]-a;e2=triangles[:,2]-a
            directions=np.array([[1.,.3713906763541037,.6947465906068658],
                                 [.219873512,1.,.53719327],[.613517,.297183,1.]])
            prepared=[]
            for direction in directions:
                h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h)
                valid=np.abs(det)>np.finfo(float).eps*np.linalg.norm(e1,axis=1)*np.linalg.norm(e2,axis=1)*32
                inv=np.zeros_like(det);inv[valid]=1/det[valid]
                prepared.append((direction,h,inv,valid))
            self.rays.append((a,e1,e2,prepared,block_lo,block_hi))

        self.bounds=np.asarray(bounds).reshape(-1,2,3)

    def contains(self,point):
        # Even/odd occupancy across closed shells also preserves internal cavities.
        inside=False
        candidates=np.flatnonzero(np.all(point>=self.bounds[:,0],axis=1)&np.all(point<=self.bounds[:,1],axis=1))
        for index in candidates:
            a,e1,e2,rays,block_lo,block_hi=self.rays[index]
            votes=[]
            for direction,h,inv,valid in rays:
                near=(block_lo-point)/direction;far=(block_hi-point)/direction
                selected=np.flatnonzero(np.max(near,axis=1)<=np.maximum(0.,np.min(far,axis=1)))
                indices=(selected[:,None]*64+np.arange(64)).ravel()
                indices=indices[indices<len(a)]
                s=point-a[indices];q=np.cross(s,e1[indices]);reciprocal=inv[indices]
                u=np.einsum('ij,ij->i',s,h[indices])*reciprocal
                v=np.einsum('ij,j->i',q,direction)*reciprocal
                t=np.einsum('ij,ij->i',e2[indices],q)*reciprocal
                forward=valid[indices] & (t>0) & (u>=0) & (v>=0) & (u+v<=1)
                # Ray through an edge/vertex is ambiguous; try another direction.
                if np.any(forward & ((u<1e-10)|(v<1e-10)|(1-u-v<1e-10))):continue
                votes.append(bool(np.count_nonzero(forward)%2))
            if not votes or any(v!=votes[0] for v in votes):
                raise ValueError('SOLID_RAY_CLASSIFICATION_UNRESOLVED')
            inside ^= votes[0]
        return inside
