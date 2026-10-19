"""Validated SE(3), stamped feedback and eye-in-hand transforms. No SDK."""
import numpy as np
from scipy.spatial.transform import Rotation

def rigid(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4,4) or not np.isfinite(t).all() or not np.allclose(t[3], [0,0,0,1], atol=1e-8):
        raise ValueError('INVALID_TRANSFORM')
    r = t[:3,:3]
    if not np.allclose(r.T@r, np.eye(3), atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
        raise ValueError('INVALID_ROTATION')
    return t

def pose_matrix(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1)>1e-3:
        raise ValueError('INVALID_POSE')
    t = np.eye(4);t[:3,3] = p[:3];t[:3,:3] = Rotation.from_quat(p[3:]).as_matrix()
    return t

def pose_distance(a, b):
    a, b = pose_matrix(a), pose_matrix(b)
    return float(np.linalg.norm(a[:3,3]-b[:3,3])), float(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())

def xyzrpy(value):
    if not isinstance(value, dict):
        raise ValueError('VEHICLE_MOUNT_MISSING')
    t = np.eye(4)
    t[:3,3] = value['xyz_m'];t[:3,:3] = Rotation.from_euler('xyz', value['rpy_rad']).as_matrix()
    return rigid(t)

def fixed_joint(urdf,name):
    import xml.etree.ElementTree as ET
    joint=next((j for j in ET.parse(urdf).getroot().findall('joint') if j.get('name')==name),None)
    if joint is None or joint.get('type')!='fixed':raise ValueError('FIXED_MOUNT_JOINT_MISSING')
    origin=joint.find('origin')
    if origin is None:raise ValueError('FIXED_MOUNT_ORIGIN_MISSING')
    return xyzrpy(dict(xyz_m=[float(x) for x in origin.get('xyz','0 0 0').split()],rpy_rad=[float(x) for x in origin.get('rpy','0 0 0').split()]))

class EyeInHand:
    def __init__(self, path, fk, vehicle_mount, max_age_s, mount_arm, frame_chain=None, compensation=None):
        data = np.load(path, allow_pickle=False)
        if str(data['mode'][0]) != 'eye_in_hand':
            raise ValueError('WRONG_HAND_EYE_MODE')
        self.end_camera, self.fk = rigid(data['T_result']), fk
        self.vehicle_mount = xyzrpy(vehicle_mount)
        # Existing, unchanged URDF fixed joint arm_mount_to_base_link.
        self.mount_arm = rigid(mount_arm)
        self.compensation=np.eye(4)
        self.compensation[:3,3]=[float((compensation or {}).get(axis,0.)) for axis in 'xyz']
        rigid(self.compensation)
        self.frame_chain=frame_chain
        self.max_age_ns = int(max_age_s*1e9)
        self.odom_max_age_ns = self.max_age_ns

    def chain(self, rgbd_ns, feedback_ns, joints, odom_ns, odom_vehicle):
        if not (0 <= rgbd_ns-feedback_ns <= self.max_age_ns and 0 <= rgbd_ns-odom_ns <= self.odom_max_age_ns):
            raise ValueError('RGBD_TF_FEEDBACK_STALE')
        arm_camera = self.compensation @ rigid(self.fk.forward(joints)) @ self.end_camera
        odom_arm = (self.frame_chain.arm(odom_vehicle) if self.frame_chain is not None
                    else rigid(odom_vehicle) @ self.vehicle_mount @ self.mount_arm)
        return rigid(arm_camera), rigid(odom_arm)

    @staticmethod
    def anchor(arm_camera, odom_arm, camera_grasp):
        return rigid(odom_arm) @ rigid(arm_camera) @ rigid(camera_grasp)

    @staticmethod
    def rebase(odom_arm_now, odom_grasp):
        return np.linalg.inv(rigid(odom_arm_now)) @ rigid(odom_grasp)
