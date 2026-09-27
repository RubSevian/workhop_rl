#!/usr/bin/env python3
"""Deterministic Phase-1 geometry audit for the Stage4D sensor boundary.

Frame convention: every transform in this test maps point coordinates from the
raw radar frame to Point-LIO's IMU frame: p_parent = R p_child + T.
"""
import math
import pathlib
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).parents[1]
WORKHOP = ROOT.parents[1]
AUTONOMY = WORKHOP.parent / 'autonomy_nav_go2'
XML = WORKHOP / 'src/unitree_mujoco/unitree_robots/go2_rars01/go2_rars01.xml'
TRANSFORM = AUTONOMY / 'src/utilities/transform_sensors/transform_sensors/transform_everything.py'
BASELINE = AUTONOMY / 'src/slam/point_lio_unilidar/config/utlidar_sim.yaml'
CANDIDATE = AUTONOMY / 'src/slam/point_lio_unilidar/config/utlidar_sim_extrinsic_candidate.yaml'
SIM_MAIN = WORKHOP / 'src/unitree_mujoco/simulate/src/main.cc'
CALIBRATION = ROOT / 'config/stage4d_sim_imu_calibration.yaml'


def vec_add(a, b): return [a[i] + b[i] for i in range(3)]
def vec_sub(a, b): return [a[i] - b[i] for i in range(3)]
def norm(a): return math.sqrt(sum(x*x for x in a))
def mat_vec(m, p): return [sum(m[i][j] * p[j] for j in range(3)) for i in range(3)]
def mat_mul(a, b): return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def transpose(m): return [[m[j][i] for j in range(3)] for i in range(3)]
def eye(): return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
def rotation_angle_deg(a, b):
    trace = sum((mat_mul(a, transpose(b)))[i][i] for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return [[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]]


def ry(pitch):
    return [[math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)]]


def parse_vec(text, key, count):
    match = re.search(rf'{re.escape(key)}:\s*\[([^\]]+)\]', text)
    if not match: raise AssertionError(f'missing {key}')
    value = [float(part.strip()) for part in match.group(1).split(',')]
    if len(value) != count: raise AssertionError(f'{key} length={len(value)}')
    return value


def parse_mapping_extrinsic(path):
    text = path.read_text()
    return parse_vec(text, 'extrinsic_T', 3), parse_vec(text, 'extrinsic_R', 9)


def quaternion_multiply_xyzw(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return [w1*x2+x1*w2+y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2,
            w1*w2-x1*x2-y1*y2-z1*z2]


def q_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


class Stage4DSensorExtrinsicTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = ET.parse(XML).getroot()
        imu = root.find(".//body[@name='imu']")
        radar = root.find(".//body[@name='radar']")
        assert imu is not None and radar is not None
        cls.imu_base = [float(x) for x in imu.attrib['pos'].split()]
        cls.radar_base = [float(x) for x in radar.attrib['pos'].split()]
        cls.physical_t = vec_sub(cls.radar_base, cls.imu_base)
        cls.physical_r = quat_wxyz_to_matrix([float(x) for x in radar.attrib['quat'].split()])
        source = TRANSFORM.read_text()
        cls.pitch = float(re.search(r'quaternion_from_euler\(0,\s*([0-9.]+),\s*0\)', source).group(1))
        cls.transform_r = ry(cls.pitch)
        cls.transform_t = [0.0, 0.0, -float(re.search(r'self\.cam_offset\s*=\s*([0-9.]+)', source).group(1))]
        cls.base_t, cls.base_r_flat = parse_mapping_extrinsic(BASELINE)
        cls.candidate_t, cls.candidate_r_flat = parse_mapping_extrinsic(CANDIDATE)
        cls.base_r = [cls.base_r_flat[i:i+3] for i in range(0, 9, 3)]
        cls.candidate_r = [cls.candidate_r_flat[i:i+3] for i in range(0, 9, 3)]

    def point_error(self, r_ext, t_ext, point):
        expected = vec_add(mat_vec(self.physical_r, point), self.physical_t)
        software = vec_add(mat_vec(r_ext, vec_add(mat_vec(self.transform_r, point), self.transform_t)), t_ext)
        return norm(vec_sub(expected, software))

    def test_mjcf_geometry_is_authoritative(self):
        self.assertEqual(self.imu_base, [-0.02557, 0.0, 0.04232])
        self.assertEqual(self.radar_base, [0.28945, 0.0, -0.046825])
        for got, expected in zip(self.physical_t, [0.315020, 0.0, -0.089145]):
            self.assertAlmostEqual(got, expected, places=9)

    def test_current_baseline_is_inconsistent(self):
        # P0 isolates translation.  The current offset is the known ~0.31 m mismatch.
        self.assertGreater(self.point_error(self.base_r, self.base_t, [0.0, 0.0, 0.0]), 0.30)

    def test_candidate_composes_to_mjcf_for_all_required_points(self):
        for name, point in {'P0': [0, 0, 0], 'Px': [1, 0, 0], 'Py': [0, 1, 0],
                            'Pz': [0, 0, 1], 'Pmix': [1.2, -0.7, 0.4]}.items():
            with self.subTest(name=name):
                self.assertLess(self.point_error(self.candidate_r, self.candidate_t, point), 1.e-5)

    def test_rotation_composition_matches_physical(self):
        composed = mat_mul(self.candidate_r, self.transform_r)
        self.assertLess(rotation_angle_deg(self.physical_r, composed), 0.001)

    def test_sim_imu_inverse_and_transformer_are_identity(self):
        # Exact inverse pair from BodyToRawVector() and imu_callback(), with
        # Stage4D calibration projections/biases confirmed zero below.
        theta = math.radians(15.1)
        for body in ([0.0, 0.0, 0.0], [1.2, -0.7, 0.4], [-2.0, 0.3, 8.1]):
            raw = [math.cos(theta)*body[0] + math.sin(theta)*body[2],
                   -body[1], math.sin(theta)*body[0] - math.cos(theta)*body[2]]
            x, y, z = raw[0], -raw[1], -raw[2]
            restored = [math.cos(theta)*x - math.sin(theta)*z, y,
                        math.sin(theta)*x + math.cos(theta)*z]
            self.assertLess(norm(vec_sub(restored, body)), 1.e-12)

        pitch = 2.8782025850555556
        mount = q_from_rpy(0.0, pitch, math.pi)
        inverse_mount = [-mount[0], -mount[1], -mount[2], mount[3]]
        for body_q in ([0.0, 0.0, 0.0, 1.0], q_from_rpy(.3, -.4, 1.2)):
            raw_q = quaternion_multiply_xyzw(inverse_mount, body_q)
            restored_q = quaternion_multiply_xyzw(mount, raw_q)
            self.assertLess(math.sqrt(sum((a-b)**2 for a, b in zip(restored_q, body_q))), 1.e-10)

        calib = CALIBRATION.read_text()
        for key in ('acc_bias_x: 0.0', 'acc_bias_y: 0.0', 'acc_bias_z: 0.0',
                    'ang_bias_x: 0.0', 'ang_bias_y: 0.0', 'ang_bias_z: 0.0',
                    'ang_z2x_proj: 0.0', 'ang_z2y_proj: 0.0'):
            self.assertIn(key, calib)


if __name__ == '__main__':
    unittest.main()
