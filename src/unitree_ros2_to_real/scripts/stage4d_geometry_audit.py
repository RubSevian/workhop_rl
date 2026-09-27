#!/usr/bin/env python3
"""Reproducible Stage4D collision-geometry audit and optional walk recorder.

The tool only reads the authoritative MJCF and `/stage4d/joint_states`; it has
no publishers and cannot influence navigation.  `--record-seconds` captures
collision-geometry extents expressed in the base/vehicle frame.
"""
import argparse
import json
import math
import statistics
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

COLLISION_CLASSES = {"go2_collision", "go2_foot"}


def vec(text, n, default=None):
    if text is None:
        return list(default if default is not None else [0.0] * n)
    values = [float(x) for x in text.split()]
    if len(values) != n:
        raise ValueError(f"expected {n} values, got {values}: {text!r}")
    return values


def eye():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def rot_axis(axis, angle):
    x, y, z = axis
    n = math.sqrt(x*x + y*y + z*z)
    if not n:
        return eye()
    x, y, z = x/n, y/n, z/n
    c, s, t = math.cos(angle), math.sin(angle), 1.0-math.cos(angle)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]


def quat_rot(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if not n:
        return eye()
    w, x, y, z = w/n, x/n, y/n, z/n
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = (len(values)-1) * fraction
    lo, hi = int(math.floor(index)), int(math.ceil(index))
    return values[lo] if lo == hi else values[lo] + (values[hi]-values[lo]) * (index-lo)


class Geometry:
    def __init__(self, xml_path):
        self.xml_path = str(xml_path)
        self.root = ET.parse(xml_path).getroot()
        self.base = self.root.find('.//body[@name="base"]')
        if self.base is None:
            raise RuntimeError("MJCF has no body named 'base'")
        key = self.root.find('.//key[@name="home"]')
        if key is None:
            raise RuntimeError("MJCF has no home keyframe")
        values = vec(key.get('qpos'), len(key.get('qpos').split()))
        # base freejoint consumes 7 qpos entries; leg joints follow in MJCF order.
        names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
        ]
        self.home = dict(zip(names, values[7:19]))
        self.joint_names = names
        self.named = {body.get('name'): body for body in self.base.findall('.//body')}

    def collision_boxes(self, joint_values):
        """Return axis-aligned world/base-frame XY extents for all collision geoms."""
        boxes = []

        def walk(body, pos, rot):
            # A body transform is parent transform then its XML body pose and its
            # hinge rotation.  The Go2 legs each have one joint at zero local pos.
            body_pos = add(pos, mv(rot, vec(body.get('pos'), 3)))
            body_rot = mm(rot, quat_rot(vec(body.get('quat'), 4, [1, 0, 0, 0])))
            joint = body.find('joint')
            if joint is not None:
                angle = joint_values.get(joint.get('name'), 0.0)
                body_rot = mm(body_rot, rot_axis(vec(joint.get('axis'), 3), angle))
            for geom in body.findall('geom'):
                if geom.get('class') not in COLLISION_CLASSES:
                    continue
                gpos = add(body_pos, mv(body_rot, vec(geom.get('pos'), 3)))
                grot = mm(body_rot, quat_rot(vec(geom.get('quat'), 4, [1, 0, 0, 0])))
                shape = geom.get('type', 'sphere')
                size = vec(geom.get('size'), len(geom.get('size', '').split()))
                if shape == 'box':
                    half = size[:3]
                elif shape == 'cylinder':
                    half = [size[0], size[0], size[1]]
                elif shape == 'sphere':
                    half = [size[0], size[0], size[0]]
                else:
                    continue
                ext = [sum(abs(grot[i][j]) * half[j] for j in range(3)) for i in range(3)]
                boxes.append({'name': geom.get('name', geom.get('class', 'unnamed')),
                              'center': gpos, 'half': ext, 'shape': shape, 'body': body.get('name', '')})
            for child in body.findall('body'):
                walk(child, body_pos, body_rot)

        walk(self.base, [0.0, 0.0, 0.0], eye())
        return boxes

    @staticmethod
    def extent(boxes, predicate=lambda _: True):
        selected = [b for b in boxes if predicate(b)]
        if not selected:
            return None
        return {
            'min_x': min(b['center'][0]-b['half'][0] for b in selected),
            'max_x': max(b['center'][0]+b['half'][0] for b in selected),
            'min_y': min(b['center'][1]-b['half'][1] for b in selected),
            'max_y': max(b['center'][1]+b['half'][1] for b in selected),
        }

    def static_report(self):
        boxes = self.collision_boxes(self.home)
        central = next((b for b in boxes if b['name'] == 'base_collision'), None)
        if central is None:
            raise RuntimeError('base_collision missing')
        body = self.extent([central])
        rigid = self.extent(boxes, lambda b: not b['body'].startswith(('FL_', 'FR_', 'RL_', 'RR_')))
        leg = self.extent(boxes, lambda b: b['body'].startswith(('FL_', 'FR_', 'RL_', 'RR_')))
        feet = self.extent(boxes, lambda b: '_foot_collision' in b['name'])
        hips = {}
        for name in ('FL_hip', 'FR_hip', 'RL_hip', 'RR_hip'):
            pos = vec(self.named[name].get('pos'), 3)
            hips[name] = {'x': pos[0], 'y': pos[1]}
        return {
            'source_mjcf': self.xml_path,
            'home_joint_angles_rad': self.home,
            'base_body_box': body,
            'rigid_collision_envelope_including_head': rigid,
            'nominal_standing_feet': feet,
            'nominal_full_leg_envelope': leg,
            'hips': hips,
            'collision_geoms': boxes,
            'notes': [
                'base_body_box is only the named central base_collision box.',
                'nominal_full_leg_envelope is calculated from collision geoms at MJCF home qpos.',
                'runtime recorder below must be used before assigning a swing/sway margin.'
            ]
        }


def summarise_samples(samples):
    if not samples:
        return {'sample_count': 0}
    keys = ('min_x', 'max_x', 'min_y', 'max_y')
    output = {'sample_count': len(samples)}
    for key in keys:
        vals = [s[key] for s in samples]
        # "p95 extent" means outward absolute extent, not a signed percentile.
        output[f'{key}_max_observed'] = min(vals) if key.startswith('min_') else max(vals)
    widths = [s['max_y']-s['min_y'] for s in samples]
    lengths = [s['max_x']-s['min_x'] for s in samples]
    output.update({
        'width_p95': percentile(widths, .95), 'width_max': max(widths),
        'length_p95': percentile(lengths, .95), 'length_max': max(lengths),
        'left_extent_p95': percentile([-s['min_y'] for s in samples], .95),
        'right_extent_p95': percentile([s['max_y'] for s in samples], .95),
        'front_extent_p95': percentile([s['max_x'] for s in samples], .95),
        'rear_extent_p95': percentile([-s['min_x'] for s in samples], .95),
    })
    return output


def scene_report(scene):
    root = ET.parse(scene).getroot()
    geoms = []
    for geom in root.findall('.//geom'):
        name = geom.get('name', '')
        if not name.startswith('stage4d_'):
            continue
        pos = vec(geom.get('pos'), 3); size = vec(geom.get('size'), len(geom.get('size', '').split()))
        geoms.append({'name': name, 'type': geom.get('type', 'sphere'), 'pos': pos, 'size': size})
    pairs = []
    by_name = {g['name']: g for g in geoms}
    for number in ('01', '02', '03', '04', '05'):
        left = by_name.get(f'stage4d_corridor_left_{number}')
        right = by_name.get(f'stage4d_corridor_right_{number}')
        if not left or not right:
            continue
        # Actual closest free separation for axis-aligned boxes.  We report the
        # centre-axis separation as well because these legacy labels do not
        # prove that the two geoms form a traversable lane.
        dx = abs(left['pos'][0]-right['pos'][0]) - left['size'][0]-right['size'][0]
        dy = abs(left['pos'][1]-right['pos'][1]) - left['size'][1]-right['size'][1]
        pairs.append({'pair': number, 'free_x': dx, 'free_y': dy,
                      'centre_distance_xy': math.hypot(left['pos'][0]-right['pos'][0], left['pos'][1]-right['pos'][1]),
                      'interpretation': 'Named pair shares y; it is not a documented lateral corridor. Do not use as the controlled gap suite.'})
    return {'source_scene': str(scene), 'named_stage4d_geoms': geoms, 'named_corridor_pairs': pairs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', default=None, help='authoritative go2_rars01.xml')
    parser.add_argument('--scene', default=None, help='Stage4D MuJoCo scene XML')
    parser.add_argument('--output', required=True, help='JSON output path')
    parser.add_argument('--record-seconds', type=float, default=0.0, help='record /stage4d/joint_states for this duration')
    args, unknown = parser.parse_known_args()
    if unknown:
        parser.error(f'unrecognized arguments: {unknown}')
    root = Path(__file__).resolve().parents[2]
    xml = Path(args.xml) if args.xml else root / 'unitree_mujoco/unitree_robots/go2_rars01/go2_rars01.xml'
    geometry = Geometry(xml)
    report = {'static': geometry.static_report()}
    if args.scene:
        report['scene'] = scene_report(Path(args.scene))
    if args.record_seconds > 0.0:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise SystemExit(f'ROS 2 Python environment required for recording: {exc}')
        rclpy.init()
        node = Node('stage4d_collision_envelope_recorder')
        samples = {'full_collision': [], 'leg_collision': [], 'foot_collision': []}
        def callback(msg):
            joints = dict(zip(msg.name, msg.position))
            if not all(name in joints for name in geometry.joint_names):
                return
            boxes = geometry.collision_boxes(joints)
            snapshots = {
                'full_collision': Geometry.extent(boxes),
                'leg_collision': Geometry.extent(boxes, lambda b: b['body'].startswith(('FL_', 'FR_', 'RL_', 'RR_'))),
                'foot_collision': Geometry.extent(boxes, lambda b: '_foot_collision' in b['name']),
            }
            for key, ext in snapshots.items():
                if ext:
                    samples[key].append(ext)
        sub = node.create_subscription(JointState, '/stage4d/joint_states', callback, 20)
        end = time.monotonic() + args.record_seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub); node.destroy_node(); rclpy.shutdown()
        report['runtime_collision_envelope_vehicle_frame'] = {key: summarise_samples(value) for key, value in samples.items()}
        report['runtime_input'] = '/stage4d/joint_states'
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'output': args.output, 'static': report['static']['nominal_full_leg_envelope'],
                      'runtime_samples': report.get('runtime_collision_envelope_vehicle_frame', {}).get('leg_collision', {}).get('sample_count', 0)}, indent=2))

if __name__ == '__main__':
    main()
