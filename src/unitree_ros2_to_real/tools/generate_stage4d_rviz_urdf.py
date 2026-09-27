#!/usr/bin/env python3
"""Generate the RViz-only, install-space Go2 + RARS01 URDF derivative.

The authoritative topology is the generated combined URDF beside the MuJoCo
model.  This tool changes only visual mesh URIs and link frame names: joint
names, types, axes, origins and parent/child topology remain authoritative.
"""
import argparse
import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

GO2_VISUALS = {
    'base.dae': ('base_0.obj', 'base_1.obj', 'base_2.obj', 'base_3.obj', 'base_4.obj'),
    'hip.dae': ('hip_0.obj', 'hip_1.obj'),
    'thigh.dae': ('thigh_0.obj', 'thigh_1.obj'),
    'thigh_mirror.dae': ('thigh_mirror_0.obj', 'thigh_mirror_1.obj'),
    'calf.dae': ('calf_0.obj', 'calf_1.obj'),
    'calf_mirror.dae': ('calf_mirror_0.obj', 'calf_mirror_1.obj'),
    'foot.dae': ('foot.obj',),
}
MOVABLE = {
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
    'gripper_left_joint', 'gripper_right_joint',
}


def mesh_replacements(visual):
    mesh = visual.find('./geometry/mesh')
    if mesh is None:
        return [visual]
    filename = mesh.get('filename', '')
    basename = Path(filename).name
    if basename in GO2_VISUALS:
        visuals = []
        for obj in GO2_VISUALS[basename]:
            duplicate = copy.deepcopy(visual)
            duplicate.find('./geometry/mesh').set('filename', 'package://unitree_mujoco/assets/go2/' + obj)
            visuals.append(duplicate)
        return visuals
    if filename.startswith('../assets/rars01/'):
        visual = copy.deepcopy(visual)
        visual.find('./geometry/mesh').set('filename', 'package://unitree_mujoco/assets/rars01/' + basename)
        return [visual]
    raise ValueError(f'No deterministic RViz mesh mapping for {filename}')


def topology(root):
    links = {element.get('name') for element in root.findall('link')}
    joints = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        joints[joint.get('name')] = (joint.get('type'), parent, child)
    return links, joints


def generate(source, output):
    tree = ET.parse(source)
    root = tree.getroot()
    original_links, original_joints = topology(root)
    prefix = 'sim_visual_'
    rename = {link: prefix + link for link in original_links}
    for link in root.findall('link'):
        link.set('name', rename[link.get('name')])
        originals = list(link.findall('visual'))
        for visual in originals:
            link.remove(visual)
            for replacement in mesh_replacements(visual):
                link.append(replacement)
    for joint in root.findall('joint'):
        # This Gazebo-only attribute is unknown to standard URDF/RViz parsers.
        joint.attrib.pop('dont_collapse', None)
        joint.find('parent').set('link', rename[joint.find('parent').get('link')])
        joint.find('child').set('link', rename[joint.find('child').get('link')])
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    new_links, new_joints = topology(root)
    if {name.removeprefix(prefix) for name in new_links} != original_links:
        raise RuntimeError('link hierarchy changed while deriving RViz model')
    if set(new_joints) != set(original_joints):
        raise RuntimeError('joint set changed while deriving RViz model')
    for name, (joint_type, parent, child) in original_joints.items():
        new_type, new_parent, new_child = new_joints[name]
        if (joint_type, prefix + parent, prefix + child) != (new_type, new_parent, new_child):
            raise RuntimeError(f'joint topology changed: {name}')
    movable = {name for name, (kind, _, _) in new_joints.items() if kind in ('revolute', 'continuous', 'prismatic')}
    if movable != MOVABLE:
        raise RuntimeError(f'movable joint mismatch: expected {sorted(MOVABLE)}, got {sorted(movable)}')
    for mesh in root.findall('.//mesh'):
        if not mesh.get('filename', '').startswith('package://unitree_mujoco/assets/'):
            raise RuntimeError(f'non-install-space mesh URI: {mesh.get("filename")}')
    return len(movable)


def main():
    here = Path(__file__).resolve()
    workspace = here.parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=workspace / 'src/unitree_mujoco/unitree_robots/go2_rars01/generated/go2_arm_dynamic_train_obj.urdf')
    parser.add_argument('--output', type=Path, default=here.parents[1] / 'config/go2_rars01_stage4d_rviz.urdf')
    args = parser.parse_args()
    print(f'generated {args.output} ({generate(args.source, args.output)}/20 movable joints)')

if __name__ == '__main__':
    main()
