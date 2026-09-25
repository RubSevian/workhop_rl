#!/usr/bin/env python3
"""Deterministically compile the canonical Go2+RARS01 URDF into a MuJoCo model."""
import argparse, hashlib, shutil, struct, subprocess, tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
import mujoco

LEG_DDS = ("FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint")
ARM = tuple("joint{}".format(i) for i in range(1, 7))
GRIP = ("gripper_left_joint", "gripper_right_joint")

def digest(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for block in iter(lambda:f.read(1<<20),b''): h.update(block)
 return h.hexdigest()

def attrs(node, key): return tuple(float(v) for v in node.get(key).split())
def joint_map(root): return {j.get('name'):j for j in root.findall('joint')}
def effort(j): return float(j.find('limit').get('effort'))
def clean_urdf(source, description_root, temp):
 tree=ET.parse(source); root=tree.getroot(); joints=joint_map(root)
 for name in ARM + GRIP:
  j=joints.get(name)
  if j is None or j.get('type') not in ('revolute','prismatic'): raise RuntimeError('missing movable '+name)
 for visual in root.findall('.//visual'):
  mesh=visual.find('./geometry/mesh')
  if mesh is not None and mesh.get('filename','').lower().endswith('.dae'):
   parent=next(n for n in root.iter() if visual in list(n)); parent.remove(visual)
 for mesh in list(root.findall('.//mesh')):
  file=mesh.get('filename','')
  if not file.startswith('package://rars01_description/'): continue
  relative=file.split('package://rars01_description/',1)[1]; source_mesh=description_root/relative
  if source_mesh.name == 'arm_mount_link.STL':
   geometry=next(n for n in root.iter() if mesh in list(n)); geometry.remove(mesh)
   # MuJoCo refuses this canonical 431649-face mesh (limit 200000). Keep a
   # documented collision approximation; every other RARS01 mesh is retained.
   ET.SubElement(geometry,'box',{'size':'0.22 0.16 0.10'}); continue
  shutil.copy2(source_mesh, temp/source_mesh.name); mesh.set('filename',source_mesh.name)
 muj=ET.SubElement(root,'mujoco'); ET.SubElement(muj,'compiler',{'fusestatic':'false','discardvisual':'false'})
 path=temp/'combined.urdf'; tree.write(path,encoding='utf-8',xml_declaration=True)
 return joints,path

def split_binary_stl(source, destination, faces_per_part=150000):
    raw = source.read_bytes(); faces = struct.unpack('<I', raw[80:84])[0]
    for part, start in enumerate(range(0, faces, faces_per_part)):
        count = min(faces_per_part, faces - start); output = destination / ('arm_mount_link_part%d.STL' % part)
        output.write_bytes(raw[:80] + struct.pack('<I', count) + raw[84 + start * 50:84 + (start + count) * 50])

def add_runtime_layers(xml_path, joints, assets):
 tree=ET.parse(xml_path); root=tree.getroot(); root.insert(1,ET.Element('option',{'timestep':'0.005','cone':'elliptic','impratio':'100'}))
 # Restore the complete canonical mount as visual-only chunks. MuJoCo's face
 # limit applies per mesh, so three source-preserving chunks retain all detail.
 asset = root.find('asset')
 for mesh in sorted(assets.glob('arm_mount_link_part*.STL')):
  name=mesh.stem; ET.SubElement(asset,'mesh',{'name':name,'file':'assets/rars01/'+mesh.name})
 mount=next(b for b in root.findall('.//body') if b.get('name')=='arm_mount_link')
 for mesh in sorted(assets.glob('arm_mount_link_part*.STL')):
  ET.SubElement(mount,'geom',{'type':'mesh','mesh':mesh.stem,'contype':'0','conaffinity':'0','group':'2'})
 base=next(b for b in root.findall('.//body') if b.get('name')=='base'); base.insert(0,ET.Element('freejoint',{'name':'base_freejoint'})); ET.SubElement(base,'site',{'name':'imu','pos':'-0.02557 0 0.04232'})
 actuator=ET.SubElement(root,'actuator')
 for index,name in enumerate(LEG_DDS):
  limit=effort(joints[name]); ET.SubElement(actuator,'motor',{'name':name.replace('_joint','_motor'),'joint':name,'ctrlrange':'{} {}'.format(-limit,limit)})
 for name in ARM+GRIP:
  limit=effort(joints[name]); ET.SubElement(actuator,'motor',{'name':name.replace('_joint','_motor'),'joint':name,'ctrlrange':'{} {}'.format(-limit,limit)})
 sensor=ET.SubElement(root,'sensor')
 for suffix,tag in (('pos','jointpos'),('vel','jointvel'),('torque','jointactuatorfrc')):
  for name in LEG_DDS: ET.SubElement(sensor,tag,{'name':name+'_'+suffix,'joint':name})
 ET.SubElement(sensor,'framequat',{'name':'imu_quat','objtype':'site','objname':'imu'})
 ET.SubElement(sensor,'gyro',{'name':'imu_gyro','site':'imu'}); ET.SubElement(sensor,'accelerometer',{'name':'imu_acc','site':'imu'})
 ET.SubElement(sensor,'framepos',{'name':'frame_pos','objtype':'site','objname':'imu'}); ET.SubElement(sensor,'framelinvel',{'name':'frame_vel','objtype':'site','objname':'imu'})
 for suffix,tag in (('pos','jointpos'),('vel','jointvel'),('torque','jointactuatorfrc')):
  for name in ARM+GRIP: ET.SubElement(sensor,tag,{'name':name+'_'+suffix,'joint':name})
 keyframe=ET.SubElement(root,'keyframe'); q=[0,0,0.4,1,0,0,0, .1,.8,-1.5,-.1,.8,-1.5,.1,.8,-1.5,-.1,.8,-1.5]+[0]*8
 ET.SubElement(keyframe,'key',{'name':'home','qpos':' '.join(str(v) for v in q),'ctrl':' '.join('0' for _ in range(20))})
 ET.indent(tree,space='  '); tree.write(xml_path,encoding='utf-8',xml_declaration=True)

def main():
 parser=argparse.ArgumentParser(); parser.add_argument('--description-root',type=Path,required=True); parser.add_argument('--output-root',type=Path,required=True); args=parser.parse_args()
 source=args.description_root/'urdf/go2_arm_dynamic_base.urdf'; out=args.output_root; out.mkdir(parents=True,exist_ok=True); assets=out/'assets/rars01'; assets.mkdir(parents=True,exist_ok=True)
 with tempfile.TemporaryDirectory() as tmp_string:
  tmp=Path(tmp_string); joints,urdf=clean_urdf(source,args.description_root,tmp); model=mujoco.MjModel.from_xml_path(str(urdf)); raw=tmp/'raw.xml'; mujoco.mj_saveLastXML(str(raw),model)
  text=raw.read_text();
  for mesh in tmp.glob('*.STL'):
   shutil.copy2(mesh,assets/mesh.name); text=text.replace('file="{}"'.format(mesh.name),'file="assets/rars01/{}"'.format(mesh.name))
  split_binary_stl(args.description_root/'meshes/arm_mount_link.STL', assets)
  xml=out/'go2_rars01.xml'; xml.write_text(text); add_runtime_layers(xml,joints,assets)
 # Compile final output before publishing manifest.
 final=mujoco.MjModel.from_xml_path(str(xml));
 commit=subprocess.check_output(['git','-C',str(args.description_root),'rev-parse','HEAD'],text=True).strip(); stock=Path(__file__).resolve().parents[1]/'unitree_robots/go2/go2.xml'
 (out/'SOURCE_MANIFEST.yaml').write_text('''model:\n  name: go2_rars01\nsources:\n  rars01_description:\n    repository: "https://github.com/RubSevian/rars01_description.git"\n    branch: "go2_arm"\n    commit: "{}"\n    urdf: "urdf/go2_arm_dynamic_base.urdf"\n    urdf_sha256: "{}"\n  stock_unitree_mujoco:\n    repository: "https://github.com/RubSevian/workhop_rl.git"\n    branch: "ros2_go2_rars01_sim"\n    source_model: "src/unitree_mujoco/unitree_robots/go2/go2.xml"\n    source_model_sha256: "{}"\nruntime:\n  timestep: 0.005\n  leg_actuators: 12\n  arm_actuators: 6\n  gripper_actuators: 2\n  mount_collision_approximation: "arm_mount_link collision uses 0.22x0.16x0.10 box; its full 431649-face canonical STL is retained as three visual-only chunks"\ngenerated:\n  file: "go2_rars01.xml"\n  sha256: "{}"\n'''.format(commit,digest(source),digest(stock),digest(xml)))
 print('generated {} (nq={}, nv={}, nu={})'.format(xml,final.nq,final.nv,final.nu))
if __name__=='__main__': main()
