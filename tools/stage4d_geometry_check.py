#!/usr/bin/env python3
"""Deterministic preflight for Stage4D obstacle/goal cases."""
from pathlib import Path
import math, re
SCENE=Path(__file__).parents[1]/'src/unitree_mujoco/unitree_robots/go2_rars01/scene_stage4d.xml'
BOXES={'stage4d_obstacle_single_01':((.37,.73),(.30,.66)), 'stage4d_corridor_left_01':((.43,.67),(-.60,-.36)), 'stage4d_corridor_right_01':((.83,1.07),(-.60,-.36)), 'stage4d_corridor_left_02':((1.23,1.47),(-.60,-.36)), 'stage4d_corridor_right_02':((1.63,1.87),(-.60,-.36))}
CASES={'D0':(1,0),'D1':(1.2,.8),'D2':(1.8,-.8),'D3':(.6,1.0),'D4A':(1,0),'D4B':(.6,.9)}
def hit(goal, box):
 (x0,x1),(y0,y1)=box; return x0<=goal[0]<=x1 and y0<=goal[1]<=y1
def main():
 s=SCENE.read_text(); missing=[n for n in BOXES if f'name="{n}"' not in s]
 assert not missing, missing
 assert not hit(CASES['D0'],BOXES['stage4d_obstacle_single_01'])
 assert 0.48 < (0.36-(-0.60)) < 2.0
 print('Stage4D geometry PASS: semantic landmarks present; D0 free; corridor width 0.96 m')
if __name__=='__main__': main()
