#!/usr/bin/env python3
"""Recorder only: Stage4D never publishes a route or navigation_active."""
import argparse, json, math, time, signal
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
class Evaluator(Node):
 def __init__(self, report):
  super().__init__('stage4d_full_navigation_evaluator'); self.report=report; self.goal=None; self.latest={}; self.trace=[]; self.t0=time.time()
  self.create_subscription(PointStamped,'/goal_point',lambda m:self.set('goal',m),10)
  for typ,topic,key in [(Odometry,'/state_estimation','odom'),(Odometry,'/sim/ground_truth_odom','gt'),(Path,'/path','path'),(PointStamped,'/way_point','waypoint'),(TwistStamped,'/cmd_vel','cmd'),(Bool,'/navigation_active','navigation_active'),(Bool,'/far_reach_goal_status','far_reach_goal'),(Float32,'/planning_time','planning_time'),(Float32,'/runtime','runtime'),(Bool,'/stage4d/navigation_ready','navigation_ready')]: self.create_subscription(typ,topic,lambda m,k=key:self.set(k,m),10)
  self.create_timer(0.1,self.sample); self.create_service(Trigger,'/stage4d/save_report',self.save)
 def set(self,k,m):
  if k in ('goal','waypoint'): v={'x':m.point.x,'y':m.point.y,'frame':m.header.frame_id}
  elif k in ('odom','gt'): v={'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'z':m.pose.pose.position.z,'frame':m.header.frame_id}
  elif k=='path': v={'poses':len(m.poses),'frame':m.header.frame_id}
  elif k=='cmd': v={'vx':m.twist.linear.x,'vy':m.twist.linear.y,'wz':m.twist.angular.z}
  elif hasattr(m,'data'): v=m.data
  else: v=float(m.data)
  self.latest[k]=v
 def sample(self): self.trace.append({'t':time.time()-self.t0,**self.latest}); self.trace=self.trace[-20000:]
 def save(self,req,res):
  out={'schema':'stage4d-full-navigation-v1','goal':self.latest.get('goal'),'latest':self.latest,'samples':len(self.trace),'trace':self.trace,'manual_status':'TO_BE_FILLED_USER'}
  with open(self.report,'w') as f: json.dump(out,f,indent=2)
  res.success=True; res.message=self.report; return res
def main():
 p=argparse.ArgumentParser(); p.add_argument('--report',default='/tmp/stage4d_full_navigation_report.json'); a,_=p.parse_known_args(); rclpy.init(); n=Evaluator(a.report); rclpy.spin(n)
if __name__=='__main__': main()
