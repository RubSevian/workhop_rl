#!/usr/bin/env python3
"""Record Stage4D FAR-to-localPlanner hand-off into JSON and CSV."""
import argparse,csv,json,os,time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped,TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
class Trace(Node):
 def __init__(self,out):
  super().__init__('stage4d_replanning_trace'); self.out=out; self.rows=[]
  for topic,kind,name in [('/far/planner_status',DiagnosticArray,'far'),('/local_planner/status',DiagnosticArray,'local'),('/path_follower/status',DiagnosticArray,'follower')]: self.create_subscription(kind,topic,lambda m,n=name:self.diag(n,m),10)
  self.create_subscription(PointStamped,'/way_point',lambda m:self.add('waypoint',x=m.point.x,y=m.point.y,z=m.point.z),10); self.create_subscription(Bool,'/navigation_active',lambda m:self.add('navigation_active',active=m.data),10); self.create_subscription(Path,'/path',lambda m:self.add('path',size=len(m.poses)),10); self.create_subscription(TwistStamped,'/cmd_vel',lambda m:self.add('cmd_vel',vx=m.twist.linear.x,vy=m.twist.linear.y,wz=m.twist.angular.z),10)
 def add(self,source,**values): self.rows.append({'wall_time':time.time(),'source':source,**values})
 def diag(self,source,msg):
  for st in msg.status: self.add(source,**{v.key:v.value for v in st.values})
 def save(self):
  os.makedirs(os.path.dirname(os.path.abspath(self.out)),exist_ok=True); json.dump(self.rows,open(self.out+'.json','w'),indent=2); keys=sorted({k for r in self.rows for k in r}); w=csv.DictWriter(open(self.out+'.csv','w',newline=''),fieldnames=keys); w.writeheader(); w.writerows(self.rows)
def main():
 a=argparse.ArgumentParser();a.add_argument('--output',default='/tmp/stage4d_replanning_trace');a.add_argument('--duration',type=float,default=30);x=a.parse_args();rclpy.init();n=Trace(x.output);end=time.monotonic()+x.duration
 try:
  while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 finally:n.save();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
