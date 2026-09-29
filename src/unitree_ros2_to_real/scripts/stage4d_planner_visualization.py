#!/usr/bin/env python3
"""RViz-only Stage4D navigation geometry markers; this node publishes no TF."""
import math
import os
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray

DEFAULTS={"vehicle_length":0.62,"vehicle_width":0.40,"path_scale":0.75,
          "correspondence_search_radius":0.55,"enable_orientation_aware_check":0.0}
class Visualizer(Node):
 def __init__(self):
  super().__init__("stage4d_planner_visualization")
  self.v=dict(DEFAULTS);self.mode=os.getenv("STAGE4D_ROBOT_GEOMETRY_VIZ","both").lower()
  self.far_tol=float(os.getenv("STAGE4D_FAR_CONVERGE_DISTANCE","0.25"));self.stop_tol=float(os.getenv("STAGE4D_STOP_DIS_THRE","0.30"))
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.markers=self.create_publisher(MarkerArray,"/stage4d/localplanner_collision_envelope",q)
  self.rectangle=self.create_publisher(Marker,"/stage4d/robot_rectangle_reference",q)
  self.ellipse=self.create_publisher(Marker,"/stage4d/robot_ellipse_reference",q)
  self.exact=self.create_publisher(Marker,"/stage4d/exact_orientation_aware_footprint",q)
  self.goals=self.create_publisher(MarkerArray,"/stage4d/goal_markers",q)
  self.path=None;self.requested=None;self.effective=None;self.waypoint=None
  self.create_subscription(DiagnosticArray,"/local_planner/status",self.status,q)
  self.create_subscription(Path,"/path",lambda m:setattr(self,"path",m),10)
  self.create_subscription(PointStamped,"/goal_point",self.on_goal,10)
  self.create_subscription(PointStamped,"/way_point",self.on_waypoint,10)
  self.create_subscription(DiagnosticArray,"/far/planner_status",self.far_status,q)
  self.create_subscription(Empty,"/navigation_cancel",self.on_cancel,10)
  self.create_timer(.2,self.publish)
 def on_goal(self,msg):
  self.requested=msg;self.effective=None;self.waypoint=None
  self.clear_goals()
 def on_waypoint(self,msg):
  if self.requested is not None:self.waypoint=msg
 def on_cancel(self,_msg):
  self.requested=None;self.effective=None;self.waypoint=None
  self.clear_goals()
 def clear_goals(self):
  marker=self.marker(Marker.SPHERE,"Goal - Requested");marker.action=Marker.DELETEALL
  arr=MarkerArray();arr.markers=[marker];self.goals.publish(arr)
 def far_status(self,msg):
  if self.requested is None:return
  for status in msg.status:
   if status.name!="far_planner":continue
   values={item.key:item.value for item in status.values}
   try:coords=[float(values["goal_current_"+axis]) for axis in "xyz"]
   except (KeyError,ValueError):return
   if not all(math.isfinite(value) for value in coords):return
   point=PointStamped();point.header.frame_id="map"
   point.point=Point(x=coords[0],y=coords[1],z=coords[2]);self.effective=point
 def status(self,msg):
  for status in msg.status:
   if status.name!="local_planner":continue
   for item in status.values:
    try:self.v[item.key]=float(item.value)
    except ValueError:self.v[item.key]=1.0 if item.value.lower()=="true" else 0.0
 def marker(self,kind,namespace,ident=0,frame="vehicle"):
  m=Marker();m.header.frame_id=frame;m.header.stamp=self.get_clock().now().to_msg();m.ns=namespace;m.id=ident;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;return m
 def circle(self,ident,point,radius,color,label):
  m=self.marker(Marker.LINE_STRIP,label,ident,"map");m.scale.x=.025;m.color.r,m.color.g,m.color.b,m.color.a=color
  for i in range(41):
   a=2*math.pi*i/40;p=Point();p.x=point.x+radius*math.cos(a);p.y=point.y+radius*math.sin(a);p.z=point.z+.03;m.points.append(p)
  return m
 def publish_geometry(self,l,w):
  if self.mode in ("rectangle","both"):
   m=self.marker(Marker.CUBE,"Robot - Rectangle Reference");m.pose.position.z=.018;m.scale.x,m.scale.y,m.scale.z=l,w,.036;m.color.r,m.color.g,m.color.b,m.color.a=.75,.18,.95,.36;self.rectangle.publish(m)
  if self.mode in ("ellipse","both"):
   m=self.marker(Marker.LINE_STRIP,"Robot - Ellipse Reference");m.scale.x=.025;m.color.r,m.color.g,m.color.b,m.color.a=.1,.95,.85,.9
   for i in range(65):
    a=2*math.pi*i/64;p=Point();p.x=l*.5*math.cos(a);p.y=w*.5*math.sin(a);p.z=.05;m.points.append(p)
   self.ellipse.publish(m)
  if self.v.get("enable_orientation_aware_check",0)>0.5:
   m=self.marker(Marker.CUBE,"Planner - Exact Orientation-Aware Footprint");m.pose.position.z=.06;m.scale.x,m.scale.y,m.scale.z=l,w,.025;m.color.r,m.color.g,m.color.b,m.color.a=1.,.8,.05,.22;self.exact.publish(m)
 def publish_path(self):
  if not self.path or not self.path.poses:return
  arr=MarkerArray();line=self.marker(Marker.LINE_STRIP,"Planner - Selected Local Path",0,self.path.header.frame_id);line.scale.x=.045;line.color.r,line.color.g,line.color.b,line.color.a=.05,.3,1.,1.
  envelope=self.marker(Marker.SPHERE_LIST,"Planner - Broad Collision Envelope",1,self.path.header.frame_id);radius=self.v["correspondence_search_radius"]*self.v["path_scale"];envelope.scale.x=envelope.scale.y=2*radius;envelope.scale.z=.018;envelope.color.r,envelope.color.g,envelope.color.b,envelope.color.a=.95,.2,.1,.18
  for pose in self.path.poses:
   point=pose.pose.position;line.points.append(point);copy=Point();copy.x,copy.y,copy.z=point.x,point.y,.015;envelope.points.append(copy)
  arr.markers=[line,envelope];self.markers.publish(arr)
 def publish_goals(self):
  if not self.requested and not self.effective and not self.waypoint:return
  arr=MarkerArray()
  if self.requested:
   m=self.marker(Marker.SPHERE,"Goal - Requested",0,self.requested.header.frame_id)
   p=self.requested.point;m.pose.position=Point(x=p.x,y=p.y,z=p.z+.08)
   m.scale.x=m.scale.y=m.scale.z=.14;m.color.r,m.color.g,m.color.b,m.color.a=1.,.85,.05,1.;arr.markers.append(m)
  if self.effective:
   m=self.marker(Marker.SPHERE,"Goal - FAR Effective",1,self.effective.header.frame_id)
   p=self.effective.point;m.pose.position=Point(x=p.x,y=p.y,z=p.z+.08)
   m.scale.x=m.scale.y=m.scale.z=.14;m.color.r,m.color.g,m.color.b,m.color.a=.05,1.,.25,1.
   arr.markers.extend([m,self.circle(2,p,self.far_tol,(.05,1.,.25,.8),"Goal - FAR convergence tolerance")])
  if self.waypoint:
   p=self.waypoint.point;m=self.marker(Marker.SPHERE,"FAR - Current waypoint",4,self.waypoint.header.frame_id)
   m.pose.position=Point(x=p.x,y=p.y,z=p.z+.06)
   m.scale.x=m.scale.y=m.scale.z=.09;m.color.r,m.color.g,m.color.b,m.color.a=.1,.55,1.,.95
   arr.markers.extend([m,self.circle(3,p,self.stop_tol,(.1,.55,1.,.7),"FAR waypoint - follower stop tolerance")])
  self.goals.publish(arr)
 def publish(self):
  self.publish_geometry(self.v["vehicle_length"],self.v["vehicle_width"]);self.publish_path();self.publish_goals()
def main():
 rclpy.init();n=Visualizer()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
