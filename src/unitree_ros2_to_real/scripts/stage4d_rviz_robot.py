#!/usr/bin/env python3
"""RViz-only robot replica and trails; publishes no motion commands."""
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class RobotViz(Node):
    def __init__(self):
        super().__init__('stage4d_rviz_robot')
        self.gt = None; self.lio = None; self.gt_trail=[]; self.lio_trail=[]; self.last_pub=0.0
        self.pub = self.create_publisher(MarkerArray, '/stage4d/robot_model', 10)
        self.gt_path_pub = self.create_publisher(Path, '/stage4d/ground_truth_path', 10)
        self.lio_path_pub = self.create_publisher(Path, '/stage4d/pointlio_path', 10)
        self.create_subscription(Odometry, '/sim/ground_truth_odom', lambda m: self.set_pose('gt',m), 10)
        self.create_subscription(Odometry, '/state_estimation', lambda m: self.set_pose('lio',m), 10)
        self.create_timer(0.05, self.tick)
    def set_pose(self, kind, msg):
        setattr(self, kind, msg)
    def tick(self):
        now=time.monotonic()
        if now-self.last_pub < .05: return
        self.last_pub=now; arr=MarkerArray()
        for kind,msg,color in (('gt',self.gt,(0.1,0.9,0.2,0.95)),('lio',self.lio,(0.2,0.5,1.0,0.95))):
            if msg is None: continue
            frame=msg.header.frame_id or 'camera_init'; p=msg.pose.pose.position; q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            body=Marker(); body.header=msg.header; body.header.frame_id=frame; body.ns='robot_'+kind; body.id=0; body.type=Marker.CUBE; body.action=Marker.ADD; body.pose=msg.pose.pose; body.scale.x=.55; body.scale.y=.38; body.scale.z=.28; body.color.r,body.color.g,body.color.b,body.color.a=color; body.lifetime.nanosec=100000000; arr.markers.append(body)
            for i,(dx,dy) in enumerate(((.22,.16),(.22,-.16),(-.22,.16),(-.22,-.16)),1):
                leg=Marker(); leg.header=body.header; leg.ns='robot_'+kind+'_legs'; leg.id=i; leg.type=Marker.LINE_LIST; leg.action=Marker.ADD; leg.scale.x=.045; leg.color.r,leg.color.g,leg.color.b,leg.color.a=color
                c,s=math.cos(yaw),math.sin(yaw); x=p.x+c*dx-s*dy; y=p.y+s*dx+c*dy
                leg.points=[Point(x=x,y=y,z=p.z),Point(x=x,y=y,z=max(0.02,p.z-.30))]; leg.lifetime.nanosec=100000000; arr.markers.append(leg)
            trail=self.gt_trail if kind=='gt' else self.lio_trail; trail.append((msg.header.stamp,p.x,p.y,p.z,frame)); del trail[:-1000]
        self.pub.publish(arr)
        self.publish_path(self.gt_trail,self.gt_path_pub)
        self.publish_path(self.lio_trail,self.lio_path_pub)
    def publish_path(self, trail, pub):
        if not trail: return
        path=Path(); path.header.stamp=self.get_clock().now().to_msg(); path.header.frame_id=trail[-1][4]
        for stamp,x,y,z,_ in trail:
            from geometry_msgs.msg import PoseStamped
            ps=PoseStamped(); ps.header.stamp=stamp; ps.header.frame_id=path.header.frame_id; ps.pose.position.x=x; ps.pose.position.y=y; ps.pose.position.z=z; ps.pose.orientation.w=1.; path.poses.append(ps)
        pub.publish(path)
def main():
    rclpy.init(); n=RobotViz()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
