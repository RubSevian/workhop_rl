#!/usr/bin/env python3
"""Publish one Stage4D goal in the FAR/Point-LIO world frame."""
import argparse, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
CASES={'D0':(1.0,0.0),'D1':(1.2,0.8),'D2':(1.8,-0.8),'D3':(0.6,1.0),'D4A':(1.0,0.0),'D4B':(0.6,0.9)}
def main():
    p=argparse.ArgumentParser(); p.add_argument('--case',choices=CASES); p.add_argument('--x',type=float); p.add_argument('--y',type=float); p.add_argument('--frame',default='camera_init'); p.add_argument('--duration',type=float,default=1.0); p.add_argument('--rate',type=float,default=10.0); a=p.parse_args()
    if a.case is None and (a.x is None or a.y is None): p.error('--case or --x/--y is required')
    x,y=CASES[a.case] if a.case else (a.x,a.y)
    rclpy.init(); n=Node('stage4d_send_goal'); pub=n.create_publisher(PointStamped,'/goal_point',10); period=1.0/a.rate
    discovery_deadline=time.monotonic()+10.0
    while rclpy.ok() and n.count_subscribers('/goal_point') == 0 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.count_subscribers('/goal_point') == 0:
        n.get_logger().error('No /goal_point subscriber discovered; goal was not sent')
        n.destroy_node(); rclpy.shutdown(); return
    end=time.monotonic()+a.duration
    while rclpy.ok() and time.monotonic()<end:
        m=PointStamped(); m.header.stamp=n.get_clock().now().to_msg(); m.header.frame_id=a.frame; m.point.x=x; m.point.y=y; pub.publish(m); rclpy.spin_once(n,timeout_sec=0.0); time.sleep(period)
    n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
