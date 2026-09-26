#!/usr/bin/env python3
"""Load the packaged FAR visibility graph once graph_decoder is ready."""
import argparse, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Bootstrap(Node):
 def __init__(self,path,delay):
  super().__init__('stage4d_graph_bootstrap'); self.path=path; self.delay=delay; self.pub=self.create_publisher(String,'/read_file_dir',10); self.start=time.monotonic(); self.sent=False; self.create_timer(.2,self.tick)
 def tick(self):
  if self.sent or time.monotonic()-self.start<self.delay or self.count_subscribers('/read_file_dir')==0: return
  self.pub.publish(String(data=self.path)); self.sent=True; self.get_logger().info('Loaded FAR graph: '+self.path)
def main():
 p=argparse.ArgumentParser(); p.add_argument('--graph-path',required=True); p.add_argument('--delay',type=float,default=3.0); a,_=p.parse_known_args(); rclpy.init(); n=Bootstrap(a.graph_path,a.delay)
 try: rclpy.spin(n)
 except KeyboardInterrupt: pass
 finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
