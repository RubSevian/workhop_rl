#!/usr/bin/env python3
"""SDK-only query subprocess -> timestamped ROS observation. Never switches mode."""
import argparse
import json
import subprocess
import time

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--interface',required=True)
    args=parser.parse_args()
    import rclpy
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_prefix
    helper=get_package_prefix('unitree_legged_real')+'/lib/unitree_legged_real/go2_mode_switch'
    rclpy.init();node=rclpy.create_node('r3_read_only_sport_monitor')
    pub=node.create_publisher(String,'/go2/commissioning/sport_observation',10)
    def query():
        before=time.monotonic_ns()
        state='SPORT_ERROR'
        try:
            result=subprocess.run([helper,'--interface',args.interface,'--status','--machine-readable'],capture_output=True,text=True,timeout=8)
            states=[line.strip() for line in result.stdout.splitlines() if line.strip() in ('SPORT_ACTIVE','SPORT_RELEASED','SPORT_UNKNOWN','SPORT_ERROR')]
            if len(states)==1 and result.returncode==0:state=states[0]
        except (OSError,subprocess.TimeoutExpired):pass
        pub.publish(String(data=json.dumps({'state':state,'observed_ns':before})))
    node.create_timer(.2,query)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
