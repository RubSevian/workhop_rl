"""Real NAV callback methods on fake node ports; no ROS/DDS init or output."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent))
import first_grasp_mission_test
from navigation_gate import NavigationGate
from navigation_goal import GoalTracker
from navigation_recovery import PathRecovery
from go2_navigation_bridge import NavigationBridge

class NavAdapterContracts(unittest.TestCase):
    def node(self):
        calls=[];gate=NavigationGate(recovery_samples=5)
        node=NS(gate=gate,goals=GoalTracker(),goal_after_ns=100,command_after_ns=100,
                mission_health=NS(base_locked=False,return_request_ns=0),path_recovery=PathRecovery(),
                header_valid=lambda msg:True,header_age=lambda msg:0.,
                get_clock=lambda:NS(now=lambda:NS(nanoseconds=1000)),tick=lambda:calls.append('tick'),
                get_logger=lambda:NS(warn=lambda msg:None))
        return node,calls

    def test_soft_invalidation_does_not_release_operator_cancel(self):
        for cancelled in (False,True):
            node,calls=self.node()
            node.goals.accept((1,2,0),'map',10,1.);node.gate.goal_received=True
            if cancelled:node.gate.cancel()
            response=NavigationBridge.invalidate_goal(node,None,NS())
            self.assertTrue(response.success);self.assertEqual(node.gate.cancelled,cancelled)
            self.assertFalse(node.gate.goal_received);self.assertEqual(node.goals.state,'LOCALIZATION_LOST')
            self.assertEqual(node.goal_after_ns,1000);self.assertEqual(calls,['tick'])

    def test_goal_rejected_before_fifth_pair_then_new_goal_required(self):
        node,_=self.node()
        for name in ('lidar','imu','odom','tf'):node.gate.observe(name,True,100.)
        msg=NS(header=NS(frame_id='map',stamp=NS(sec=0,nanosec=200)),point=NS(x=1,y=2,z=0))
        for n in range(1,5):node.gate.observe_pair(n,True)
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.):
            NavigationBridge.goal(node,msg);self.assertEqual(node.goals.revision,0)
            node.gate.observe_pair(5,True);NavigationBridge.goal(node,msg)
        self.assertEqual(node.goals.revision,1);self.assertTrue(node.gate.goal_received)
        node.gate.cancel()
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.):
            msg.header.stamp.nanosec=300;NavigationBridge.goal(node,msg)
        self.assertEqual(node.goals.revision,1)

    def test_recovered_path_does_not_accept_queued_old_velocity(self):
        node,_=self.node();node.goals.stamp_ns=100;node.command_after_ns=1000
        accepted=[];node.gate.accept=lambda *args:accepted.append(args)
        msg=NS(header=NS(frame_id='vehicle',stamp=NS(sec=0,nanosec=999)),
               twist=NS(linear=NS(x=.1,y=0),angular=NS(z=0)))
        NavigationBridge.command(node,msg);self.assertEqual(accepted,[])
        msg.header.stamp.nanosec=1001;NavigationBridge.command(node,msg)
        self.assertEqual(len(accepted),1)

if __name__=='__main__':unittest.main()
