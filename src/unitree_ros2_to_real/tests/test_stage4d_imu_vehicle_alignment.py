#!/usr/bin/env python3
import math
import unittest
T_BI=(-0.02557,0.0,0.04232)
T_IV=(0.02557,0.0,-0.04232)
def rotate(yaw,v):
 c,s=math.cos(yaw),math.sin(yaw); return (c*v[0]-s*v[1],s*v[0]+c*v[1],v[2])
class TestImuVehicleAlignment(unittest.TestCase):
 def test_imu_to_vehicle_recovers_base(self):
  for yaw in (0,math.pi/2,-math.pi/2,math.pi/4):
   base=(1.7,-0.8,0.35); d=rotate(yaw,T_BI); imu=tuple(base[i]+d[i] for i in range(3)); correction=rotate(yaw,T_IV); vehicle=tuple(imu[i]+correction[i] for i in range(3));
   self.assertLess(math.dist(base,vehicle),1e-6)
if __name__=='__main__': unittest.main()
