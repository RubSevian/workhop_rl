#!/bin/bash
conda deactivate
echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
source $HOME/go2_deploy/workhop_rl/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source $HOME/go2_deploy/workhop_rl/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enx00e04c6802fc" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

