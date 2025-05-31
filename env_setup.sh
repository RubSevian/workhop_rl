#!/bin/bash
conda deactivate
echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
source $HOME/workhop_rl/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

