# Introduction
This package can send control command to real robot from ROS2. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.4.1, namely Go1 robot.

## Packages:

Basic message function: `unitree_legged_msgs`

The interface between ROS and real robot: `unitree_legged_real`

# Dependencies
* [unitree_legged_sdk](https://wavegit.mipt.ru/litvinenkov/unitree_legged_sdk.git): v3.4.1 (develop)

# Configuration
Download this package into this `~/unitree_ws/src` folder. 

After you download this package into this folder, your folder should be like this
```
~/ros2_ws/src/unitree_ros2_to_real
```
And now download unitree_legged_sdk v3.4.1 into the path `~/unitree_ws/src_unitree_ros2_to_real`


# Build
```
cd ~/unitree_ws
```

```
colcon build
```


# Run the package
Before you do low level control, you should run the `lcm_server` node, which is a bridge that connects users and robot
```
cd unitree_ws
source install/setup.bash 
```

```
ros2 run unitree_legged_real lcm_server lowlevel
```

Then run the node `ros2_rl`
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/libtorch/lib/
```

```
ros2 run unitree_legged_real ros2_rl
```

And before you do the low-level control, please press L2+A to sit the robot down and then press L1+L2+start to make the robot into
mode in which you can do joint-level control, finally make sure you hang the robot up before you run low-level control.

