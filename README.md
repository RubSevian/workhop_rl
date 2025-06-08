# workhop_rl

### How to install and build code
1. Use ros2 Humble for this project + Ubuntu 22
2. Clone repo
```bash
git clone -b ros2 https://github.com/RubSevian/workhop_rl.git
```
3. Add to your terminal source ros2
```bash
source /opt/ros/humble/setup.bash
``` 
4.  Install this packages
```bash
sudo apt update
sudo apt install ros-humble-std-msgs
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
sudo apt install ros-$ROS_DISTRO-rosidl-generator-dds-idl
```
5. Download and deploy `libtorch` at any location

```bash
cd /path/to/your/libtorch
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.1.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/libtorch' >> ~/.bashrc
source ~/.bashrc
```
6. Install `yaml-cpp` and `lcm`. If you are using Ubuntu, you can directly use the package manager for installation:

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

## Install mujoco
Current version is tested in mujoco-3.3.1 check in original repo mujoco this version
```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```
```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
Test:
```bash
simulate
```

## Installation unitree_ros2_sdk

To build your own application with the SDK, you can install the unitree_sdk2 to your system directory:

```bash
cd src/unitree_ros2_to_real/library/
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
mkdir build
cd build
cmake ..
sudo make install
```


## If your verison ros2 foxy , change version rmw_cyclonedds to foxy / humble

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
sudo apt install ros-humble-rosidl-generator-dds-idl
cd src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd .. && cd ..
colcon build --packages-select cyclonedds
cd .. && cd .. && cd ..
```

## Compilation for SIM2SIM on Mujoco
1. Compile in the root directory of the project

```bash
source ./env_setup.sh  # in first time error (install/setup.bash: No such file or directory) - normal

colcon build 

source ./env_setup.sh  #(without errors)
```
2. Export path to libtorch

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ruben/workhop_rl/src/libtorch/lib/ #change to your libtorch path
```
3. Start code in terminal with (source ./env_setup.sh) you need 2 treminals

3.1. First Terminal (run Mujoco)

```bash
sudo ./build/unitree_mujoco/unitree_mujoco 

```
3.2. Second Terminal (run statnd_up)

```bash
ros2 run stand_go2 stand_go2
```
## Results WORK (with RL policy in mujoco sim)
![alt text](img/Screenshot%20from%202025-06-07%2000-52-52.png)


## Compilation for Real Robot

1. Compile in the root directory of the project

```bash
cd ..

colcon build --merge-install --symlink-install

source install/setup.bash
```
2. Export path to libtorch

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ruben/workhop_rl/src/libtorch/lib/ #change to your libtorch path
```
3. Start code in terminal with (source install/setup.bash)
```bash
ros2 run unitree_legged_real ros2_rl

```

### ISSUE


1. Error : selected interface "lo" is not multicast-capable: disabling multicast
```bash
sudo ip link set lo multicast on # fix
```
