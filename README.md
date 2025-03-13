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
```
5. Download and deploy `libtorch` at any location

```bash
cd /path/to/your/libtorch
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/libtorch' >> ~/.bashrc
source ~/.bashrc
```
6. Install `yaml-cpp` and `lcm`. If you are using Ubuntu, you can directly use the package manager for installation:

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```
## Compilation

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



