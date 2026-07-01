# XPBD-Rods
Using XPBD to simulate the dynamics of 3D Cosserat rods in real-time.

## Installing Dependencies
As of 10/28/25, this repository relies on 3 external libraries: **Eigen**, **yaml-cpp**, and **VTK**. Instructions for cloning and installing these libraries are given below:
```
#########################
# Eigen install
#########################
cd <folder where you want to clone>
git clone https://gitlab.com/libeigen/eigen.git
mkdir eigen/build
cd eigen/build
cmake ..
make -j 8
sudo make install
```
```
########################
# YAML-cpp
########################
cd <folder where you want to clone>
git clone https://github.com/jbeder/yaml-cpp.git
mkdir yaml-cpp/build
cd yaml-cpp/build
cmake ..
make -j 8
sudo make install
```
```
##########################
# VTK install
##########################
cd <folder where you want to clone>
git clone https://gitlab.kitware.com/vtk/vtk.git
mkdir vtk/build
cd vtk/build
cmake ..
make -j 8
sudo make install
```
## Building and Running a Basic Example
From the repo's base directory, make a build folder, run `cmake`, and then `make`:
```
mkdir build
cd build
cmake ..
make -j12
```
If CMake is able to find the dependencies (which it should be able to if you followed the steps in the previous section), then everything should build!

Then, simply run a basic example with:
```
./Sim ../config/config.yaml
```

A VTK window should pop up with an oscillating rod. Boom done!

The file `../config/config.yaml` contains setup parameters for changing aspects of the simulation, such as the rod's properties.

## ROS Interface
(07/01/2026) A basic ROS interface is provided for manipulating a Cosserat rod. To launch, run:
```
ros2 launch launch/sim_bridge.launch.py config_filename:=<path/to/yaml/config>
```
This runs a simulation based ont he YAML config file given in the command line. If no path is specified, it will default to `config/ros/rod.yaml`.

There is 1 topic published:
- `/sim/rod_frames` (`geometry_msgs/PoseArray`) - array of poses along the length of the rod. The first entry is the base of the rod, and the last entry is the tip of the rod.

There are 2 topics subscribed to:
- `/sim/rod_base_pose` (`geometry_msgs/PoseStamped`) - commanded base pose of the rod.
- `/sim/rod_tip_pose` (`geometry_msgs/PoseStamped`) - commanded tip pose of the rod.
