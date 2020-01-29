# rover_nav

## Requirements
### System requirements
* Ubuntu 16.04
* ROS kinect
* Gazebo 7 (needed for running simulations only)

### ROS packages
* realsense2_camera
  * Install from  [here](https://github.com/IntelRealSense/realsense-ros).
* elevation_mapping
  * Install from [here](https://github.com/ANYbotics/elevation_mapping).
* ocotmap
```shell
sudo apt install ros-kinetic-octomap
sudo apt install ros-kinetic-octomap-server
```

## Installation
```bash
sudo apt-get update
sudo apt-get upgrade
cd catkin_ws/src
git clone https://gitlab.com/marsworks/rover_nav
git clone https://github.com/ros-planning/navigation.git
git clone https://gitlab.com/marsworks/rover_description
cd rover_description/worlds
sudo cp moon_surface.world //usr/share/gazebo-7/worlds/moon_surface.world
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
### costmap_2d hack
There is an issue with the static layer in costmap_2d when the static layer resizes. Implement the temp fix below for thing to work as intended:
* Find the file
```bash
roscd costmap_2d/plugins
gedit static_layer.cpp
```
* Edit line 194 by adding "layered_costmap_->" at the start of the line.
* compile and build changes
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage Instructions
* Octomap + 2D navigation + simultaion
```bash
roslaunch rover_nav sim_2D_nav.launch
```
* Octomap + 2D navigation + hardware
```bash
roslaunch rover_nav hardware_2D_nav.launch
```
