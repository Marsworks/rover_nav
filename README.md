# rover_nav
## Overview
This is a [ROS](https://www.ros.org/) package developed for 3D-terrain navigation for outdoor mobile robots. This package contains a 3D path planner ([planner_3d/Planner3D](https://github.com/Marsworks/rover_nav/tree/master/src)) that adheres to the nav_core::BaseGlobalPlanner interface found in the [nav_core](http://wiki.ros.org/nav_core) package.

This package is still **under development**.

## Requirements
### System requirements
* Ubuntu 16.04
* ROS kinect
* Gazebo 7 (needed for running simulations only)

## Installation

### Dependencies
* realsense2_camera

  Install from  [here](https://github.com/IntelRealSense/realsense-ros).
* grid_map 
  ```shell
  sudo apt-get install ros-kinetic-grid-map
  ```
* ocotmap
  ```shell
  sudo apt install ros-kinetic-octomap-server
  ```

### Build

```bash
cd catkin_ws/src
git clone https://github.com/Marsworks/rover_nav
git clone https://github.com/Marsworks/rover_description
git clone https://github.com/ros-planning/navigation.git
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
* Build changes
  ```bash
  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash
  ```

## Basic Usage
* 3D-terrain navigation using 
  ```bash
  roslaunch rover_nav sim_2D_nav.launch
  ```
* Octomap + 2D navigation + hardware
```bash
roslaunch rover_nav hardware_2D_nav.launch
```
