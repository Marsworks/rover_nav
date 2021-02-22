# rover_nav
## Overview
This is a [ROS](https://www.ros.org/) package developed for 3D-terrain navigation for outdoor mobile robots. This package contains a 3D path planner ([planner_3d/Planner3D](https://github.com/Marsworks/rover_nav/tree/master/src)) that adheres to the nav_core::BaseGlobalPlanner interface found in the [nav_core](http://wiki.ros.org/nav_core) package. 

[grid_map](https://github.com/ANYbotics/grid_map) is used as replacement for costmap_2D to representing the 3D terrain. The planner is based off A* but with a modification to the heuristic cost function to include the the slope of the cell. The output of the planner is a 2D plan that is used by [base_local_planner](http://wiki.ros.org/base_local_planner) to generate trajectories, the local costmap is assumed to be empty and the base_local_planner just attempts to follow the path created by the global 3D planner.

Initially the map is filtered and the cells slope values are then inflated to help in avoiding non-traversable paths.

This package is **under development**.

## Requirements
 * ROS kinetic/melodic

## Installation

### Build

```bash
cd catkin_ws/src
git clone https://github.com/Marsworks/rover_nav
cd ~/catkin_ws
rosdep install --rosdistro melodic --from-paths src -y
catkin_make
source devel/setup.bash
```

## Basic Usage
  ```bash
  roslaunch rover_nav sim_3D_nav.launch
  ```