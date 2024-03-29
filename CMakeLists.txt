cmake_minimum_required(VERSION 3.5.1)
project(rover_nav)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_ros
  grid_map_pcl
  base_local_planner
  nav_core
  pluginlib
  filters
  dynamic_reconfigure
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in televation
# dynamic reconfigure installation stuff
generate_dynamic_reconfigure_options(
  cfg/planner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# install(DIRECTORY config doc launch rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(PROGRAMS scripts/ground_truth_pose_publisher.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_library(planner_3d src/planner_3d.cpp)
add_dependencies(
  planner_3d 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  planner_3d
  ${catkin_LIBRARIES}
)

install(
  TARGETS planner_3d
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # PATTERN ".svn" EXCLUDE
)

install(
  FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# planner_tuner c++ node build
add_executable(planner_tuner src/planner_tuner.cpp)
target_link_libraries(planner_tuner 
  ${catkin_LIBRARIES}
  planner_3d
)

add_dependencies(planner_tuner ${PROJECT_NAME}_gencfg)