#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <vector>


void mapCallback(const grid_map_msgs::GridMap::ConstPtr& map)
{
  ROS_INFO("Res: %f", map->info.resolution);
  ROS_INFO("size; x: %f, y:%f", map->info.length_x, map->info.length_y);
  
  int col_num = map->data[0].layout.dim[0].size;
  int col_stride = map->data[0].layout.dim[0].stride;

  int row_num = map->data[0].layout.dim[1].size;
  int row_stride = map->data[0].layout.dim[1].stride;

  std::vector<float> height_data = map->data[0].data;
  for(auto i:height_data)
     ROS_INFO("%f", i);

  ROS_INFO("Row; num: %d, stride: %d", row_num, row_stride);
  ROS_INFO("Column; num: %d, stride: %d", col_num, col_stride);
  ROS_INFO("row size(): %d", map->data[0].data.size());
  ROS_INFO("col size(): %d", map->data[1].data.size());
  ROS_INFO("label 0: %s", map->data[0].layout.dim[0].label.c_str());
  ROS_INFO("label 1: %s", map->data[0].layout.dim[1].label.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_3d"); // Node Initialization
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/octomap_to_gridmap_demo/grid_map", 1000, mapCallback);
  ros::spin();

  return 0;
}