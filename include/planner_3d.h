#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <algorithm>
#include <boost/bind.hpp>
#include <deque>
#include <math.h>
#include <vector>

#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <filters/filter_chain.h>

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace planner_3d {

class Planner3D : public nav_core::BaseGlobalPlanner {
public:
  Planner3D();
  Planner3D(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  double factors[4]; // heuristic function coeffcients
  std::string elevation_layer;
  std::string slope_layer;

private:
  grid_map::GridMap raw_gridmap;  // converted ocotomap to gridmap
  grid_map::GridMap full_gridmap; // Full size gridmap 40x80m (non-filtered)
  grid_map::GridMap
      filtered_gridmap; // Filtered gridmap (based on Slope + inflation)

  grid_map_msgs::GridMap grid_map_message;

  grid_map::Index children[8] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                                 {0, 1}, {0, -1}, {1, 0},  {-1, 0}};

  filters::FilterChain<grid_map::GridMap> map_filter{"grid_map::GridMap"};
  std::string filter_chain_params;

  ros::NodeHandle nh;
  ros::Publisher marker_publisher;
  ros::Publisher full_map_publisher; // Publishes the full gridmap
  ros::Publisher filtered_map_publisher;
  ros::Subscriber global_map_subscrber;

  visualization_msgs::Marker marker;

  bool grid_map_initialized;

  void gridmap_callback(const grid_map_msgs::GridMapConstPtr &grid_map_msg_ptr);
};
}; // namespace planner_3d
#endif
