#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <deque>
#include <vector>
#include <math.h>
#include <algorithm>
#include <boost/bind.hpp>


 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>

 using std::string;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

 namespace planner_3d 
 {

    class Planner3D : public nav_core::BaseGlobalPlanner 
    {
        public:
            Planner3D();
            Planner3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            /** overridden classes from interface nav_core::BaseGlobalPlanner **/
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan);

        private:
            ros::NodeHandle n;

            ros::Publisher marker_pub;
            ros::Publisher plan_pub;
            ros::Subscriber sub;

            visualization_msgs::Marker marker;

            grid_map::Index children[8] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                        {0, 1}, {0, -1}, {1, 0}, {-1, 0}};
  };
 };
 #endif