#include <ros/ros.h>
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


struct node
{
    grid_map::Position position; // position in the odom frame (x, y) m
    grid_map::Index index; // index in the map 2D array

    double g = 0; // Number of cells from the start node
    double h; // Distance from node to goal
    double f; // cost function
    double height; //height data in the cell

    node *parent_node = NULL;

    node(grid_map::Position pos, const grid_map::GridMap &map)
    {
      position = pos;
      if(map.isInside(position))
      {
        map.getIndex(position, index);
        height = map.atPosition("elevation", position);
        height = (std::isfinite(height))? height:0;
      }

      // ROS_INFO("Special2 row: %d, col: %d, res: %f", row, col, res);
    }

    node(grid_map::Index ind, const grid_map::GridMap &map)
    {
      index = ind;
      if(map.isValid(index))
      {
        map.getPosition(index, position);
        
        height = map.atPosition("elevation", position);
        height = (std::isfinite(height))? height:0;
      }
      else
      {
        height = 0;
      }
    }

    node(grid_map::Index ind)
    {
      index = ind;
    }

    void calc_f(node *parent, node goal)
    {
      parent_node = parent;
      //ROS_INFO("parent_node; row: %d, col: %d", parent_node->row, parent_node->col);
      
      g = 1 + parent_node->g;
      
      h = sqrt(pow(goal.position(0) - position(0), 2) + pow(goal.position(1) - position(1), 2));
      
      f = g + h*50 + height*40;
      //ROS_INFO("Cost function; g: %.2f, h: %.2f, height_cost:%.2f & f: %.2f", g, h, height_cost, f);
    }
};

bool send_path(node *cell, node start, ros::Publisher &plan_pub)
{
  nav_msgs::Path plan;
  geometry_msgs::PoseStamped temp_pos;
  std::vector<geometry_msgs::PoseStamped> poses_list;
  
  while(cell != NULL && ros::ok())
  { 
    temp_pos.pose.position.x = cell->position(0);
    temp_pos.pose.position.y = cell->position(1);
    temp_pos.pose.position.z = cell->height;

    poses_list.push_back(temp_pos);
    cell = cell->parent_node;
  }

  plan.header.frame_id = "t265_odom_frame";
  plan.poses = poses_list;
  
  plan_pub.publish(plan);

  return 1;
}

bool node_sort (node i,node j) 
{ 
  return i.f < j.f; 
}

int is_in_list(std::deque<node> list, node cell)
{
  // ToDo convert to binary search
  for(int c=0; c<list.size(); c++)
    if(list[c].index(0) == cell.index(0) && list[c].index(1) == cell.index(1))
      return c;
  return -1;
}

bool a_start(const node start, const node goal, grid_map::GridMap &map, ros::Publisher &plan_pub, ros::Publisher &arrow_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "t265_odom_frame";
  marker.header.stamp = ros::Time();
    
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

  int cntr = 0;
  //std::cin.get();
  ROS_INFO("Finding path...");
 
  std::deque<node> open_list, closed_list;
  
  grid_map::Index children[8] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                    {0, 1}, {0, -1}, {1, 0}, {-1, 0}};

  open_list.push_back(start);
  ros::Time time;
  while(open_list.size() && ros::ok())
  {    
    std::sort(open_list.begin(), open_list.end(), node_sort);

    node *current = new node(open_list[0]);
    
    marker.id = ++cntr;
    marker.pose.position.x = current->position.x();
    marker.pose.position.y = current->position.y();
    marker.pose.position.z = current->height;

    // arrow_pub.publish(marker);

    open_list.pop_front();
    closed_list.push_back(*current);

    if(current->index(0) == goal.index(0) && current->index(1) == goal.index(1)) // Index has been choosen instead of position to avoid comparing floats
    {
      ROS_INFO("Plan found");
      
      if(send_path(current, start, plan_pub))
        ROS_INFO("Plan published");
      
      return 1;
    }

    for(auto c:children)
    {
      grid_map::Index temp_pos(current->index(0)+c(0), current->index(1)+c(1));
      node child = node(temp_pos, map);
      
      int pos_in_list = is_in_list(closed_list, child);
  
      if(map.isValid(temp_pos) && pos_in_list == -1) // Checking if the index is within the map
      {  
          child.calc_f(current, goal);
          
          pos_in_list = is_in_list(open_list, child);
          
          if(pos_in_list == -1 || (pos_in_list >= 0 && open_list[pos_in_list].g > child.g))
            open_list.push_back(child);
      }
    }
  }

  ROS_INFO("No plan found.");
  return 0;
}


void mapCallback(ros::Publisher &plan_pub, ros::Publisher &arrow_pub, const grid_map_msgs::GridMap::ConstPtr& map_msg_ptr)
{
  grid_map::GridMap gridMap;
  const grid_map_msgs::GridMap map_msg = *map_msg_ptr;
  visualization_msgs::Marker marker;
  grid_map::Position pos;

  grid_map::GridMapRosConverter::fromMessage(map_msg, gridMap);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/t265_odom_frame", "/base_link", ros::Time(0), ros::Duration(10.0));
  listener.lookupTransform("/t265_odom_frame", "/base_link",ros::Time(0), transform);
    
  pos(0) = transform.getOrigin().x();
  pos(1) = transform.getOrigin().y();
  node start(pos, gridMap);
  
  std::cout << "Enter goal: \n";
  std::cin >> pos(0) >> pos(1);

  node goal(pos, gridMap);

  marker.header.frame_id = "t265_odom_frame";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.8;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  marker.id = 0;
  marker.pose.position.x = pos(0);
  marker.pose.position.y = pos(1);
  marker.pose.position.z = goal.height;
  marker.color.r = 0.0;
  marker.color.g = 1.0;

  arrow_pub.publish(marker);

  ros::Time time;
  time = ros::Time::now();
  a_start(start, goal, gridMap, plan_pub, arrow_pub);
  ROS_INFO("Total time: %f", (ros::Time::now()-time).toSec());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_3d"); // Node Initialization
  ros::NodeHandle n;
  

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>( "points_visualization", 0 );
  ros::Publisher plan_pub = n.advertise<nav_msgs::Path>("path_3D", 1000);
  
  // ros::Subscriber sub = n.subscribe("/octomap_to_gridmap_demo/grid_map", 1000, mapCallback);
  // https://answers.ros.org/question/12970/how-to-use-nodehandle-in-callback/
  ros::Subscriber sub = n.subscribe<grid_map_msgs::GridMap>("/octomap_to_gridmap_demo/grid_map", 
      1000, boost::bind(&mapCallback, boost::ref(plan_pub), boost::ref(marker_pub) , _1));
  
  ROS_INFO("LOL Started...");

  

 

  ros::spin();

  return 0;
}