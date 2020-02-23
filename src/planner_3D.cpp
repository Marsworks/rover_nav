#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <deque>
#include <vector>
#include <math.h>
#include <algorithm>
#include <boost/bind.hpp>


struct node
{
    double x; // Node x position in meters
    double y; // Node y position in meters
    int row;
    int col;
    int pos;

    double g = 0; // Number of cells from the start node
    double h; // Distance from node to goal
    double f; // cost function
    double height; //height data in the cell

    node *parent_node = NULL;

    node(){}
    node(double x_pos, double y_pos, const double res, double x_offset, double y_offset, double map_x_len, double map_y_len)
    {
      /*
      args:
        x_pos: x pose in the t265 frame (m)
        y_pos: y pose in the t265 frame (m)
        res: map resolution (m/cell)
        x_offset: map x origin pose with reference to global frame (t265_odom_frame) (m)
        y_offset: map y origin pose with reference to global frame (t265_odom_frame) (m)
      */
      row = (int)round((x_offset + (map_x_len/2) - x_pos)/res);
      col = (int)round((y_offset + (map_y_len/2) - y_pos)/res);

      x = x_pos;
      y = y_pos;

      ROS_INFO("Special2 row: %d, col: %d, res: %f", row, col, res);
      std::cin.get();
    }

    node(int r, int c)
    {
      row = r;
      col = c;
    }

    node(int r, int c, const float *height_array, const int r_num, const int r_stride, 
      const int c_num, const double res, double x_offset, double y_offset, double map_x_len, double map_y_len)
    {
      row = r;
      col = c;

      x = x_offset + (map_x_len/2) - (row*res);
      y = y_offset + (map_y_len/2) - (col*res);

      // Calculate position in array
      if(row >=0 && row <r_num && col >=0 && col < c_num)
      {
        // multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
        pos = (r_stride*row) + col;
        height = (std::isfinite(height_array[pos]))? height_array[pos]:0;
      }
      else
        pos = -1; // Means oustide of map
      
      // ROS_INFO("pos: %d, row: %d, col: %d, height: %f", pos, row, col, height);
    }

    void calc_f(node *parent, node goal)
    {
      parent_node = parent;
      //ROS_INFO("parent_node; row: %d, col: %d", parent_node->row, parent_node->col);
      
      g = 1 + parent_node->g;
      h = sqrt(pow(goal.row - row, 2) + pow(goal.col - col, 2)); 
      //height = pow(height, 3);
      double height_factor = 100;
      
      f = g + h + (height*height_factor);
      ROS_INFO("Cost function; g: %.2f, h: %.2f, height:%.2f & f: %.2f", g, h, height*height_factor, f);
    }
};

bool send_path(node *cell, node start, double res, ros::Publisher &plan_pub)
{
  nav_msgs::Path plan;
  geometry_msgs::PoseStamped temp_pos;
  std::vector<geometry_msgs::PoseStamped> poses_list;
  
  while(cell != NULL && ros::ok())
  {    
    temp_pos.pose.position.x = cell->x;
    temp_pos.pose.position.y = cell->y;

    if(std::isfinite(cell->height))
      temp_pos.pose.position.z = cell->height;
    else
      temp_pos.pose.position.z = 0;

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
    if(list[c].pos == cell.pos)
      return c;
  return -1;
}

bool a_start(const node start, const node goal, const grid_map_msgs::GridMap::ConstPtr& map, ros::Publisher &plan_pub)
{
  //std::cin.get();
  ROS_INFO("Finding path...");
  double map_res = map->info.resolution;

  int col_num = map->data[0].layout.dim[0].size; //num of columns
  int data_len = map->data[0].layout.dim[0].stride; //data len

  int row_num = map->data[0].layout.dim[1].size; //num of rows
  int row_stride = map->data[0].layout.dim[1].stride; // row stride

  int x_offset = (map->info.pose.position.x);
  int y_offset = (map->info.pose.position.y);

  const float *height_data = &map->data[0].data[0];

  std::deque<node> open_list, closed_list;
  
  node children[8] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                    {0, 1}, {0, -1}, {1, 0}, {-1, 0}};

  open_list.push_back(start);

  while(open_list.size() && ros::ok())
  {
    //std::cin.get();
    
    //ROS_INFO("Looping!");
    std::sort(open_list.begin(), open_list.end(), node_sort);
  
    node *current = new node(open_list[0]);
    //current = &open_list[0];
    //ROS_INFO("Current addres: %d", current);
    open_list.pop_front();
    //ROS_INFO("dbg2");
    closed_list.push_back(*current);

    if(current->row == goal.row && current->col == goal.col)
    {
      ROS_INFO("Plan found");
      
      if(send_path(current, start, map_res, plan_pub))
        ROS_INFO("Plan published");
      
      return 1;
    }

    for(auto c:children)
    {
      node child = node(current->row+c.row, current->col+c.col, 
            height_data, row_num, row_stride, col_num, map_res, x_offset, y_offset, map->info.length_x, map->info.length_y);
      
      if(child.pos >= 0) //That means it is within the map
      {
        //child.height = height_data[child.pos];
        
        int pos_in_list = is_in_list(closed_list, child);
        if(pos_in_list == -1)
        {
          child.calc_f(current, goal);
          //ROS_INFO("===============================================================");
          pos_in_list = is_in_list(open_list, child);
          if(pos_in_list == -1 || (pos_in_list >= 0 && open_list[pos_in_list].g > child.g))
          {
            //ROS_INFO("//////////////////////////////////////////////////////////////");
            open_list.push_back(child);
          }
        }
      }
    }
  }

  ROS_INFO("Failed to publish plan");
  return 0;
}


void mapCallback(ros::Publisher &plan_pub, ros::Publisher &arrow_pub,const grid_map_msgs::GridMap::ConstPtr& map)
{
  // ROS_INFO("Res: %f", map->info.resolution);
  // ROS_INFO("size; x: %f, y:%f", map->info.length_x, map->info.length_y);
  
  double map_res = map->info.resolution;

  //int col_num = map->data[0].layout.dim[0].size; // ?????
  
  int col_num = map->data[0].layout.dim[0].size; //num of columns
  int data_len = map->data[0].layout.dim[0].stride; //data len

  int row_num = map->data[0].layout.dim[1].size; //num of rows
  int row_stride = map->data[0].layout.dim[1].stride; // row stride

  
  // std::vector<float> height_data = map->data[0].data;
  // for(auto i:height_data)
  //    ROS_INFO("%f", i);
  // ROS_INFO("Res: %f",map_res);
  // ROS_INFO("data.size(): %d", map->data[0].data.size());
  // ROS_INFO("data_len: %d", data_len);

  // ROS_INFO("Row; num: %d, stride: %d", row_num, row_stride);
  // ROS_INFO("Column; num: %d, stride: %d", col_num);

  // ROS_INFO("row size(): %d", map->data[0].data.size()); // ???
  // ROS_INFO("col size(): %d", map->data[1].data.size()); // ???

  // ROS_INFO("label 0: %s", map->data[0].layout.dim[0].label.c_str());
  // ROS_INFO("label 1: %s", map->data[0].layout.dim[1].label.c_str());

  visualization_msgs::Marker marker;
  marker.header.frame_id = "t265_odom_frame";
  marker.header.stamp = ros::Time();
  
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.8;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!

  int x_offset = (map->info.pose.position.x);
  int y_offset = (map->info.pose.position.y);
  
  double start_x, start_y, goal_x, goal_y;
  std::cout << "Enter start & goal: \n";
  std::cin  >> start_x >> start_y >> goal_x >> goal_y;
  
  node start(start_x, start_y, map_res, x_offset, y_offset, map->info.length_x, map->info.length_y);
  
  marker.id = 0;
  marker.pose.position.x = start.x;
  marker.pose.position.y = start.y;
  marker.color.r = 1.0;
  marker.color.g = 0.0;

  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  arrow_pub.publish( marker );
  
  node goal(goal_x, goal_y, map_res, x_offset, y_offset, map->info.length_x, map->info.length_y);
  marker.id = 1;
  marker.pose.position.x = goal.x;
  marker.pose.position.y = goal.y;
  marker.color.r = 0.0;
  marker.color.g = 1.0;

  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  arrow_pub.publish( marker );

  a_start(start, goal, map, plan_pub);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_3d"); // Node Initialization
  ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("/octomap_to_gridmap_demo/grid_map", 1000, mapCallback);
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>( "points_visualization", 0 );
  ros::Publisher plan_pub = n.advertise<nav_msgs::Path>("path_3D", 1000);
  
  // https://answers.ros.org/question/12970/how-to-use-nodehandle-in-callback/
  ros::Subscriber sub = n.subscribe<grid_map_msgs::GridMap>("/octomap_to_gridmap_demo/grid_map", 
      1000, boost::bind(&mapCallback, boost::ref(plan_pub), boost::ref(marker_pub) , _1));
  
  ROS_INFO("LOL Started...");
  ros::spin();

  return 0;
}