#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/PoseStamped.h>

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
    node(double x_pos, double y_pos, const double res)
    {
      row = (int)round(x_pos/res);
      col = (int)round(y_pos/res);
      ROS_INFO("Special2 row: %d, col: %d, res: %f", row, col, res);
    }

    node(int r, int c)
    {
      row = r;
      col = c;
    }

    node(int r, int c, const float *height_array, const int r_num, const int r_stride, 
      const int c_num)
    {
      row = r;
      col = c;

      //Calculate position in array
      if(row >=0 && row <r_num && col >=0 && col < c_num)
      {
        // multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
        pos = (r_stride*row) + col;
        height = (std::isnan(height_array[pos]))? 0:height_array[pos];
        ROS_INFO("pos: %d, row: %d, col: %d, height: %f", pos, row, col, height);
      }
      else
      {
        pos = -1; // Means oustide of map
        ROS_INFO("Skipped; row: %d, col: %d",row, col);
      }
    }

    void calc_f(node *parent, node goal)
    {
      parent_node = parent;
      ROS_INFO("parent_node; row: %d, col: %d", parent_node->row, parent_node->col);
      if( parent_node->parent_node != NULL)
      ROS_INFO("parent_node->parent_node; row: %d, col: %d", 
          parent_node->parent_node->row, parent_node->parent_node->col);
      g = 1 + parent_node->g;
      h = pow(goal.row - row, 2) + pow(goal.col - col, 2); 
      height = 0;

      f = g + h + height;
      ROS_INFO("Cost function; g: %f, h: %f f: %f", g, h, f);
    }
};

bool send_path(node *cell, node start, double res, ros::Publisher &plan_pub)
{
  nav_msgs::Path plan;
  geometry_msgs::PoseStamped temp_pos;
  std::vector<geometry_msgs::PoseStamped> poses_list;
  ROS_INFO("dbg1");
  ROS_INFO("start row: %d, col: %d", start.row, start.col);
  
  while(cell != NULL && ros::ok())
  {
    //std::cin.get();

    ROS_INFO("Looping !; Cell address %d", cell);
    ROS_INFO("Looping !; Cell row: %d, col: %d", cell->row, cell->col);
    
    temp_pos.pose.position.x = (cell->row) * res;
    temp_pos.pose.position.y = (cell->col) * res;
    //temp_pos.pose.position.y = temp_node->height;

    poses_list.push_back(temp_pos);
    cell = cell->parent_node;
  }

  plan.header.frame_id = "map";
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
  for(int c=0; c<list.size(); c++)
    if(list[c].pos == cell.pos)
      return c;
  return -1;
}

bool a_start(const node start, const node goal, const grid_map_msgs::GridMap::ConstPtr& map, ros::Publisher &plan_pub)
{
  double map_res = map->info.resolution;

  int col_num = map->data[0].layout.dim[0].size; //num of columns
  int data_len = map->data[0].layout.dim[0].stride; //data len

  int row_num = map->data[0].layout.dim[1].size; //num of rows
  int row_stride = map->data[0].layout.dim[1].stride; // row stride

  const float *height_data = &map->data[0].data[0];

  std::deque<node> open_list, closed_list;
  
  node children[8] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                    {0, 1}, {0, -1}, {1, 0}, {-1, 0}};

  open_list.push_back(start);

  while(open_list.size() && ros::ok())
  {
    //std::cin.get();
    
    ROS_INFO("Looping!");
    std::sort(open_list.begin(), open_list.end(), node_sort);
  
    node *current = new node(open_list[0]);
    //current = &open_list[0];
    ROS_INFO("Current addres: %d", current);
    open_list.pop_front();
    //ROS_INFO("dbg2");
    closed_list.push_back(*current);

    if(current->row == goal.row && current->col == goal.col)
    {
      ROS_INFO("Plan found");
      
      if(send_path(current, start, map_res, plan_pub))
        ROS_INFO("Plan published");
      else
        ROS_INFO("Failed to publish plan");
      
      return 1;
    }

    for(auto c:children)
    {
      node child = node(current->row+c.row, current->col+c.col, 
            height_data, row_num, row_stride, col_num);
      
      if(child.pos >= 0) //That means it is within the map
      {
        child.height = height_data[child.pos];
        
        int pos_in_list = is_in_list(closed_list, child);
        if(pos_in_list == -1)
        {
          child.calc_f(current, goal);
          ROS_INFO("===============================================================");
          pos_in_list = is_in_list(open_list, child);
          if(pos_in_list == -1 || (pos_in_list >= 0 && open_list[pos_in_list].g > child.g))
          {
            ROS_INFO("//////////////////////////////////////////////////////////////");
            open_list.push_back(child);
          }
        }
      }
    }
  }
}


void mapCallback(ros::Publisher &plan_pub, const grid_map_msgs::GridMap::ConstPtr& map)
{
  ROS_INFO("Res: %f", map->info.resolution);
  ROS_INFO("size; x: %f, y:%f", map->info.length_x, map->info.length_y);
  
  double map_res = map->info.resolution;

  //int col_num = map->data[0].layout.dim[0].size; // ?????
  
  int col_num = map->data[0].layout.dim[0].size; //num of columns
  int data_len = map->data[0].layout.dim[0].stride; //data len

  int row_num = map->data[0].layout.dim[1].size; //num of rows
  int row_stride = map->data[0].layout.dim[1].stride; // row stride

  
  // std::vector<float> height_data = map->data[0].data;
  // for(auto i:height_data)
  //    ROS_INFO("%f", i);
  ROS_INFO("Res: %f",map_res);
  ROS_INFO("data.size(): %d", map->data[0].data.size());
  ROS_INFO("data_len: %d", data_len);

  ROS_INFO("Row; num: %d, stride: %d", row_num, row_stride);
  ROS_INFO("Column; num: %d, stride: %d", col_num);

  ROS_INFO("row size(): %d", map->data[0].data.size()); // ???
  ROS_INFO("col size(): %d", map->data[1].data.size()); // ???

  ROS_INFO("label 0: %s", map->data[0].layout.dim[0].label.c_str());
  ROS_INFO("label 1: %s", map->data[0].layout.dim[1].label.c_str());
  

  node start(0.0, 0.0, map_res);
  node goal(0.5, 0.5, map_res);

  a_start(start, goal, map, plan_pub);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_3d"); // Node Initialization
  ros::NodeHandle n;
  // https://answers.ros.org/question/12970/how-to-use-nodehandle-in-callback/
  //ros::Subscriber sub = n.subscribe("/octomap_to_gridmap_demo/grid_map", 1000, mapCallback);
  ros::Publisher plan_pub = n.advertise<nav_msgs::Path>("path_3D", 1000);
  ros::Subscriber sub = n.subscribe<grid_map_msgs::GridMap>("/octomap_to_gridmap_demo/grid_map", 1000, boost::bind(&mapCallback, boost::ref(plan_pub), _1));
  
  ROS_INFO("Started...");
  ros::spin();

  return 0;
}