#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <vector>
#include <deque>
#include <algorithm>

/*
/////////////////////////////////////
/////////CODE NOT COMPLETE!//////////
/////////////////////////////////////
*/

struct node
{
    int row;
    int col;
    int pos;

    float g; // Number of cells from the start node
    float h; // Distance from node to goal
    float height; //height data in the cell
    float f; // cost function

    node *parent_node;

    bool visited;

    node(int r, int c)
    {
      row = r;
      col = c;
    }

    node(int r, int c, float *height_array, const int r_num, const int r_stride, 
      const int c_num, const int c_stride)
    {
      row = r;
      col = c;
      height = height_val;

      //Calculate position in array
      if(row >=0 && row <r_num && col >=0 && col < c_num)
      {
        // multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
        pos = (r_stride*row) + (c_stride*col);
        height = height_array[pos];
      }
      else
        pos = -1; // Means oustide of map
    }

    float calc_f(node parent, node goal)
    {
      h = pow(goal.row - row, 2) + pow(goal.col - col, 2); 
      height *= 1;

      f = g + h + height;  
      return f;
    }
};

bool node_sort (node i,node j) { return (i.g)<j.g; }

bool send_path()
{
  return 0;
}

int is_in_list(std::deque<node> list, node cell)
{
  for(int c=0; c<list.size(); c++)
    if(list[c].pos == cell.pos)
      return c;
  return -1;
}

bool a_start(const node start, const node goal, const float *height_data, 
    const int col_num, const int col_stride, const int row_num, const int row_stride)
{
  node children[8] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                    {0, 1}, {0, -1}, {1, 0}, {-1, 0}};

  std::deque<node> open_list, closed_list;

  open_list.push_back(start);

  while(open_list.size())
  {
    std::sort(open_list.begin(), open_list.end(), node_sort);

    node current = open_list[0];
    open_list.pop_front();

    closed_list.push_back(current);

    if(current.row == goal.row && current.col == goal.col)
    {
      ROS_INFO("Plan found");
      
      if(send_path())
        ROS_INFO("Plan published");
      else
        ROS_INFO("Failed to publish plan");
      
      break;
    }

    for(auto c:children)
    {
        node child(current.row+c.row, current.col+c.col);
        
        if(child.pos >= 0) //That means it is within the map
        {
          child.height = height_data[child.pos];
          
          int pos_in_list = is_in_list(closed_list, child);
          if(pos_in_list == -1)
          {
            child.calc_f(current, goal);

            pos_in_list = is_in_list(open_list, child);
            if(pos_in_list == -1 || (pos_in_list >= 0 && open_list[pos_in_list].g > child.g))
              open_list.push_back(child);
          }
        }
    }
  }
}


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