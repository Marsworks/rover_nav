 #include <pluginlib/class_list_macros.h>
 #include <planner_3d.h>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(planner_3d::Planner3D, nav_core::BaseGlobalPlanner)

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

bool send_path(node *cell, node start, std::vector<geometry_msgs::PoseStamped>& poses_list)
{
    geometry_msgs::PoseStamped temp_pos;
  
    while(cell != NULL && ros::ok())
    { 
        temp_pos.pose.position.x = cell->position(0);
        temp_pos.pose.position.y = cell->position(1);
        temp_pos.pose.position.z = cell->height;

        poses_list.push_back(temp_pos);
        cell = cell->parent_node;
    }
    return 1;
}

bool node_sort (node i,node j) 
{ 
    return i.f < j.f; 
}

int is_in_list(std::deque<node> list, node cell)
{
    for(int c=0; c<list.size(); c++)
        if(list[c].index(0) == cell.index(0) && list[c].index(1) == cell.index(1))
            return c;
    return -1;
}


 //Default Constructor
 namespace planner_3d {
    grid_map::GridMap map;
    
    void  mapCallback(const grid_map_msgs::GridMap::ConstPtr& map_msg_ptr)
    {
        const grid_map_msgs::GridMap map_msg = *map_msg_ptr;
        grid_map::GridMapRosConverter::fromMessage(map_msg, map);
    }

    Planner3D::Planner3D (){}

    Planner3D::Planner3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }

 
    void Planner3D::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        n = ros::NodeHandle("~/" + name);

        marker_pub = n.advertise<visualization_msgs::Marker>( "points_visualization", 0 );
    
        sub = n.subscribe<grid_map_msgs::GridMap>("/octomap_to_gridmap_demo/grid_map", 1000, mapCallback);

        // marker.header.frame_id = "t265_odom_frame";
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
    
        ROS_WARN("planner_3D Initiallised");
    }

    bool Planner3D::makePlan(const geometry_msgs::PoseStamped& start_pos, const geometry_msgs::PoseStamped& goal_pos,  std::vector<geometry_msgs::PoseStamped>& plan )
    {
        ros::Time time;
        time = ros::Time::now();

        int cntr = 0;
        ROS_INFO("Finding path...");

        grid_map::Position pos_vector;

        pos_vector(0) = start_pos.pose.position.x;
        pos_vector(1) = start_pos.pose.position.y;
        node start(pos_vector, map);

        pos_vector(0) = goal_pos.pose.position.x;
        pos_vector(1) = goal_pos.pose.position.y;
        node goal(pos_vector, map);
        
        std::deque<node> open_list, closed_list;

        open_list.push_back(start);
        
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
                
                if(send_path(current, start, plan))
                {
                    ROS_INFO("Plan published");
                    ROS_INFO("Total time: %f", (ros::Time::now()-time).toSec());
                    return 1;
                }
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
        ROS_INFO("Total time: %f", (ros::Time::now()-time).toSec());
        return 0;    
    }
 };