#include <pluginlib/class_list_macros.h>
#include <planner_3d.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(planner_3d::Planner3D, nav_core::BaseGlobalPlanner)

struct node
{
    grid_map::Position position;  // position in the odom frame (x, y) m
    grid_map::Index index;  // index in the map 2D array

    double g = 0;  // Number of cells from the start node
    double h = 0;  // Distance from node to goal
    double f = 0;  // cost function
    double height = 0;  // height data in the cell
    double slope=0;

    bool traversable = true;

    node *parent_node = NULL;

    node() {}

    node(grid_map::Position pos, const grid_map::GridMap &map)
    {
        position = pos;
        if (map.isInside(position))
        {
            map.getIndex(position, index);
            height = map.at("elevation", index);
            height = (std::isfinite(height)) ? height : 0;
        }
    }

    node(grid_map::Index ind, const grid_map::GridMap &map, std::string elevation_layer, std::string slope_layer)
    {
        index = ind;
        if (map.isValid(index, elevation_layer))
        {
            map.getPosition(index, position);

            height = map.at("elevation", index);
            height = (std::isfinite(height)) ? height : 0;

            slope = map.at(slope_layer, index);
            
            ROS_INFO("Slope: %0.3f", slope);

            // Marking cell with a slope of bigger 45 deg as non-traversable
            if(abs(slope) > 1.07)
                traversable = false;
        }
        else
            ROS_WARN_STREAM("Invalid index (" << ind[0] << ", " << ind[1] << ")");
    }

    node(grid_map::Index ind)
    {
        index = ind;
    }

    void calc_f(node *parent, node goal, double k[])
    {
        parent_node = parent;
        g = 1 + parent_node->g;

        h = pow(goal.index(0) - index(0), 2) + pow(goal.index(1) - index(1), 2);

        f = k[0]*(g + h) + k[3]*slope;
        // ROS_INFO("Cost function; g: %.2f, h: %.2f, f: %.2f", g , h, f);
    }
};

bool get_path(node *cell, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &poses_list)
{
    geometry_msgs::PoseStamped temp_pos = goal;

    while (cell != NULL && ros::ok())
    {
        temp_pos.pose.position.x = cell->position(0);
        temp_pos.pose.position.y = cell->position(1);
        temp_pos.pose.position.z = cell->height;

        poses_list.push_back(temp_pos);
        cell = cell->parent_node;
    }
    std::reverse(poses_list.begin(), poses_list.end());
    return true;
}

bool node_sort(node i, node j)
{
    return i.f < j.f;
}

int is_in_list(std::deque<node> list, node cell)
{
    for (int c = 0; c < list.size(); c++)
        if (list[c].index(0) == cell.index(0) && list[c].index(1) == cell.index(1))
            return c;
    return -1;
}

//Default Constructor
namespace planner_3d
{

Planner3D::Planner3D() { ROS_WARN("Initialised!");}

Planner3D::Planner3D(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
}

void Planner3D::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    ROS_INFO("Initiallising...");

    n = ros::NodeHandle("~/" + name); // name: Planner3D

    marker_publisher = n.advertise<visualization_msgs::Marker>("/explored_cells", 1, true);
    full_map_publisher = n.advertise<grid_map_msgs::GridMap>("/full_gridmap", 1, true);
    filtered_map_publisher = n.advertise<grid_map_msgs::GridMap>("/filtered_gridmap", 1, true);

    global_map_subscrber = n.subscribe<grid_map_msgs::GridMap>("/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud", 1, &Planner3D::gridmap_callback, this);

    grid_map_initialized = false;

    // Configuring the filter chain
    if (!map_filter.configure("/grid_map_filters", n))
    {
        ROS_ERROR("Could not configure the filter chain.");
        return;
    }
    else
        ROS_INFO("Map filter chain configured");

    marker.header.frame_id = "map";
    marker.ns = "cubes";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    factors[0] = 1; // g+h factor
    factors[1] = 1; // not used
    factors[2] = 1; // elevation factor; not used
    factors[3] = 1; // slope factor 

    elevation_layer = "elevation";
    slope_layer = "elevation";

    ROS_INFO("planner_3D initiallised");
}

void Planner3D::gridmap_callback(const grid_map_msgs::GridMapConstPtr &grid_map_msg_ptr)
{
    ROS_INFO("Global grid_map received");
    
    grid_map::GridMapRosConverter::fromMessage(*grid_map_msg_ptr, raw_gridmap);
    grid_map_initialized = true;

    // grid_map::GridMapRosConverter::toMessage(raw_gridmap, grid_map_message);
    // full_map_publisher.publish(grid_map_message);

    // Filter grid map
    ROS_INFO("Filtering raw GridMap...");
    if (!map_filter.update(raw_gridmap, filtered_gridmap)) 
    {
        ROS_ERROR("Could not update the grid map filter chain!");
        return;
    }

    grid_map::GridMapRosConverter::toMessage(filtered_gridmap, grid_map_message);
    filtered_map_publisher.publish(grid_map_message);

    ROS_INFO("GridMap filtered");
}

bool Planner3D::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    if(!grid_map_initialized)
        return false;

    // Used to track the planner execution time
    ros::Time time;
    time = ros::Time::now();

    ROS_INFO("Finding path...");
    ROS_INFO("layer used: %s", slope_layer.c_str());

    // Clear all the existing markers from the previous path
    marker.type = visualization_msgs::Marker::DELETEALL;
    marker_publisher.publish(marker);

    // gridmap to use for finding the path
    // grid_map::GridMap &search_gridmap = full_gridmap; //full_gridmap, raw_gridmap or filtered_gridmap
    grid_map::GridMap search_gridmap = raw_gridmap;
    
    node start_pos, goal_pos;
    grid_map::Position pos;

    // Validating that the starting pose is within the map
    pos = grid_map::Position(start.pose.position.x, start.pose.position.y);
    if (search_gridmap.isInside(pos))
        start_pos = node(pos, search_gridmap);
    else
    {
        ROS_WARN("Robot starting pose is outside of the map, can't find a path");
        return false;
    }

    // Validating that the goal pose is within the map
    pos = grid_map::Position(goal.pose.position.x, goal.pose.position.y);
    if (search_gridmap.isInside(pos))
        goal_pos = node(pos, search_gridmap);
    else
    {
        ROS_WARN("Goal pose is outside of the map, can't find a path");
        return false;
    }

    std::deque<node> open_list, closed_list;

    open_list.push_back(start_pos);

    int cntr = 0;
    
    while (open_list.size() && ros::ok())
    {
        std::sort(open_list.begin(), open_list.end(), node_sort);

        node *current = new node(open_list[0]);

        open_list.pop_front();
        closed_list.push_back(*current);
        
        marker.id = ++cntr;
        marker.pose.position.x = current->position.x();
        marker.pose.position.y = current->position.y();
        marker.pose.position.z = current->height;
        marker_publisher.publish(marker);

        if (current->index(0) == goal_pos.index(0) && current->index(1) == goal_pos.index(1)) // Index has been choosen instead of position to avoid comparing floats
        {
            ROS_INFO("Plan found");

            if (get_path(current, goal, plan))
            {
                ROS_INFO("Plan published");
                ROS_INFO("Total time: %f", (ros::Time::now() - time).toSec());
                return true;
            }
        }

        for (auto c : children)
        {
            grid_map::Index temp_index(current->index(0) + c(0), current->index(1) + c(1));
            node child = node(temp_index, search_gridmap, elevation_layer, slope_layer);

            int pos_in_list = is_in_list(closed_list, child);
            if (search_gridmap.isValid(temp_index, "elevation") && pos_in_list == -1 && child.traversable) // Checking if the index is within the map
            {
                child.calc_f(current, goal_pos, factors);

                pos_in_list = is_in_list(open_list, child);

                if (pos_in_list == -1 || (pos_in_list >= 0 && open_list[pos_in_list].g > child.g))
                    open_list.push_back(child);
            }
        }
    }

    ROS_INFO("No plan found");
    ROS_INFO("Total time: %f", (ros::Time::now() - time).toSec());

    return false;
}
}; // namespace planner_3d