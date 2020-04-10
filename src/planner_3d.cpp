#include <pluginlib/class_list_macros.h>
#include <planner_3d.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(planner_3d::Planner3D, nav_core::BaseGlobalPlanner)

struct node
{
    grid_map::Position position; // position in the odom frame (x, y) m
    grid_map::Index index;       // index in the map 2D array

    double g = 0;  // Number of cells from the start node
    double h;      // Distance from node to goal
    double f;      // cost function
    double height; //height data in the cell

    node *parent_node = NULL;

    node() {}

    node(grid_map::Position pos, const grid_map::GridMap &map)
    {
        position = pos;
        if (map.isInside(position))
        {
            map.getIndex(position, index);
            height = map.atPosition("elevation", position);
            height = (std::isfinite(height)) ? height : 0;
        }

        // ROS_INFO("Special2 row: %d, col: %d, res: %f", row, col, res);
    }

    node(grid_map::Index ind, const grid_map::GridMap &map)
    {
        index = ind;
        if (map.isValid(index))
        {
            map.getPosition(index, position);

            height = map.atPosition("elevation", position);
            height = (std::isfinite(height)) ? height : 0;
        }
        else
            height = 0;
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

        f = g + h * 50 + height * 40;
        //ROS_INFO("Cost function; g: %.2f, h: %.2f, height_cost:%.2f & f: %.2f", g, h, height_cost, f);
    }
};

bool get_path(node *cell, node start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &poses_list)
{
    geometry_msgs::PoseStamped temp_pos = goal;
    tf2::Quaternion goal_quat;
    int cntr = 0;
    goal_quat.setRPY(0, 0, 0);

    temp_pos.header.frame_id = "t265_odom_frame";
    while (cell != NULL && ros::ok())
    {
        temp_pos.pose.position.x = cell->position(0);
        temp_pos.pose.position.y = cell->position(1);
        // temp_pos.pose.position.z = cell->height;

        temp_pos.pose.orientation.x = goal_quat.x();
        temp_pos.pose.orientation.y = goal_quat.y();
        temp_pos.pose.orientation.z = goal_quat.z();
        temp_pos.pose.orientation.w = goal_quat.w();

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
    // std::cout << "name " << ros::this_node::getName() << std::endl; // /move_base
    // std::cout << "namespace " << ros::this_node::getNamespace()<< std::endl; // /

    marker_publisher = n.advertise<visualization_msgs::Marker>("explored_cells", 0);
    grid_map_publisher = n.advertise<grid_map_msgs::GridMap>("ocotomap_2_gridmap", 0);
    full_map_publisher = n.advertise<grid_map_msgs::GridMap>("full_gridmap", 0);
    filtered_map_publisher = n.advertise<grid_map_msgs::GridMap>("filtered_gridmap", 0);

    client = n.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");

    if (!map_filter.configure("/grid_map_filters", n))
    {
        ROS_ERROR("Could not configure the filter chain.");
        return;
    }
    else
        ROS_INFO("Map filter chain configured");

    full_gridmap.setGeometry(grid_map::Length(40, 80), 0.1, grid_map::Position::Zero());
    full_gridmap.add("elevation", grid_map::Matrix::Constant(400, 800, 0.63));
    full_gridmap.setBasicLayers({"elevation"});
    full_gridmap.setFrameId("t265_odom_frame");

    marker.header.stamp = ros::Time();
    marker.header.frame_id = "t265_odom_frame";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    ROS_INFO("planner_3D Initiallised");
}

void Planner3D::mapCallback(const octomap_msgs::Octomap &ocotomap_msg)
{
    ROS_INFO("New grid_map received");

    octomap::OcTree *octomap = nullptr;
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(ocotomap_msg);

    if (tree)
        octomap = dynamic_cast<octomap::OcTree *>(tree);
    else
    {
        ROS_ERROR("Failed to call convert Octomap.");
        return;
    }

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;

    octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
    octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

    // min_bound(0) = -40; // min x
    // max_bound(0) = 40; // max x
    // min_bound(1) = -40; // min y
    // max_bound(1) = 40; // max y
    // min_bound(2) = -2; // min z
    // max_bound(2) = 2; // max z

    bool ret = grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", raw_gridmap, &min_bound, &max_bound);

    if (ret)
    {
        raw_gridmap.setFrameId(ocotomap_msg.header.frame_id);
    
        grid_map::GridMapRosConverter::toMessage(raw_gridmap, grid_map_message);
        grid_map_publisher.publish(grid_map_message);

        ROS_INFO("Octomap converted to grid_map");

        if (!map_filter.update(raw_gridmap, filtered_gridmap)) 
        {
            ROS_ERROR("Could not update the grid map filter chain!");
            return;
        }

        grid_map::GridMapRosConverter::toMessage(filtered_gridmap, grid_map_message);
        filtered_map_publisher.publish(grid_map_message);

        ROS_INFO("GridMap filtered");
    }
    else
    {
        ROS_ERROR("Failed to call convert Octomap.");
        return;
    }
}

bool Planner3D::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    // Used to track the planner execution time
    ros::Time time;
    time = ros::Time::now();

    ROS_INFO("Finding path...");

    // Clear all the existing markers from the previous path
    marker.type = visualization_msgs::Marker::DELETEALL;
    marker_publisher.publish(marker);

    // Get the ocotomap from the octomap_server
    if (client.call(srv))
        Planner3D::mapCallback(srv.response.map);
    else
    {
        ROS_ERROR("Failed to call Octomap service: ");
        return false;
    }

    ROS_INFO("Adding raw_gridmap to full_gridmap...");
    // The function below is modified to speedup the data copying
    bool ret = full_gridmap.addDataFrom(raw_gridmap, false, true, "elevation");
    if (ret)
    {
        ROS_INFO("Added!");
        // grid_map::GridMapRosConverter::toMessage(full_gridmap, grid_map_message);
        // full_map_publisher.publish(grid_map_message);
    }
    else
    {
        ROS_ERROR("Failed");
        return false;
    }

    int cntr = 0;

    // gridmap to use for finding the path
    grid_map::GridMap &search_gridmap = full_gridmap;
    grid_map::Position pos;

    node start_pos, goal_pos;

    pos = grid_map::Position(start.pose.position.x, start.pose.position.y);
    if (search_gridmap.isInside(pos))
        start_pos = node(pos, search_gridmap);
    else
    {
        ROS_WARN("Robot starting pose is outside of the map, can't find a path");
        return false;
    }

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

    while (open_list.size() && ros::ok())
    {
        std::sort(open_list.begin(), open_list.end(), node_sort);

        node *current = new node(open_list[0]);

        marker.id = ++cntr;
        marker.pose.position.x = current->position.x();
        marker.pose.position.y = current->position.y();
        marker.pose.position.z = current->height;

        marker_publisher.publish(marker);

        open_list.pop_front();
        closed_list.push_back(*current);

        if (current->index(0) == goal_pos.index(0) && current->index(1) == goal_pos.index(1)) // Index has been choosen instead of position to avoid comparing floats
        {
            ROS_INFO("Plan found");

            if (get_path(current, start_pos, goal, plan))
            {
                ROS_INFO("Plan published");
                ROS_INFO("Total time: %f", (ros::Time::now() - time).toSec());
                return true;
            }
        }

        for (auto c : children)
        {
            grid_map::Index temp_index(current->index(0) + c(0), current->index(1) + c(1));
            node child = node(temp_index, search_gridmap);

            int pos_in_list = is_in_list(closed_list, child);

            if (search_gridmap.isValid(temp_index) && pos_in_list == -1) // Checking if the index is within the map
            {
                child.calc_f(current, goal_pos);

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