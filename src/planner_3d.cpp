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

    node(){}

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

        f = g + h * 50 + height * 40;
        //ROS_INFO("Cost function; g: %.2f, h: %.2f, height_cost:%.2f & f: %.2f", g, h, height_cost, f);
    }
};

bool send_path(node *cell, node start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &poses_list)
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

        // ROS_INFO("Current point, %f %f", cell->position(0), cell->position(1));

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
grid_map::GridMap map;
grid_map_msgs::GridMap grid_map_message;
ros::Publisher grid_map_publisher;

void mapCallback(const octomap_msgs::Octomap::ConstPtr &ocotomap_msg_ptr)
{
    ROS_INFO("New grid_map received");
    octomap::OcTree *octomap = nullptr;
    octomap_msgs::Octomap ocotomap_msg = *ocotomap_msg_ptr;
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

    bool res = grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", map, &min_bound, &max_bound);

    if (res)
    {
        map.setFrameId(ocotomap_msg.header.frame_id);
        grid_map::GridMapRosConverter::toMessage(map, grid_map_message);
        grid_map_publisher.publish(grid_map_message);
        ROS_INFO("Octomap converted to grid_map");
    }
    else
    {
        ROS_ERROR("Failed to call convert Octomap.");
        return;
    }
}

Planner3D::Planner3D() { ROS_WARN("Initialised!"); }

Planner3D::Planner3D(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
}

void Planner3D::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    ROS_INFO("Initiallising...");

    n = ros::NodeHandle("~/" + name);

    marker_pub = n.advertise<visualization_msgs::Marker>("points_visualization", 0);
    grid_map_publisher = n.advertise<grid_map_msgs::GridMap>("empty_grid_map", 0);
    octomap_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_full", 10, mapCallback);

    // grid_map::Matrix empty_world = grid_map::Matrix::Constant(800, 800, 0);
    // full_map.setFrameId("t265_odom_frame");
    // full_map.setGeometry(grid_map::Length(80, 80), 0.1, grid_map::Position::Zero());
    // full_map.add("elevation", empty_world);
    // std::cout << full_map["elevation"];
    // grid_map::GridMapRosConverter::toMessage(full_map, grid_map_message);

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

bool Planner3D::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO("Finding path...");
    // marker.type = visualization_msgs::Marker::DELETEALL;
    // arrow_pub.publish(marker);
    // map["elevation"] += full_map["elevation"];
    ros::Time time;
    time = ros::Time::now();

    int cntr = 0;

    grid_map::Position pos;
    node start_pos, goal_pos;

    pos = grid_map::Position(start.pose.position.x, start.pose.position.y);
    if (map.isInside(pos))
        start_pos = node(pos, map);
    else
    {
        ROS_WARN("Robot starting pose is outside of the map, can't find a path");
        return false;
    }

    pos = grid_map::Position(goal.pose.position.x, goal.pose.position.y);
    if(map.isInside(pos))
        goal_pos = node(pos, map);
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

        marker_pub.publish(marker);

        open_list.pop_front();
        closed_list.push_back(*current);

        if (current->index(0) == goal_pos.index(0) && current->index(1) == goal_pos.index(1)) // Index has been choosen instead of position to avoid comparing floats
        {
            ROS_INFO("Plan found");

            if (send_path(current, start_pos, goal, plan))
            {
                ROS_INFO("Plan published");
                ROS_INFO("Total time: %f", (ros::Time::now() - time).toSec());
                return true;
            }
        }

        for (auto c : children)
        {
            grid_map::Index temp_pos(current->index(0) + c(0), current->index(1) + c(1));
            node child = node(temp_pos, map);

            int pos_in_list = is_in_list(closed_list, child);

            if (map.isValid(temp_pos) && pos_in_list == -1) // Checking if the index is within the map
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