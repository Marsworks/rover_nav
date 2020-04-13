#include <ros/ros.h>
#include <planner_3d.h>
#include <rover_nav/plannerConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>

#include <vector>
#include <iostream>

void callback(rover_nav::plannerConfig& config, planner_3d::Planner3D& planner, ros::Publisher& path_pub)
{
    ROS_INFO("Reconfiguring planner..");

    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped start, goal;

    planner.factors[0] = config.g_gain;
    planner.factors[1] = config.h_gain;
    planner.factors[2] = config.elevation_gain;
    planner.factors[3] = config.slope_gain;

    start.pose.position.x = config.start_x;
    start.pose.position.y = config.start_y;

    goal.pose.position.x = config.goal_x;
    goal.pose.position.y = config.goal_y;

    nav_msgs::Path my_path;

    my_path.header.frame_id = "t265_odom_frame";

    if(planner.makePlan(start, goal, path))
    {
        my_path.poses = path;
        path_pub.publish(my_path);
    }

    ROS_INFO("Callback done");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_testing_node");
    ros::NodeHandle nh("planner_tuner");

    costmap_2d::Costmap2DROS *dummy_costmap;
    planner_3d::Planner3D planner("Planner3D", dummy_costmap);
    
    dynamic_reconfigure::Server<rover_nav::plannerConfig> server;
    dynamic_reconfigure::Server<rover_nav::plannerConfig>::CallbackType f;
    
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 0);

    f = boost::bind(&callback, _1, boost::ref(planner), boost::ref(path_pub));
    server.setCallback(f);

    ros::spin();

    return 0;
}