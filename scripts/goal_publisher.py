#! /usr/bin/env python

import rospy
from actionlib import SimpleActionClient, SimpleGoalState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == '__main__':
    rospy.init_node('navigation_goal_broadcaster')

    rospy.loginfo("Node started")

    client = SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()
    ret = client.wait_for_server(timeout = rospy.Duration(2))
    rospy.loginfo("Server found: " + str(ret))

    if ret:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "t265_odom_frame"
        goal.target_pose.header.stamp = rospy.get_rostime()

        goal.target_pose.pose.position.x = -4
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        
        while client.get_state() != SimpleGoalState.DONE and not rospy.is_shutdown():
            rospy.loginfo(SimpleGoalState.to_string(client.get_state()))
            rospy.sleep(1)
        
        rospy.loginfo("SUCCEEDED")

