#! /usr/bin/env python

import rospy
from actionlib import SimpleActionClient, SimpleGoalState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

def goal_feedback_cb(feedback):
    rospy.loginfo(feedback)

if __name__ == '__main__':
    rospy.init_node('navigation_goal_broadcaster')

    rospy.loginfo("Node started")

    client = SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()

    while not client.wait_for_server(timeout = rospy.Duration(2)):
        rospy.loginfo("Waiting for server")

    rospy.loginfo("Server found")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "t265_odom_frame"
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = -1
    goal.target_pose.pose.position.y = -1
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal, feedback_cb=goal_feedback_cb)
    

    # uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    # uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    # uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
    #                             #   and has since completed its execution (Terminal State)
    # uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    #                             #    to some failure (Terminal State)
    # uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
    #                             #    because the goal was unattainable or invalid (Terminal State)
    # uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
    #                             #    and has not yet completed execution
    # uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
    #                             #    but the action server has not yet confirmed that the goal is canceled
    # uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
    #                             #    and was successfully cancelled (Terminal State)
    # uint8 LOST            = 9

    while (client.get_state() == 0 or client.get_state() == 1) and not rospy.is_shutdown():
        rospy.loginfo(SimpleGoalState.to_string(client.get_state()))
        rospy.sleep(1)
    if client.get_state() == 3:
        rospy.loginfo("SUCCEEDED")
    elif client.get_state() == 4:
        rospy.loginfo("ABORTED")
    elif client.get_state() == 5:
        rospy.loginfo("REJECTED")
    else:
        rospy.loginfo(client.get_state())
