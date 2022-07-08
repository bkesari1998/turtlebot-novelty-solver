#!/usr/bin/env python
from waypoints_dict import waypoints

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def assign_goal(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

if __name__ == "__main__":
    rospy.init_node('move_tb_to_goal_points')

    # Create SimpleActionClient of a move_base action type and wait for server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # for each goal point in the list, call the action server and move to goal
    for tb_pose in goal_points:
        tb_goal = assign_goal(tb_pose)
        client.send_goal(tb_goal)
        client.wait_for_result()

        if client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("success")
        else:
            rospy.loginfo("failed")