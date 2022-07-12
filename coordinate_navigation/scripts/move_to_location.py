#!/usr/bin/env python 

from http import client
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from coffee_bot_srvs.srv import Move
from std_srvs.srv import Trigger
from waypoints_dict import waypoints

class MoveTB():
    def __init__(self):

        rospy.init_node("move_tb", anonymous=False)

        self.move_tb_srv = rospy.Service("/move", Trigger, self.move_tb)

        # Create a SimpleActionClient of a move_base action type and wait for server
        self.simple_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.simple_action_client.wait_for_server()

        self.goal = [(21.7725623016, 19.14204212, 0), (0, 0, -0.0778055926945, 0.996968550028)]

        while not rospy.is_shutdown():
            rospy.spin()

    def assign_goal(self, pose):
        
        # Create MoveBaseGoal object from input
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.x = pose[1][2]
        goal_pose.target_pose.pose.orientation.y = pose[1][3]

        return goal_pose

    def move_tb(self, req):

        # Assign the turtlebot's goal
        tb_goal = self.assign_goal(self.goal)
        rospy.loginfo("Goal assigned")
        self.simple_action_client.send_goal(tb_goal)
        self.simple_action_client.wait_for_result()

        if (self.simple_action_client.get_state() == GoalStatus.SUCCEEDED):
            return True, "Turtlebot successfully navigated to goal position"
        else:
            return False, "Turtlebot unable to navigate to goal position"

if __name__ == "__main__":
    try:
        MoveTB()
    except:
        rospy.logerr("MoveTB failed")