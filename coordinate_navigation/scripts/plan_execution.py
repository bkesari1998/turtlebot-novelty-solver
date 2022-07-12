#!/usr/bin/env python
from waypoints_dict import waypoints

import rospy

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from coffee_bot_srvs.srv import Move

class PlanExecutor():
    
    def __init__(self):

        # Initialize node
        rospy.init_node("move_to_start", anonymous=False)
        rospy.loginfo("Moving turtlebot to starting position")

        # Wait for action services
        rospy.wait_for_service("move_to_start")
        rospy.wait_for_service("dock")
        rospy.wait_for_service("move_tb")

        self.start_action()


    def start_action(self):

        # Call to service
        try:
            move_to_start = rospy.ServiceProxy("move_to_start", Trigger)
            response = move_to_start()
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)
        
    def dock_action(self):

        # Call to service
        try:
            dock = rospy.ServiceProxy("dock", Trigger)
            response = dock()
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def move_action(self, pose, orientation):

        # Call to service
        goal = Move()
        goal.final_pose = pose
        goal.final_orientation = orientation
        try:
            move_tb = rospy.ServiceProxy("move_tb", Move)
            response = move_tb(goal)
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def shutdown(self):

        rospy.loginfo("Stopping plan_executor node")
        
        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    
    try:
        PlanExecutor()
    except:
        rospy.logerr("PlanExecutor failed")