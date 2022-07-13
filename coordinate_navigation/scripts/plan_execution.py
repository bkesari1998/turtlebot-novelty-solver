#!/usr/bin/env python
from waypoints_dict import waypoints

import rospy

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from coffee_bot_srvs.srv import Move

class PlanExecutor():
    
    def __init__(self):

        # Initialize node
        rospy.init_node("plan_execution", anonymous=False)
        rospy.loginfo("plan_execution node active")


        # Wait for action services
        rospy.loginfo("Waiting for move_to_start service")
        rospy.wait_for_service("move_to_start")
        rospy.loginfo("Waiting for dock service")
        rospy.wait_for_service("dock")
        rospy.loginfo("Waiting for move service")
        rospy.wait_for_service("move")

        rospy.loginfo("All services running")

        self.start_action()
        self.move_action("lab_door_in")
        self.move_action("dock_in_view")
        self.dock_action()

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

    def move_action(self, loc):
        # Call to service
        try:
            move_tb = rospy.ServiceProxy("move", Move)
            response = move_tb(waypoints[loc][0], waypoints[loc][1])
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