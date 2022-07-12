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

        # Wait for action services
        rospy.wait_for_service("move_to_start")
        rospy.wait_for_service("dock")
        rospy.wait_for_service("move")

        rospy.loginfo("All services running")

        self.start_action()
        rospy.loginfo(waypoints["lab_door_in"])
        self.move_action(waypoints["lab_door_in"])

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

    def move_action(self, goal_pose):
        rospy.loginfo("In move_action")
        # Call to service
        goal = Move()
        goal.final_pose = goal_pose[0]
        goal.final_orientation = goal_pose[1]
        rospy.loginfo("In move_action, after Move object created")
        try:
            move_tb = rospy.ServiceProxy("move", Move)
            rospy.loginfo("In move_action, Service proxy created")
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