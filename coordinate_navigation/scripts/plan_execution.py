#!/usr/bin/env python
from subprocess import call
from waypoints_dict import waypoints

import rospy
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class PlanExecutor():
    
    def __init__(self):

        # Initialize node
        rospy.init_node("move_to_start", anonymous=False)
        rospy.loginfo("Moving turtlebot to starting position")

        # Wait for action services
        rospy.wait_for_service("move_to_start")
        rospy.wait_for_service("dock")

        self.start_action()

    def init_pos_recieved_handler(self, msg):
        if (msg.pose.pose.position.x >= 12.5 and msg.pose.pose.position.x <= 14.5) and \
            (msg.pose.pose.position.y >= 17.5 and msg.pose.pose.position.y <= 19.5):
            self.init_pos_flag = True
            self.init_pos_recieved.unregister()
            rospy.loginfo("Initial position recieved")

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