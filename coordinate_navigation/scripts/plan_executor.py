#!/usr/bin/env python
from waypoints_dict import waypoints

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class PlanExecutor():
    
    def __init__(self):

        # Initialize node
        rospy.init_node("move_to_start", anonymous=False)
        rospy.loginfo("Moving turtlebot to starting position")
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10) # publisher of Twist msg to robot
        self.rate = rospy.Rate(1) # publish at 1 Hz
        
        rospy.wait_for_service("move_to_start")
        rospy.wait_for_service("dock")

        try:
            move_to_start = rospy.ServiceProxy("move_to_start", Empty)
            response = move_to_start(Empty())
        except rospy.ServiceException as e:
            rospy.logerr(e)
        
        try:
            dock = rospy.ServiceProxy("dock", Empty)
            response = dock(Empty())
        except rospy.ServiceException as e:
            rospy.logerr(e)
    
    def shutdown(self):

        rospy.loginfo("Stopping plan_executor node")
        
        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        rospy.sleep()