#!/usr/bin/env python
from waypoints_dict import waypoints

import rospy
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Trigger

class PlanExecutor():
    
    def __init__(self):

        # Initialize node
        rospy.init_node("move_to_start", anonymous=False)
        rospy.loginfo("Moving turtlebot to starting position")

        # Set initial pos
        self.init_pos = PoseWithCovarianceStamped()
        self.init_pos.pose.pose.position.x = 13.4004869461
        self.init_pos.pose.pose.position.y = 18.36510849
        self.init_pos.pose.pose.position.z = 0.0
        self.init_pos.pose.pose.orientation.x = 0.0
        self.init_pos.pose.pose.orientation.y = 0.0
        self.init_pos.pose.pose.orientation.z = -0.988785192633
        self.init_pos.pose.pose.orientation.w = 0.149344711419
        self.init_pos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, \
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, \
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

        self.init_pos_pub.publish(self.init_pos)
        self.rate.sleep()

        # Wait for action services
        rospy.wait_for_service("move_to_start")
        rospy.wait_for_service("dock")

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