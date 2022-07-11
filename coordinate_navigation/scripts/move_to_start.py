#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class MoveToStart():
    
    def __init__(self):
        
        # Initialize node
        rospy.init_node("move_to_start", anonymous=False)
        rospy.loginfo("move_to_start service active")
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10) # publisher of Twist msg to robot
        self.rate = rospy.Rate(10) # publish at 1 Hz

        self.move_service = rospy.Service("move_to_start", Trigger, self.move_to_start) # service to start move commands

        while not rospy.is_shutdown():
            rospy.spin()

    def move_to_start(self, req):

        # reverse and rotate the turtlebot
        self.reverse()
        self.rotate()

        return True, "Turtlebot successfully moved to starting position"

    def reverse(self):

        # create msg to reverse turtlebot 0.5 m
        move_cmd = Twist()
        move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0

        for i in range(25):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def rotate(self):

        # create msg to rotate 180 degrees
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 3.1

        # publish msg
        for i in range(13):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

        # stop the turtlebot
        self.cmd_vel.publish(Twist())
        self.rate.sleep()

    def shutdown(self):
 
        rospy.loginfo("Stopping move_to_start node")
        
        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        self.rate.sleep()


if __name__ == "__main__":
    try:
        MoveToStart()
    except:
        rospy.logerr("MoveToStart failed")

