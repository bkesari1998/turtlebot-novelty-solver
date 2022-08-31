#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_circle():

    # Create a publisher which can "talk" to Turtlesim and tell it to move
    rospy.init_node("test")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
     
    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    move_cmd.angular.z = 1.0

    # Save current time and set publish rate at 10 Hz
    rate = rospy.Rate(10)

    # For the next 6 seconds publish cmd_vel move commands to Turtlesim
    while True:
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass