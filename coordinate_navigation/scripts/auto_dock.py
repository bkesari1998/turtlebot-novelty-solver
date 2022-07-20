#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


def dock_tb(req):
    """
    Service request handler.
    req: Trigger object.
    """

    # Run shell command to launch auto docking node
    os.system("roslaunch kobuki_auto_docking activate.launch")

    return True, "Turtlebot Docked"


def shutdown():
    """
    Called on node shutdown.
    """

    rospy.loginfo("Stopping auto_dock node")
    rospy.loginfo("Stopping turtlebot")

    # Create Twist msg publisher
    cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

    # Stop the turtlebot
    cmd_vel.publish(Twist())
    rospy.sleep(1)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("auto_dock", anonymous=False)
    rospy.loginfo("auto_dock node active")
    rospy.on_shutdown(shutdown)

    # Initialize service
    dock_srv = rospy.Service("/dock", Trigger, dock_tb)
    rospy.loginfo("dock service active")

    while not rospy.is_shutdown():
        rospy.spin()
