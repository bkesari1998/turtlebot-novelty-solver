#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Trigger

def dock_tb(req):
            
    # Run shell command to launch auto docking node
    os.system("roslaunch kobuki_auto_docking activate.launch")

    return True, "Turtlebot Docked"

if __name__ == "__main__":

    rospy.init_node("auto_dock", anonymous=False)

    dock_srv = rospy.Service("/dock", Trigger, dock_tb)

    while not rospy.is_shutdown():
        rospy.spin()
