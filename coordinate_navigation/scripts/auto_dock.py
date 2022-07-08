#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Empty

def dock(msg):

    # Run shell command to launch auto docking node
    os.system("roslaunch kobuki_auto_docking activate.launch")

if __name__ == "__main__":

    rospy.init_node("auto_dock", anonymous=False)

    dock_srv = rospy.Service("/dock", Empty, callback=dock)

    while not rospy.is_shutdown():
        rospy.spin()
