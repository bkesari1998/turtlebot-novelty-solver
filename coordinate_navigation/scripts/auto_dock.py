#!/usr/bin/env python

import rospy
import os

from std_msgs.msg import Bool

def dock(msg):
    os.system("roslaunch kobuki_auto_docking activate.launch")

if __name__ == "__main__":

    rospy.init_node("auto_dock", anonymous=False)

    sub = rospy.Subscriber("/dock", Bool, callback=dock)